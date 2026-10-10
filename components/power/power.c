#include "power.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "config.h"
#include "events.h"
#include "imu.h"
#include "update.h"

#define TAG "power"

#define TICK_MS          100
#define ADC_SAMPLES      8
#define V_MIN_MV         7000       // вне 7–16 В делитель не подключён или неисправен: обрыв нижнего
#define V_MAX_MV         16000      // плеча упирает АЦП в потолок, это около 17 В в пересчёте
#define RISE_MV          400        // рост над уровнем стоянки — двигатель запустили
// Делителю доверяем, только если он в этом включении совпал с эталоном в пределах 10 %. Эталон —
// напряжение от блока управления машины (PID 42), а если машина его не отдаёт — от ELM327 (ATRV).
// Без доверия на стоянку и в сон не уходим: блок было бы нечем разбудить.
#define VERIFY_RATIO     0.10f
#define MISMATCH_US      30000000   // расхождение дольше 30 с — событие, до перезагрузки делителю не верим
// Калибровка: по тому же эталону делитель подгоняется поправочным коэффициентом.
#define CAL_SAMPLES      12         // замеры раз в 5 с на стоящей машине — минута
#define CAL_DIFF         0.01f      // новый коэффициент берётся, если отличается больше чем на 1 %
#define CAL_ELM          1
#define CAL_ECU          2
#define SAVE_MIN_US      600000000  // во флеш — первый коэффициент сразу, изменившийся не чаще раза в 10 мин
#define SLEEP_PERIOD_US  2000000    // во сне напряжение проверяется раз в 2 с
#define SLEEP_AWAKE_MS   10         // и на это уходит около 10 мс бодрствования
#define LOW_MV           11800      // LOW_VOLTAGE
#define LOW_REARM_MV     12200
#define LOW_US           60000000
#define NVS_KEY          "vcal"

typedef struct {
    int32_t k_milli;    // поправочный коэффициент делителя, 0 — не калиброван
    int32_t k_src;      // CAL_ELM или CAL_ECU
} cal_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static power_status_t s_st = {.radio = true, .adc_mv = -1, .k_milli = 1000};

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_chan;
static float s_adc_mv = -1;     // сглаженное напряжение сети по АЦП без поправки, -1 пока нет
static float s_k = 1.0f;        // поправочный коэффициент делителя
static volatile bool s_radio_on = true;

// ---- АЦП ----

static void adc_init(void)
{
#if CONFIG_NOGPS_VBAT_GPIO >= 0
    adc_unit_t unit;
    if (adc_oneshot_io_to_channel(CONFIG_NOGPS_VBAT_GPIO, &unit, &s_chan) != ESP_OK || unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "GPIO%d не вход ADC1, напряжение не измеряется", CONFIG_NOGPS_VBAT_GPIO);
        return;
    }
    adc_oneshot_unit_init_cfg_t ucfg = {.unit_id = unit};
    adc_oneshot_chan_cfg_t ccfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    adc_cali_curve_fitting_config_t cal = {
        .unit_id = unit,
        .chan = s_chan,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_oneshot_new_unit(&ucfg, &s_adc) != ESP_OK ||
        adc_oneshot_config_channel(s_adc, s_chan, &ccfg) != ESP_OK ||
        adc_cali_create_scheme_curve_fitting(&cal, &s_cali) != ESP_OK) {
        ESP_LOGE(TAG, "АЦП не запустился, напряжение не измеряется");
        s_adc = NULL;
        return;
    }
    ESP_LOGI(TAG, "напряжение сети: GPIO%d, делитель %d/%d Ом", CONFIG_NOGPS_VBAT_GPIO,
             CONFIG_NOGPS_VBAT_R_TOP, CONFIG_NOGPS_VBAT_R_BOTTOM);
#endif
}

// Напряжение сети без поправки, мВ; -1, если делителя нет или на входе не похоже на бортовую сеть.
static int adc_read(bool smooth)
{
    if (!s_adc) {
        return -1;
    }
    int sum = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        int raw, mv;
        if (adc_oneshot_read(s_adc, s_chan, &raw) != ESP_OK ||
            adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) {
            return -1;
        }
        sum += mv;
    }
    float pin = (float)sum / ADC_SAMPLES;
    float v = pin * (CONFIG_NOGPS_VBAT_R_TOP + CONFIG_NOGPS_VBAT_R_BOTTOM) / CONFIG_NOGPS_VBAT_R_BOTTOM;
    if (v < V_MIN_MV || v > V_MAX_MV) {
        s_adc_mv = -1;
        return -1;
    }
    s_adc_mv = s_adc_mv < 0 || !smooth ? v : s_adc_mv + 0.2f * (v - s_adc_mv);
    return (int)s_adc_mv;
}

// ---- коэффициент делителя в NVS ----

static void cal_load(cal_t *c)
{
    memset(c, 0, sizeof(*c));
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        size_t len = sizeof(*c);
        if (nvs_get_blob(h, NVS_KEY, c, &len) != ESP_OK || len != sizeof(*c)) {
            memset(c, 0, sizeof(*c));
        }
        nvs_close(h);
    }
}

static void cal_save(const cal_t *c)
{
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY, c, sizeof(*c));
        nvs_commit(h);
        nvs_close(h);
    }
}

// ---- стоянка и сон ----

// Уровень стоянки в памяти: вниз идёт за напряжением быстро (после остановки двигателя оно ещё
// несколько минут сползает), вверх — очень медленно, чтобы запуск двигателя был виден как скачок.
static void track_base(float *base, int v, float down, float up)
{
    *base += (v - *base) * (v < *base ? down : up);
}

// Лёгкий сон ESP32: процессор стоит, память цела, гироскоп усыплён. Раз в 2 с — проснуться, измерить
// напряжение, уснуть. Возвращает причину пробуждения.
static const char *sleep_until_wake(float *base)
{
    events_add('I', "SLEEP", "%d mV", (int)*base);
    imu_set_suspended(true);
    vTaskDelay(pdMS_TO_TICKS(500));
    const char *why = NULL;
    while (!why) {
        esp_sleep_enable_timer_wakeup(SLEEP_PERIOD_US);
        if (esp_light_sleep_start() != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SLEEP_PERIOD_US / 1000));
        }
        // Дать поработать остальным задачам и сторожевому таймеру.
        vTaskDelay(pdMS_TO_TICKS(SLEEP_AWAKE_MS));

        int nom = adc_read(false);
        int v = nom >= 0 ? (int)(nom * s_k) : -1;
        if (cfg_get_int("sleep_after_s") == 0 || cfg_get_int("wifi_off_s") == 0) {
            why = "saving off";
        } else if (v < 0) {
            why = "no voltage";
        } else if (v - *base >= RISE_MV) {
            why = "voltage rise";
        } else {
            track_base(base, v, 0.5f, 0.01f);   // те же постоянные времени в пересчёте на шаг 2 с
        }
    }
    imu_set_suspended(false);
    return why;
}

// ---- задача ----

static void power_task(void *arg)
{
    cal_t saved;        // то, что лежит во флеше
    cal_load(&saved);
    int k_src = saved.k_src;
    if (saved.k_milli) {
        s_k = saved.k_milli / 1000.0f;
        ESP_LOGI(TAG, "делитель: коэффициент %.3f (%s)", s_k, k_src == CAL_ECU ? "по машине" : "по ELM327");
    }

    bool parked = false, radio = true;
    bool verified = false, mismatch_reported = false, untrusted_logged = false, low_reported = false;
    int64_t keep_until = 0, parked_since = 0, mismatch_since = 0, low_since = 0, last_save = 0;
    uint32_t ref_seq = 0;
    float acc_sum = 0, base = 0;
    int acc_n = 0, acc_src = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TICK_MS));
        int64_t now = esp_timer_get_time();
        int off_s = cfg_get_int("wifi_off_s");

        obd_status_t obd;
        obd_get_status(&obd);
        int nom = adc_read(true);           // без поправки
        int v = nom >= 0 ? (int)(nom * s_k) : -1;

        // Проверка и калибровка делителя по эталону. Эталон приходит раз в 5 с, пока машина стоит.
        if (obd.volt_seq != ref_seq) {
            ref_seq = obd.volt_seq;
            int ref = obd.ecu_mv >= 0 ? obd.ecu_mv : obd.voltage_mv;
            int src = obd.ecu_mv >= 0 ? CAL_ECU : CAL_ELM;
            if (nom >= 0 && ref >= 0) {
                float r = (float)ref / nom;
                if (fabsf(r - 1) <= VERIFY_RATIO) {
                    verified = true;
                    mismatch_since = 0;
                    if (src != acc_src) {
                        acc_src = src;
                        acc_sum = 0;
                        acc_n = 0;
                    }
                    acc_sum += r;
                    if (++acc_n >= CAL_SAMPLES) {
                        float k = acc_sum / acc_n;
                        acc_sum = 0;
                        acc_n = 0;
                        // Калибровку по машине не заменять калибровкой по адаптеру.
                        bool worse = k_src == CAL_ECU && src == CAL_ELM;
                        if (!worse && (k_src == 0 || k_src != src || fabsf(k - s_k) > CAL_DIFF)) {
                            s_k = k;
                            k_src = src;
                            events_add('I', "VOLT_CAL", "k %ld ref %s %d mV", lroundf(k * 1000),
                                       src == CAL_ECU ? "ecu" : "elm", ref);
                        }
                    }
                } else if (mismatch_since == 0) {
                    mismatch_since = now;
                } else if (!mismatch_reported && now - mismatch_since >= MISMATCH_US) {
                    mismatch_reported = true;
                    events_add('W', "VOLT_MISMATCH", "adc %d %s %d mV", nom,
                               src == CAL_ECU ? "ecu" : "elm", ref);
                }
            }
        }
        bool trusted = v >= 0 && verified && !mismatch_reported;

        // Коэффициент во флеш: первый — сразу, изменившийся — не чаще раза в 10 минут.
        int k_milli = (int)lroundf(s_k * 1000);
        if (k_src && (saved.k_src == 0 || ((k_src != saved.k_src || abs(k_milli - (int)saved.k_milli) > 10) &&
                                           now - last_save >= SAVE_MIN_US))) {
            saved.k_milli = k_milli;
            saved.k_src = k_src;
            cal_save(&saved);
            last_save = now;
        }

        // Двигатель — только по машине. Узнать не у кого (OBD выключен, адаптер не найден или ещё
        // ищется) — радио не выключаем: блок должен работать и без ELM327.
        bool no_source = obd.state == OBD_STATE_DISABLED || obd.state == OBD_STATE_NO_ADAPTER ||
                         obd.state == OBD_STATE_INIT;
        bool running = obd.engine == OBD_ENGINE_RUN;
        obd_engine_t engine = running ? OBD_ENGINE_RUN : no_source ? OBD_ENGINE_UNKNOWN : OBD_ENGINE_OFF;

        if (!parked) {
            // Работа. Радио держится первую минуту после старта и минуту после остановки двигателя.
            // Прошивка после обновления ещё не подтверждена — тоже держим: к ней должен успеть
            // подключиться телефон.
            if (keep_until == 0 || running || no_source || update_hold_awake()) {
                keep_until = now + (int64_t)off_s * 1000000;
            }
            bool want = off_s == 0 || now < keep_until;
            if (want != radio) {
                radio = want;
                // Имена событий остались от Wi-Fi: на них написаны журналы и разборщики.
                events_add('I', want ? "WIFI_ON" : "WIFI_OFF", "%s rpm %d %d mV",
                           want ? "engine on" : "engine off", obd.rpm, v);
            }
            if (!radio && !trusted && !untrusted_logged) {
                untrusted_logged = true;
                ESP_LOGW(TAG, "делитель напряжения не подтверждён — на стоянку и в сон не ухожу, "
                              "жду двигатель по опросу машины");
            }
            // На стоянку — только если есть чем проснуться. Уровень засыпания — в памяти.
            if (!radio && trusted) {
                parked = true;
                parked_since = now;
                base = v;
                obd_set_paused(true);
                ESP_LOGI(TAG, "стоянка: OBD на паузе, жду роста напряжения на %d мВ над %d мВ", RISE_MV, v);
            }
        } else {
            // Стоянка: будит только рост напряжения.
            const char *why = NULL;
            int sleep_s = cfg_get_int("sleep_after_s");
            // sleep_after_s считается от остановки двигателя, стоянка началась на off_s позже.
            int64_t sleep_at = parked_since + (int64_t)(sleep_s > off_s ? sleep_s - off_s : 0) * 1000000;
            if (off_s == 0) {
                why = "saving off";
            } else if (!trusted) {
                why = "no voltage";
            } else if (v - base >= RISE_MV) {
                why = "voltage rise";
            } else if (sleep_s && now >= sleep_at && !s_radio_on) {
                why = sleep_until_wake(&base);
                now = esp_timer_get_time();
                nom = adc_read(false);
                v = nom >= 0 ? (int)(nom * s_k) : -1;
            } else {
                track_base(&base, v, 0.1f, 0.0005f);
            }
            if (why) {
                parked = false;
                radio = true;
                keep_until = now + (int64_t)off_s * 1000000;
                obd_set_paused(false);
                events_add('I', "WIFI_ON", "%s %d mV", why, v);
            }
        }

        // Просевший аккумулятор.
        if (v >= 0 && v < LOW_MV) {
            if (low_since == 0) {
                low_since = now;
            } else if (!low_reported && now - low_since >= LOW_US) {
                low_reported = true;
                events_add('W', "LOW_VOLTAGE", "%d mV", v);
            }
        } else {
            low_since = 0;
            if (v >= LOW_REARM_MV) {
                low_reported = false;
            }
        }

        taskENTER_CRITICAL(&s_lock);
        s_st.parked = parked;
        s_st.radio = radio;
        s_st.engine = engine;
        s_st.adc_mv = v;
        s_st.k_milli = k_milli;
        s_st.adc_trusted = trusted;
        s_st.base_mv = parked ? (int)base : 0;
        taskEXIT_CRITICAL(&s_lock);
    }
}

esp_err_t power_start(void)
{
    adc_init();
    return xTaskCreate(power_task, "power_task", 4096, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void power_radio_state(bool on)
{
    s_radio_on = on;
}

void power_get_status(power_status_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_st;
    taskEXIT_CRITICAL(&s_lock);
}
