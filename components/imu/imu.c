// Ядро IMU: автоопределение по реестру драйверов, чтение в imu_task, восстановление после ошибок
// (ТЗ IMU-2, IMU-9..IMU-12). Драйверы сами не логируют, всё логирование здесь.
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "events.h"
#include "imu.h"
#include "imu_bus.h"

#define TAG "imu"

#define READ_PERIOD_MS     10
#define FAIL_HOLD_US       100000   // IMU-11: ошибка шины или тишина должны продержаться 100 мс
#define FAIL_MIN_READS     3        // и не меньше трёх чтений подряд
#define REDETECT_MS        2000     // IMU-9
#define REINIT_MS          1000     // IMU-11
#define SLOW_ODR_HZ        150      // IMU-12
// Задача гироскопа проходит цикл раз в 10 мс. Если состояние OK, а она стоит дольше этого, отсчётов
// нет и курс не считается: наружу это должно быть видно.
#define STALL_MS           500
// Сразу после включения и после сна датчик может ещё не отвечать, а Wi-Fi и Bluetooth при запуске
// пишут во флеш и задерживают обмен. Первые неудачи — без события, с быстрым повтором.
#define START_TRIES        3
#define START_RETRY_MS     200
#define BATCH_MAX          64
// На двухъядерных чипах — ядро 1, отдельно от Wi-Fi; на одноядерных (ESP32-C3) — единственное.
#define IMU_CORE           (portNUM_PROCESSORS - 1)

static imu_config_t s_cfg;
static imu_sample_cb_t s_cb;
static void *s_cb_ctx;
static const imu_driver_t *s_drv;
static imu_status_t s_status = {.state = IMU_STATE_INIT, .chip_id = -1};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static imu_sample_t s_batch[BATCH_MAX];
static volatile bool s_suspended;
static volatile bool s_paused;
static volatile uint32_t s_alive_ms;    // когда imu_task последний раз прошла цикл

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

const char *imu_state_name(imu_state_t s)
{
    switch (s) {
    case IMU_STATE_INIT:       return "INIT";
    case IMU_STATE_NOT_FOUND:  return "NOT_FOUND";
    case IMU_STATE_WRONG_CHIP: return "WRONG_CHIP";
    case IMU_STATE_BUS_ERROR:  return "BUS_ERROR";
    case IMU_STATE_NO_DATA:    return "NO_DATA";
    case IMU_STATE_OK:         return "OK";
    }
    return "?";
}

void imu_get_status(imu_status_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_status;
    taskEXIT_CRITICAL(&s_lock);
    // Метка читается до часов и ещё раз после. Того, кто спрашивает, могут вытеснить между чтениями:
    // метка, обновлённая за это время, оказалась бы новее часов, и разность без знака стала бы
    // огромной. Так в 0.3.5 появлялось ложное «задача стоит».
    uint32_t alive = s_alive_ms;
    uint32_t now = now_ms();
    out->stalled = out->state == IMU_STATE_OK && alive == s_alive_ms && (int32_t)(now - alive) > STALL_MS;
    if (out->stalled) {
        out->state = IMU_STATE_NO_DATA;
    }
}

// Смена состояния — событие для журнала (ТЗ, раздел 10). Повторы того же состояния не пишутся.
static void set_state(imu_state_t st)
{
    taskENTER_CRITICAL(&s_lock);
    imu_state_t old = s_status.state;
    imu_status_t cur = s_status;
    s_status.state = st;
    taskEXIT_CRITICAL(&s_lock);
    if (old == st) {
        return;
    }
    switch (st) {
    case IMU_STATE_NOT_FOUND:
        events_add('E', "IMU_NOT_FOUND", "no driver found a chip");
        break;
    case IMU_STATE_WRONG_CHIP:
        events_add('E', "IMU_WRONG_CHIP", "0x%02X", cur.chip_id);
        break;
    case IMU_STATE_BUS_ERROR:
        events_add('E', "IMU_BUS_ERROR", "%s", cur.name ? cur.name : "");
        break;
    case IMU_STATE_NO_DATA:
        events_add('E', "IMU_NO_DATA", "%s", cur.name ? cur.name : "");
        break;
    case IMU_STATE_OK:
        if (old == IMU_STATE_BUS_ERROR || old == IMU_STATE_NO_DATA) {
            events_add('I', "IMU_OK", "%s", cur.name);
        } else {
            events_add('I', "IMU_FOUND", "%s 0x%02X %d Hz", cur.name, cur.addr, cur.odr_hz);
        }
        break;
    case IMU_STATE_INIT:
        break;
    }
}

static bool addr_in(const uint8_t *list, uint8_t addr)
{
    for (; *list; list++) {
        if (*list == addr) {
            return true;
        }
    }
    return false;
}

// Опрос шины и WHO_AM_I. При успехе s_drv выбран и устройство подключено к шине.
static imu_state_t detect(void)
{
    bool acked[128] = {0};
    int n_acked = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (s_cfg.bus.i2c_addr && a != s_cfg.bus.i2c_addr) {
            continue;
        }
        // Без сброса перед каждым адресом драйвер I²C отдаёт ему ответ предыдущего: в списке
        // появлялись адреса, на которых никого нет, каждый раз разные.
        imu_bus_reset();
        if (imu_bus_probe(a)) {
            acked[a] = true;
            n_acked++;
        }
    }
    // Список отвечающих адресов печатается, только когда он изменился.
    static char prev[128] = "-";
    {
        char line[128] = "";
        for (int a = 0; a < 128; a++) {
            if (acked[a]) {
                size_t l = strlen(line);
                snprintf(line + l, sizeof(line) - l, " 0x%02X", a);
            }
        }
        if (strcmp(line, prev) != 0) {
            ESP_LOGI(TAG, "на шине отвечают:%s", n_acked ? line : " никто");
            strlcpy(prev, line, sizeof(prev));
        }
    }

    // Список выше — для лога. WHO_AM_I читается на каждом адресе драйверов, что бы перебор ни показал:
    // пропущенный в нём датчик иначе нашёлся бы только через 2 с.
    imu_bus_reset();
    int wrong_id = -1;
    for (int a = 0x08; a <= 0x77; a++) {
        if (s_cfg.bus.i2c_addr && a != s_cfg.bus.i2c_addr) {
            continue;
        }
        // Каждый регистр WHO_AM_I читается на адресе один раз: читать чужие регистры у спящего
        // датчика опасно (ICM-20602, 10.8).
        int read_reg = -1;
        uint8_t id = 0;
        for (int d = 0; imu_drivers[d]; d++) {
            const imu_driver_t *drv = imu_drivers[d];
            if (!addr_in(drv->i2c_addrs, a)) {
                continue;
            }
            if (read_reg != drv->id_reg) {
                if (imu_bus_attach(a) != ESP_OK || imu_bus_read(drv->id_reg, &id, 1) != ESP_OK) {
                    break;
                }
                read_reg = drv->id_reg;
            }
            if (addr_in(drv->id_values, id)) {
                s_drv = drv;
                taskENTER_CRITICAL(&s_lock);
                s_status.name = drv->name;
                s_status.addr = a;
                s_status.chip_id = id;
                taskEXIT_CRITICAL(&s_lock);
                return IMU_STATE_OK;
            }
            wrong_id = id;
            ESP_LOGD(TAG, "0x%02X: рег 0x%02X = 0x%02X, не %s", a, drv->id_reg, id, drv->name);
        }
        imu_bus_detach();
    }

    taskENTER_CRITICAL(&s_lock);
    s_status.name = NULL;
    s_status.addr = 0;
    s_status.chip_id = wrong_id;
    taskEXIT_CRITICAL(&s_lock);
    return wrong_id >= 0 ? IMU_STATE_WRONG_CHIP : IMU_STATE_NOT_FOUND;
}

static bool start_driver(void)
{
    int odr = 0;
    esp_err_t err = s_drv->init(&s_cfg, &odr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: init: %s", s_drv->name, esp_err_to_name(err));
        return false;
    }
    taskENTER_CRITICAL(&s_lock);
    s_status.odr_hz = odr;
    taskEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "%s: адрес 0x%02X, WHO_AM_I 0x%02X, %d Гц", s_drv->name, s_status.addr,
             s_status.chip_id, odr);
    return true;
}

static void count_error(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_status.errors++;
    taskEXIT_CRITICAL(&s_lock);
}

static void imu_task(void *arg)
{
    int64_t rate_t0 = 0;
    int rate_n = 0;
    bool slow = false;
    int start_tries = START_TRIES;
    // Чтения подряд без отсчётов: сколько их, когда началось и была ли среди них ошибка шины.
    int bad_n = 0;
    int64_t bad_since_us = 0;
    bool bad_bus = false;

    for (;;) {
        s_alive_ms = now_ms();
        if (s_suspended || s_paused) {
            if (s_drv) {
                s_drv->deinit();
                s_drv = NULL;
                imu_bus_detach();
                set_state(IMU_STATE_INIT);
            }
            start_tries = START_TRIES;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        // Поиск датчика: раз в 2 с, пока не найдётся.
        if (!s_drv) {
            imu_state_t found = detect();
            bool up = s_drv && start_driver();
            if (!up) {
                if (s_drv) {
                    count_error();
                    found = IMU_STATE_BUS_ERROR;
                    s_drv = NULL;
                    imu_bus_detach();
                }
                if (start_tries > 0) {
                    start_tries--;
                    vTaskDelay(pdMS_TO_TICKS(START_RETRY_MS));
                    continue;
                }
                set_state(found);
                vTaskDelay(pdMS_TO_TICKS(found == IMU_STATE_BUS_ERROR ? REINIT_MS : REDETECT_MS));
                continue;
            }
            start_tries = 0;
            // Поиск и запуск датчика могли занять больше STALL_MS: метка должна быть свежей раньше,
            // чем состояние станет OK.
            s_alive_ms = now_ms();
            set_state(IMU_STATE_OK);
            rate_t0 = esp_timer_get_time();
            rate_n = 0;
            bad_n = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
        int n = s_drv->read(s_batch, BATCH_MAX);
        int64_t now = esp_timer_get_time();

        // IMU-11. Одно сорванное чтение — ещё не отказ: запись во флеш (настройки, калибровка,
        // подтверждение прошивки) останавливает оба ядра на десятки миллисекунд.
        imu_state_t fail = IMU_STATE_OK;
        if (n > 0) {
            bad_n = 0;
        } else {
            if (bad_n++ == 0) {
                bad_since_us = now;
                bad_bus = false;
            }
            if (n < 0) {
                count_error();
                bad_bus = true;
            }
            if (bad_n >= FAIL_MIN_READS && now - bad_since_us > FAIL_HOLD_US) {
                fail = bad_bus ? IMU_STATE_BUS_ERROR : IMU_STATE_NO_DATA;
            }
        }
        if (fail != IMU_STATE_OK) {
            // Раз в секунду deinit + init. Если не поднялся — заново автоопределение.
            set_state(fail);
            s_drv->deinit();
            vTaskDelay(pdMS_TO_TICKS(REINIT_MS));
            bad_n = 0;
            if (start_driver()) {
                s_alive_ms = now_ms();
                set_state(IMU_STATE_OK);
                rate_t0 = esp_timer_get_time();
                rate_n = 0;
            } else {
                s_drv = NULL;
                imu_bus_detach();
            }
            continue;
        }

        if (n > 0) {
            rate_n += n;
            if (s_cb) {
                s_cb(s_batch, n, s_cb_ctx);
            }
        }

        if (now - rate_t0 >= 1000000) {
            float hz = rate_n * 1e6f / (float)(now - rate_t0);
            taskENTER_CRITICAL(&s_lock);
            s_status.odr_measured_hz = hz;
            taskEXIT_CRITICAL(&s_lock);
            // IMU-12: событие при переходе ниже порога, работа продолжается.
            if (hz < SLOW_ODR_HZ && !slow) {
                events_add('W', "IMU_SLOW_ODR", "%.0f Hz", hz);
            }
            slow = hz < SLOW_ODR_HZ;
            rate_t0 = now;
            rate_n = 0;
        }
    }
}

void imu_set_suspended(bool suspended)
{
    s_suspended = suspended;
}

void imu_set_paused(bool paused)
{
    s_paused = paused;
}

esp_err_t imu_start(const imu_config_t *cfg, imu_sample_cb_t cb, void *ctx)
{
    s_cfg = *cfg;
    s_cb = cb;
    s_cb_ctx = ctx;
    esp_err_t err = imu_bus_open(&s_cfg.bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "шина: %s", esp_err_to_name(err));
        return err;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(imu_task, "imu_task", 6144, NULL,
                                            configMAX_PRIORITIES - 2, NULL, IMU_CORE);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
