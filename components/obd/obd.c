// ELM327 / STN11xx по UART (ТЗ, раздел 4): поиск скорости порта, инициализация, опрос PID 0D,
// ошибки по таблице 4.4 и переподключение без перезагрузки (NO-4).
#include "obd.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "config.h"
#include "events.h"

#define TAG "obd"

#define PORT            CONFIG_NOGPS_OBD_UART_PORT
#define RX_BUF          1024
#define REPLY_MAX       160
#define NVS_KEY_BAUD    "obd_last_baud"

#define ATI_MS          1000        // OBD-2
#define ATZ_MS          2000        // 4.2
#define AT_MS           1000
#define SEARCH_MS       15000       // 0100
#define POLL_MS         500         // нет '>' дольше 500 мс — OBD_TIMEOUT
#define POLL_PERIOD_US  100000      // OBD-5: 10 запросов в секунду
#define NO_ADAPTER_RETRY_MS 10000
#define NO_CAR_RETRY_MS 5000
#define FAILS_MAX       5           // 4.4: после 5 ошибок подряд
#define NO_DATA_RECHECK 20          // 2 с NO DATA — проверить, жива ли машина (0100)
#define BAD_REPLY_EVENT_US 10000000
#define RPM_EVERY       10          // обороты (010C) — раз в секунду
#define VOLT_EVERY      50          // напряжение (ATRV) — раз в 5 с

static const int s_bauds[] = {38400, 9600, 115200, 57600, 230400, 500000};

static obd_speed_cb_t s_cb;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static obd_status_t s_st = {.state = OBD_STATE_DISABLED, .rpm = -1, .voltage_mv = -1};

// Состояние obd_task.
static int         s_proto;           // протокол ELM327, 0 — авто
static obd_error_t s_reported;        // последняя ошибка, о которой было событие
static int64_t     s_bad_event_us;
static int         s_kmh = -1;        // последняя скорость
static volatile bool s_paused;
static int         s_ecu_mv = -1;     // напряжение от блока управления машины
static uint32_t    s_volt_seq;

// ---- состояние и события ----

static void set_state(obd_state_t st, obd_error_t err)
{
    taskENTER_CRITICAL(&s_lock);
    s_st.state = st;
    s_st.error = err;
    taskEXIT_CRITICAL(&s_lock);
}

static void set_info(const char *protocol, int baud)
{
    taskENTER_CRITICAL(&s_lock);
    if (protocol) {
        strlcpy(s_st.protocol, protocol, sizeof(s_st.protocol));
    }
    if (baud >= 0) {
        s_st.baud = baud;
    }
    taskEXIT_CRITICAL(&s_lock);
}

// Ошибка: состояние для NGS всегда, событие — только при смене ошибки, чтобы не забить журнал из 32
// событий одной и той же ошибкой 10 раз в секунду.
static void fail(obd_state_t st, obd_error_t err, char level, const char *code, const char *text)
{
    set_state(st, err);
    if (s_reported != err) {
        s_reported = err;
        events_add(level, code, "%s", text);
    }
}

// Двигатель по данным машины: едет — значит работает; стоит — по оборотам. Иначе UNKNOWN.
static void set_engine(int kmh, int rpm, int mv)
{
    obd_engine_t e = OBD_ENGINE_UNKNOWN;
    if (kmh > 0) {
        e = OBD_ENGINE_RUN;
    } else if (rpm >= 0) {
        e = rpm > 0 ? OBD_ENGINE_RUN : OBD_ENGINE_OFF;
    }
    taskENTER_CRITICAL(&s_lock);
    s_st.engine = e;
    s_st.rpm = rpm;
    s_st.voltage_mv = mv;
    s_st.ecu_mv = s_ecu_mv;
    s_st.volt_seq = s_volt_seq;
    taskEXIT_CRITICAL(&s_lock);
}

// ---- UART ----

// Отправить команду и читать ответ до приглашения '>'. Переводы строк заменяются пробелами.
// true — '>' пришёл; t_sent и t_done — моменты отправки и прихода '>'.
static bool cmd(const char *c, int timeout_ms, char *out, int64_t *t_sent, int64_t *t_done)
{
    uart_flush_input(PORT);
    int64_t t0 = esp_timer_get_time();
    uart_write_bytes(PORT, c, strlen(c));
    uart_write_bytes(PORT, "\r", 1);

    size_t len = 0;
    bool prompt = false;
    int64_t deadline = t0 + (int64_t)timeout_ms * 1000;
    while (!prompt) {
        int64_t left_ms = (deadline - esp_timer_get_time()) / 1000;
        if (left_ms <= 0) {
            break;
        }
        uint8_t b[64];
        int n = uart_read_bytes(PORT, b, sizeof(b), pdMS_TO_TICKS(left_ms < 20 ? left_ms : 20));
        for (int i = 0; i < n && !prompt; i++) {
            char ch = (char)b[i];
            if (ch == '>') {
                prompt = true;
            } else if (ch == '\r' || ch == '\n' || ch == ' ') {
                if (len > 0 && out[len - 1] != ' ' && len < REPLY_MAX - 1) {
                    out[len++] = ' ';
                }
            } else if (isprint((unsigned char)ch) && len < REPLY_MAX - 1) {
                out[len++] = ch;
            }
        }
    }
    while (len > 0 && out[len - 1] == ' ') {
        len--;
    }
    out[len] = '\0';
    if (t_sent) {
        *t_sent = t0;
    }
    if (t_done) {
        *t_done = esp_timer_get_time();
    }
    return prompt;
}

// Ответ без пробелов, для поиска «410D2A» при ответе с пробелами «41 0D 2A».
static void compact(const char *in, char *out)
{
    for (; *in; in++) {
        if (*in != ' ') {
            *out++ = *in;
        }
    }
    *out = '\0';
}

static bool has(const char *s, const char *what)
{
    return strstr(s, what) != NULL;
}

static bool bus_error(const char *r)
{
    return has(r, "CAN ERROR") || has(r, "BUS ERROR") || has(r, "BUS INIT") || has(r, "STOPPED") ||
           has(r, "ERR") || has(r, "BUS BUSY") || has(r, "FB ERROR") || has(r, "DATA ERROR");
}

// ---- поиск порта (OBD-2) ----

static int load_baud(void)
{
    int32_t v = 0;
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i32(h, NVS_KEY_BAUD, &v);
        nvs_close(h);
    }
    return v;
}

static void save_baud(int baud)
{
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, NVS_KEY_BAUD, baud);
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool probe(int baud, char *ati)
{
    uart_set_baudrate(PORT, baud);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Вторая попытка: первая команда могла склеиться с мусором от прошлой скорости и получить «?».
    for (int i = 0; i < 2; i++) {
        cmd("ATI", ATI_MS, ati, NULL, NULL);
        if (has(ati, "ELM327") || has(ati, "STN")) {
            return true;
        }
    }
    return false;
}

static int find_baud(char *ati)
{
    int fixed = config_get_int("obd_baud");
    if (fixed > 0) {
        return probe(fixed, ati) ? fixed : 0;
    }
    int last = load_baud();
    if (last > 0 && probe(last, ati)) {
        return last;
    }
    for (size_t i = 0; i < sizeof(s_bauds) / sizeof(s_bauds[0]); i++) {
        if (s_bauds[i] != last && probe(s_bauds[i], ati)) {
            save_baud(s_bauds[i]);
            return s_bauds[i];
        }
    }
    return 0;
}

// ---- инициализация (4.2) ----

// false — адаптер замолчал.
static bool init_adapter(void)
{
    static const char *const steps[] = {"ATE0", "ATL0", "ATS0", "ATH0", "ATAT2"};
    char r[REPLY_MAX];
    if (!cmd("ATZ", ATZ_MS, r, NULL, NULL)) {
        return false;
    }
    ESP_LOGI(TAG, "ATZ -> %s", r);
    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        if (!cmd(steps[i], AT_MS, r, NULL, NULL)) {
            return false;
        }
        ESP_LOGI(TAG, "%s -> %s", steps[i], r);
    }
    return true;
}

typedef enum { CONN_OK, CONN_NO_CAR, CONN_LOST } conn_t;

static conn_t try_0100(int proto, char *r)
{
    char c[8];
    snprintf(c, sizeof(c), "ATSP%X", proto);
    if (!cmd(c, AT_MS, r, NULL, NULL)) {
        return CONN_LOST;
    }
    ESP_LOGI(TAG, "%s -> %s", c, r);
    if (!cmd("0100", SEARCH_MS, r, NULL, NULL)) {
        return CONN_LOST;
    }
    ESP_LOGI(TAG, "0100 -> %s", r);
    char z[REPLY_MAX];
    compact(r, z);
    return has(z, "4100") ? CONN_OK : CONN_NO_CAR;
}

// Найти протокол машины: сначала сохранённый, если не ответил — автоматически.
static conn_t connect(char *r)
{
    set_state(OBD_STATE_SEARCHING, s_st.error);
    conn_t c = try_0100(s_proto, r);
    if (c == CONN_NO_CAR && s_proto != 0) {
        c = try_0100(0, r);
    }
    if (c != CONN_OK) {
        return c;
    }

    char p[REPLY_MAX];
    if (cmd("ATDPN", AT_MS, p, NULL, NULL)) {
        // «A6» — автоматически найден 6. Номер — последняя шестнадцатеричная цифра.
        size_t n = strlen(p);
        if (n && isxdigit((unsigned char)p[n - 1])) {
            int proto = (int)strtol(&p[n - 1], NULL, 16);
            if (proto != s_proto) {
                char v[4];
                snprintf(v, sizeof(v), "%d", proto);
                config_set("obd_proto", v);
                s_proto = proto;
            }
        }
    }
    if (cmd("ATDP", AT_MS, p, NULL, NULL)) {
        const char *name = strncmp(p, "AUTO, ", 6) == 0 ? p + 6 : p;
        set_info(name, -1);
        ESP_LOGI(TAG, "протокол %d: %s", s_proto, name);
    }
    return CONN_OK;
}

// ---- опрос скорости (4.3) ----

typedef enum { POLL_OK, POLL_NO_DATA, POLL_BUS, POLL_TIMEOUT, POLL_BAD } poll_t;

static poll_t poll_speed(char *r)
{
    int64_t t_sent, t_done;
    if (!cmd("010D1", POLL_MS, r, &t_sent, &t_done)) {
        return POLL_TIMEOUT;
    }
    char z[REPLY_MAX];
    compact(r, z);
    // OBD-4: искать 410D внутри строки — перед ответом бывают SEARCHING... и BUS INIT: ...OK.
    const char *p = strstr(z, "410D");
    if (p && isxdigit((unsigned char)p[4]) && isxdigit((unsigned char)p[5])) {
        char hex[3] = {p[4], p[5], 0};
        int kmh = (int)strtol(hex, NULL, 16);
        s_kmh = kmh;
        if (s_cb) {
            s_cb(kmh, (t_sent + t_done) / 2);  // OBD-6: середина между запросом и '>'
        }
        return POLL_OK;
    }
    if (has(r, "NO DATA")) {
        return POLL_NO_DATA;
    }
    if (bus_error(r) || has(r, "UNABLE TO CONNECT")) {
        return POLL_BUS;
    }
    return POLL_BAD;
}

// Напряжение на выводе 16 разъёма, мВ: «12.6V». Машина для этого не нужна. -1 — нет ответа.
static int read_voltage(void)
{
    char r[REPLY_MAX];
    if (!cmd("ATRV", AT_MS, r, NULL, NULL)) {
        return -1;
    }
    char *end;
    float v = strtof(r, &end);
    return end != r && v > 5 && v < 20 ? (int)lroundf(v * 1000) : -1;
}

// Напряжение, которое измеряет блок управления машины, PID 42: «414231F4» — мВ.
// -1 — машина не отдаёт, -2 — адаптер не ответил.
static int read_ecu_voltage(void)
{
    char r[REPLY_MAX], z[REPLY_MAX];
    if (!cmd("01421", POLL_MS, r, NULL, NULL)) {
        return -2;
    }
    compact(r, z);
    const char *p = strstr(z, "4142");
    if (!p || strlen(p) < 8) {
        return -1;
    }
    char hex[5] = {p[4], p[5], p[6], p[7], 0};
    char *end;
    long v = strtol(hex, &end, 16);
    return *end || v < 5000 || v > 20000 ? -1 : (int)v;
}

// Обороты, PID 0C: «410C1AF8» — (A*256+B)/4. -1 — машина не отдаёт, -2 — адаптер не ответил.
static int read_rpm(void)
{
    char r[REPLY_MAX], z[REPLY_MAX];
    if (!cmd("010C1", POLL_MS, r, NULL, NULL)) {
        return -2;
    }
    compact(r, z);
    const char *p = strstr(z, "410C");
    if (!p || strlen(p) < 8) {
        return -1;
    }
    char hex[5] = {p[4], p[5], p[6], p[7], 0};
    char *end;
    long v = strtol(hex, &end, 16);
    return *end ? -1 : (int)(v / 4);
}

// ---- задача ----

typedef enum { STEP_FIND_PORT, STEP_INIT, STEP_CONNECT, STEP_POLL } step_t;

static void obd_task(void *arg)
{
    step_t step = STEP_FIND_PORT;
    char r[REPLY_MAX];
    int bus_fails = 0, timeouts = 0, no_data = 0;
    int64_t next_poll = 0;
    int rpm = -1, mv = -1;
    unsigned tick = 0;
    bool have_adapter = false, was_paused = false;
    int ecu_fails = 0;      // PID 42 есть не у всех машин: после трёх отказов больше не спрашиваем

    for (;;) {
        // Стоянка: ни одного байта в UART, пока паузу не снимут.
        if (s_paused) {
            if (!was_paused) {
                was_paused = true;
                s_reported = OBD_ERROR_NONE;
                set_state(OBD_STATE_SLEEP, OBD_ERROR_NONE);
                s_ecu_mv = -1;
                set_engine(s_kmh = -1, rpm = -1, mv = -1);
            }
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (was_paused) {
            was_paused = false;
            step = have_adapter ? STEP_CONNECT : STEP_FIND_PORT;
        }

        switch (step) {
        case STEP_FIND_PORT: {
            set_state(OBD_STATE_INIT, s_st.error);
            set_info("", 0);
            s_ecu_mv = -1;
            set_engine(s_kmh = -1, rpm = -1, mv = -1);
            have_adapter = false;
            char ati[REPLY_MAX];
            int baud = find_baud(ati);
            if (!baud) {
                fail(OBD_STATE_NO_ADAPTER, OBD_ERROR_NO_ADAPTER, 'E', "OBD_NO_ADAPTER",
                     "no reply to ATI");
                vTaskDelay(pdMS_TO_TICKS(NO_ADAPTER_RETRY_MS));
                break;
            }
            set_info(NULL, baud);
            have_adapter = true;
            events_add('I', "OBD_FOUND", "%d %s", baud, ati);
            step = STEP_INIT;
            break;
        }

        case STEP_INIT:
            set_state(OBD_STATE_INIT, s_st.error);
            if (!init_adapter()) {
                fail(OBD_STATE_ERROR, OBD_ERROR_TIMEOUT, 'E', "OBD_TIMEOUT", "init");
                step = STEP_FIND_PORT;
                break;
            }
            step = STEP_CONNECT;
            break;

        case STEP_CONNECT:
            switch (connect(r)) {
            case CONN_OK:
                step = STEP_POLL;
                bus_fails = timeouts = no_data = 0;
                next_poll = esp_timer_get_time();
                tick = 0;
                ecu_fails = 0;
                break;
            case CONN_NO_CAR:
                // Адаптер отвечает, машина — нет: зажигание или протокол. Повторять 0100 раз в 5 с.
                // Двигатель в это время виден только по напряжению.
                fail(OBD_STATE_NO_CAR, OBD_ERROR_NO_CAR, 'W', "OBD_NO_CAR", r[0] ? r : "no reply");
                s_ecu_mv = -1;
                mv = read_voltage();
                s_volt_seq++;
                set_engine(s_kmh = -1, rpm = -1, mv);
                for (int i = 0; i < NO_CAR_RETRY_MS / 100 && !s_paused; i++) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                break;
            case CONN_LOST:
                fail(OBD_STATE_ERROR, OBD_ERROR_TIMEOUT, 'E', "OBD_TIMEOUT", "0100");
                step = STEP_FIND_PORT;
                break;
            }
            break;

        case STEP_POLL: {
            int64_t wait = next_poll - esp_timer_get_time();
            if (wait > 0) {
                vTaskDelay(pdMS_TO_TICKS((wait + 999) / 1000));
            }
            next_poll = esp_timer_get_time() + POLL_PERIOD_US;

            poll_t res = poll_speed(r);
            // На ходу адаптер отдаёт только скорость: любой лишний запрос задержал бы её. Обороты (раз в
            // секунду) и напряжения (раз в 5 с) спрашиваются, только когда машина стоит.
            if (res == POLL_OK || res == POLL_NO_DATA) {
                if (res == POLL_NO_DATA) {
                    s_kmh = -1;
                }
                if (s_kmh > 0) {
                    rpm = mv = s_ecu_mv = -1;
                    tick = 0;       // после остановки — спросить сразу
                } else {
                    if (tick % RPM_EVERY == 0) {
                        int v = read_rpm();
                        rpm = v >= 0 ? v : -1;
                    }
                    if (tick % VOLT_EVERY == 0) {
                        mv = read_voltage();
                        if (res == POLL_OK && ecu_fails < 3) {
                            int e = read_ecu_voltage();
                            s_ecu_mv = e >= 0 ? e : -1;
                            ecu_fails = e >= 0 ? 0 : ecu_fails + 1;
                        } else {
                            s_ecu_mv = -1;
                        }
                        s_volt_seq++;
                    }
                    tick++;
                }
            } else {
                s_kmh = -1;
                rpm = -1;
            }
            set_engine(s_kmh, rpm, mv);

            switch (res) {
            case POLL_OK:
                bus_fails = timeouts = no_data = 0;
                if (s_reported != OBD_ERROR_NONE || s_st.state != OBD_STATE_OK) {
                    char proto[sizeof(s_st.protocol)];
                    obd_status_t st;
                    obd_get_status(&st);
                    strlcpy(proto, st.protocol, sizeof(proto));
                    events_add('I', "OBD_OK", "%s", proto);
                    s_reported = OBD_ERROR_NONE;
                }
                set_state(OBD_STATE_OK, OBD_ERROR_NONE);
                break;
            case POLL_NO_DATA:
                // Протокол жив, машина не отдаёт PID 0D. Раз в 2 с проверять, не выключено ли зажигание.
                fail(OBD_STATE_NO_DATA, OBD_ERROR_NO_SPEED, 'W', "OBD_NO_SPEED", r);
                if (++no_data >= NO_DATA_RECHECK) {
                    step = STEP_CONNECT;
                }
                break;
            case POLL_BUS:
                fail(OBD_STATE_ERROR, OBD_ERROR_BUS_ERROR, 'E', "OBD_BUS_ERROR", r);
                if (++bus_fails >= FAILS_MAX) {
                    step = STEP_INIT;   // заново с ATZ
                }
                break;
            case POLL_TIMEOUT:
                fail(OBD_STATE_ERROR, OBD_ERROR_TIMEOUT, 'E', "OBD_TIMEOUT", "no prompt");
                if (++timeouts >= FAILS_MAX) {
                    step = STEP_FIND_PORT;  // заново с поиска порта
                }
                break;
            case POLL_BAD: {
                // Мусор: повторить запрос; событие не чаще раза в 10 с, состояние не меняется.
                int64_t now = esp_timer_get_time();
                taskENTER_CRITICAL(&s_lock);
                s_st.error = OBD_ERROR_BAD_REPLY;
                taskEXIT_CRITICAL(&s_lock);
                if (now - s_bad_event_us > BAD_REPLY_EVENT_US) {
                    s_bad_event_us = now;
                    events_add('W', "OBD_BAD_REPLY", "%s", r[0] ? r : "empty");
                }
                break;
            }
            }
            break;
        }
        }
    }
}

#if CONFIG_NOGPS_OBD_SIMULATE
// Имитация для стола (раздел 13): синус 0–60 км/ч с периодом 60 с, отрицательная половина — стоянка.
static void sim_task(void *arg)
{
    set_info("SIMULATED", 0);
    set_state(OBD_STATE_OK, OBD_ERROR_NONE);
    set_engine(0, 800, 14000);
    events_add('I', "OBD_OK", "SIMULATED");
    for (;;) {
        int64_t now = esp_timer_get_time();
        double s = 60.0 * sin(2 * M_PI * (now / 1e6) / 60.0);
        if (s_cb) {
            s_cb(s > 0 ? (int)lround(s) : 0, now);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif

esp_err_t obd_start(obd_speed_cb_t cb)
{
    s_cb = cb;
    s_proto = config_get_int("obd_proto");
#if CONFIG_NOGPS_OBD_SIMULATE
    ESP_LOGW(TAG, "имитация OBD: скорость по синусу, UART не используется");
    (void)obd_task;
    return xTaskCreate(sim_task, "obd_task", 3072, NULL, 8, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#else
    uart_config_t uc = {
        .baud_rate = s_bauds[0],
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,   // OBD-1: 8N1 без управления потоком
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(PORT, RX_BUF, 0, 0, NULL, 0);
    if (err == ESP_OK) {
        err = uart_param_config(PORT, &uc);
    }
    if (err == ESP_OK) {
        err = uart_set_pin(PORT, CONFIG_NOGPS_OBD_TX, CONFIG_NOGPS_OBD_RX, UART_PIN_NO_CHANGE,
                           UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART%d: %s", PORT, esp_err_to_name(err));
        return err;
    }
    // Без адаптера вход висит в воздухе и ловит помехи — подтянуть к питанию.
    gpio_set_pull_mode(CONFIG_NOGPS_OBD_RX, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "UART%d TX=GPIO%d RX=GPIO%d", PORT, CONFIG_NOGPS_OBD_TX, CONFIG_NOGPS_OBD_RX);
    set_state(OBD_STATE_INIT, OBD_ERROR_NONE);
    return xTaskCreate(obd_task, "obd_task", 5120, NULL, 8, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

void obd_get_status(obd_status_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_st;
    taskEXIT_CRITICAL(&s_lock);
}

void obd_set_paused(bool paused)
{
    s_paused = paused;
}

const char *obd_state_name(obd_state_t s)
{
    static const char *const names[] = {"DISABLED", "NO_ADAPTER", "INIT", "SEARCHING",
                                        "NO_CAR", "NO_DATA", "ERROR", "OK", "SLEEP"};
    return s <= OBD_STATE_SLEEP ? names[s] : "?";
}

const char *obd_error_name(obd_error_t e)
{
    static const char *const names[] = {"NONE", "NO_ADAPTER", "NO_CAR", "NO_SPEED",
                                        "BUS_ERROR", "TIMEOUT", "BAD_REPLY"};
    return e <= OBD_ERROR_BAD_REPLY ? names[e] : "?";
}
