#include "net.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "config.h"
#include "events.h"
#include "imu.h"
#include "motion.h"
#include "obd.h"
#include "power.h"
#include "proto.h"

#define TAG "net"

#define HELLO_TIMEOUT_US  3000000   // 9.2 п. 3
#define NGS_PERIOD_US     1000000
#define IMU_TEST_US       10000000
#define NGT_PERIOD_US     100000
#define MAX_WAIT_US       50000
#define MAX_CLIENTS       2         // NET-4

// Биты NGD, которые не знает motion (9.3).
#define FLAG_IMU_OK     0x001
#define FLAG_OBD_ABSENT 0x200
#define FLAG_ERROR      0x400

static const char *s_fw;
static int s_sock = -1;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static net_state_t s_state;

// Состояние net_task.
static struct sockaddr_in s_peer;       // кому идут данные: последний HELLO
static bool    s_have_peer;
static int64_t s_last_hello;
static bool    s_phone;
static uint32_t s_seq;
static uint32_t s_sent_ev;
static struct sockaddr_in s_cal_addr;   // кому отвечать на калибровку
static struct sockaddr_in s_test_addr;
static int64_t s_test_until;
static int64_t s_next_ngt;
static bool    s_ngs_now;
static char    s_out[PROTO_LINE_MAX];
static bool    s_wifi_on;

void net_get_state(net_state_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
}

static void set_wifi(bool up, bool error)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.wifi_up = up;
    s_state.wifi_error = error;
    taskEXIT_CRITICAL(&s_lock);
}

static void send_to(const struct sockaddr_in *to, int len)
{
    if (len > 0) {
        sendto(s_sock, s_out, len, 0, (const struct sockaddr *)to, sizeof(*to));
    }
}

static void send_reply(const struct sockaddr_in *to, int seq, const char *what)
{
    send_to(to, proto_line(s_out, sizeof(s_out), "NGA,%d,%s", seq, what));
}

// ---- строки блока ----

static uint32_t full_flags(uint32_t motion_flags)
{
    imu_status_t imu;
    obd_status_t obd;
    imu_get_status(&imu);
    obd_get_status(&obd);
    uint32_t f = motion_flags;
    if (imu.state == IMU_STATE_OK) {
        f |= FLAG_IMU_OK;
    }
    if (obd.state == OBD_STATE_DISABLED || obd.state == OBD_STATE_NO_ADAPTER ||
        obd.state == OBD_STATE_SLEEP) {
        f |= FLAG_OBD_ABSENT;
    }
    bool obd_err = obd.state != OBD_STATE_DISABLED && obd.state != OBD_STATE_OK &&
                   obd.state != OBD_STATE_SLEEP;
    if (imu.state != IMU_STATE_OK || obd_err || (motion_flags & MOTION_MOUNT_MOVED)) {
        f |= FLAG_ERROR;
    }
    return f;
}

static void send_ngd(void)
{
    motion_packet_t p;
    motion_take_packet(&p);
    int len = proto_line(s_out, sizeof(s_out), "NGD,%d,%lu,%lu,%lld,%ld,%llu,%d,%d,%lX",
                         PROTO_VERSION, (unsigned long)s_seq++, (unsigned long)p.t_ms,
                         (long long)p.yaw_mdeg, (long)p.rate_mdps, (unsigned long long)p.dist_mm,
                         p.speed_kmh, p.speed_age_ms, (unsigned long)full_flags(p.flags));
    send_to(&s_peer, len);
}

static void send_ngs(const struct sockaddr_in *to)
{
    imu_status_t imu;
    obd_status_t obd;
    motion_status_t mo;
    imu_get_status(&imu);
    obd_get_status(&obd);
    motion_get_status(&mo);

    char protocol[sizeof(obd.protocol)];
    strlcpy(protocol, obd.protocol, sizeof(protocol));
    proto_clean_text(protocol);
    char bias_age[12] = "", temp[12] = "", chip[12] = "", rpm[12] = "", volt[12] = "", elm[12] = "", ecu[12] = "";
    if (obd.rpm >= 0) {
        snprintf(rpm, sizeof(rpm), "%d", obd.rpm);
    }
    power_status_t pw;
    power_get_status(&pw);
    // 19 — свой АЦП, 20 — ELM327 (ATRV), 21 — блок управления машины (PID 42). 20 и 21 на ходу пустые.
    if (pw.adc_mv >= 0) {
        snprintf(volt, sizeof(volt), "%d", pw.adc_mv);
    }
    if (obd.voltage_mv >= 0) {
        snprintf(elm, sizeof(elm), "%d", obd.voltage_mv);
    }
    if (obd.ecu_mv >= 0) {
        snprintf(ecu, sizeof(ecu), "%d", obd.ecu_mv);
    }
    const char *engine = pw.engine == OBD_ENGINE_RUN ? "RUN" : pw.engine == OBD_ENGINE_OFF ? "OFF" : "";
    if (mo.bias_age_s >= 0) {
        snprintf(bias_age, sizeof(bias_age), "%d", mo.bias_age_s);
    }
    if (mo.has_temp && imu.state == IMU_STATE_OK) {
        snprintf(temp, sizeof(temp), "%d", (int)(mo.temp_c * 10));
    }
    if (imu.chip_id >= 0) {
        snprintf(chip, sizeof(chip), "0x%02X", imu.chip_id);
    }
    int len = proto_line(s_out, sizeof(s_out), "NGS,%d,%s,%s,%d,%s,%s,%s,%s,%lu,%s,%s,%s,%d,%d,%lu,%s,%s,%s,%s,%s",
                         PROTO_VERSION, s_fw, imu.name && imu.state == IMU_STATE_OK ? imu.name : "",
                         imu.state == IMU_STATE_OK ? imu.odr_hz : 0, obd_state_name(obd.state),
                         protocol, bias_age, temp, (unsigned long)imu.errors,
                         imu_state_name(imu.state), chip, obd_error_name(obd.error), obd.baud,
                         mo.bias_up_mdps, (unsigned long)events_last(), engine, rpm, volt, elm, ecu);
    send_to(to, len);
}

static void send_nge(const struct sockaddr_in *to, const event_t *ev)
{
    send_to(to, proto_line(s_out, sizeof(s_out), "NGE,%d,%lu,%lu,%c,%s,%s", PROTO_VERSION,
                           (unsigned long)ev->num, (unsigned long)ev->t_ms, ev->level, ev->code,
                           ev->text));
}

static void send_ngt(void)
{
    imu_sample_t x;
    if (!motion_last_sample(&x)) {
        return;
    }
    const float r2md = 180000.0f / 3.14159265f;
    send_to(&s_test_addr,
            proto_line(s_out, sizeof(s_out), "NGT,%d,%lu,%ld,%ld,%ld,%ld,%ld,%ld", PROTO_VERSION,
                       (unsigned long)(x.t_us / 1000), lroundf(x.gyro[0] * r2md),
                       lroundf(x.gyro[1] * r2md), lroundf(x.gyro[2] * r2md),
                       lroundf(x.accel[0] * 1000), lroundf(x.accel[1] * 1000),
                       lroundf(x.accel[2] * 1000)));
}

// ---- команды NGC (9.5) ----

static void handle_cmd(char **f, int n, const struct sockaddr_in *from, int64_t now)
{
    if (n < 3 || strcmp(f[0], "NGC") != 0) {
        return;
    }
    char *end;
    long seq = strtol(f[1], &end, 10);
    if (*end) {
        return;
    }
    const char *cmd = f[2];

    if (strcmp(cmd, "HELLO") == 0) {
        bool other = !s_have_peer || s_peer.sin_addr.s_addr != from->sin_addr.s_addr ||
                     s_peer.sin_port != from->sin_port;
        s_peer = *from;
        s_have_peer = true;
        s_last_hello = now;
        if (!s_phone) {
            s_phone = true;
            s_sent_ev = events_last();
            events_add('I', "PHONE_CONNECTED", "%s:%u", inet_ntoa(from->sin_addr), ntohs(from->sin_port));
            s_ngs_now = true;
        } else if (other) {
            ESP_LOGI(TAG, "данные теперь идут на %s:%u", inet_ntoa(from->sin_addr), ntohs(from->sin_port));
            s_ngs_now = true;
        }
        return;
    }
    if (strcmp(cmd, "STATUS") == 0) {
        send_ngs(from);
        return;
    }

    struct { const char *name; motion_cmd_t cmd; } cal[] = {
        {"CAL_BIAS", MOTION_CMD_CAL_BIAS},
        {"CAL_UP", MOTION_CMD_CAL_UP},
        {"CAL_UP2", MOTION_CMD_CAL_UP2},
        {"CAL_RESET", MOTION_CMD_CAL_RESET},
    };
    for (size_t i = 0; i < sizeof(cal) / sizeof(cal[0]); i++) {
        if (strcmp(cmd, cal[i].name) != 0) {
            continue;
        }
        imu_status_t imu;
        imu_get_status(&imu);
        if (cal[i].cmd != MOTION_CMD_CAL_RESET && imu.state != IMU_STATE_OK) {
            send_reply(from, seq, "ERR,IMU");
            return;
        }
        const char *err = motion_command(cal[i].cmd, seq);
        if (err) {
            char r[24];
            snprintf(r, sizeof(r), "ERR,%s", err);
            send_reply(from, seq, r);
        } else {
            s_cal_addr = *from;
        }
        return;
    }

    if (strcmp(cmd, "CAL_FWD") == 0) {
        // 7.4: «вперёд» учится только по скорости из OBD — второй этап.
        motion_status_t mo;
        motion_get_status(&mo);
        send_reply(from, seq, (mo.flags & MOTION_UP_OK) ? "ERR,NO_OBD" : "ERR,NO_UP");
    } else if (strcmp(cmd, "EVENTS") == 0) {
        uint32_t first = n > 3 ? strtoul(f[3], NULL, 10) : 1;
        uint32_t last = events_last();
        for (uint32_t i = first; i <= last; i++) {
            event_t ev;
            if (events_get(i, &ev)) {
                send_nge(from, &ev);
            }
        }
        send_reply(from, seq, "OK");
    } else if (strcmp(cmd, "IMU_TEST") == 0) {
        imu_status_t imu;
        imu_get_status(&imu);
        if (imu.state != IMU_STATE_OK) {
            send_reply(from, seq, "ERR,IMU");
            return;
        }
        s_test_addr = *from;
        s_test_until = now + IMU_TEST_US;
        s_next_ngt = now;
        send_reply(from, seq, "OK");
    } else if (strcmp(cmd, "SET") == 0) {
        if (n < 5) {
            send_reply(from, seq, n < 4 ? "ERR,KEY" : "ERR,VALUE");
            return;
        }
        esp_err_t err = config_set(f[3], f[4]);
        send_reply(from, seq, err == ESP_OK ? "OK" : err == ESP_ERR_NOT_FOUND ? "ERR,KEY"
                                                   : err == ESP_ERR_INVALID_ARG ? "ERR,VALUE" : "ERR,NVS");
    } else if (strcmp(cmd, "REBOOT") == 0) {
        send_reply(from, seq, "OK");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        send_reply(from, seq, "ERR,UNKNOWN");
    }
}

static void receive(int64_t now)
{
    char buf[512];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    int len = recvfrom(s_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (struct sockaddr *)&from, &fl);
    if (len <= 0) {
        return;
    }
    buf[len] = '\0';
    // Обычно одна строка на датаграмму (PR-1), но разрезать по \n не мешает.
    char *save = NULL;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        char *f[8];
        int n;
        if (proto_parse(line, f, 8, &n)) {
            handle_cmd(f, n, &from, now);
        }
    }
}

// ---- задача ----

typedef struct {
    imu_state_t imu;
    int         chip;
    obd_state_t obd;
    obd_error_t obd_err;
    uint32_t    events;
} ngs_sig_t;

static void ngs_sig(ngs_sig_t *s)
{
    imu_status_t imu;
    obd_status_t obd;
    imu_get_status(&imu);
    obd_get_status(&obd);
    memset(s, 0, sizeof(*s));
    s->imu = imu.state;
    s->chip = imu.chip_id;
    s->obd = obd.state;
    s->obd_err = obd.error;
    s->events = events_last();
}

static void net_task(void *arg)
{
    int64_t next_ngd = 0, next_ngs = 0;
    ngs_sig_t sig_sent = {0};

    for (;;) {
        int64_t now = esp_timer_get_time();
        int64_t due = now + MAX_WAIT_US;
        if (s_phone) {
            due = next_ngd < due ? next_ngd : due;
            due = next_ngs < due ? next_ngs : due;
        }
        if (s_test_until > now && s_next_ngt < due) {
            due = s_next_ngt;
        }
        int64_t wait = due - now;
        if (wait > 0) {
            fd_set rd;
            FD_ZERO(&rd);
            FD_SET(s_sock, &rd);
            struct timeval tv = {.tv_sec = 0, .tv_usec = (suseconds_t)wait};
            if (select(s_sock + 1, &rd, NULL, NULL, &tv) > 0) {
                receive(esp_timer_get_time());
            }
        }
        now = esp_timer_get_time();

        // Ответы на калибровку.
        motion_tick();
        motion_reply_t r;
        while (motion_get_reply(&r)) {
            char what[32];
            if (r.progress >= 0) {
                snprintf(what, sizeof(what), "PROGRESS,%d", r.progress);
            } else if (r.ok) {
                strlcpy(what, "OK", sizeof(what));
            } else {
                snprintf(what, sizeof(what), "ERR,%s", r.err);
            }
            send_reply(&s_cal_addr, r.seq, what);
        }

        // IMU_TEST: строки NGT раз в 0.1 с.
        if (s_test_until > now && now >= s_next_ngt) {
            send_ngt();
            s_next_ngt += NGT_PERIOD_US;
        }

        // Сессия: нет HELLO 3 с — данные больше не шлём.
        if (s_phone && now - s_last_hello > HELLO_TIMEOUT_US) {
            s_phone = false;
            events_add('I', "PHONE_LOST", "%s", inet_ntoa(s_peer.sin_addr));
        }
        taskENTER_CRITICAL(&s_lock);
        s_state.phone = s_phone;
        taskEXIT_CRITICAL(&s_lock);
        if (!s_phone) {
            next_ngd = next_ngs = now;
            continue;
        }

        if (now >= next_ngd) {
            send_ngd();
            int64_t period = 1000000 / config_get_int("data_hz");
            next_ngd += period;
            if (next_ngd < now) {
                next_ngd = now + period;    // отстали — не догонять пачкой
            }
        }

        ngs_sig_t sig;
        ngs_sig(&sig);
        if (s_ngs_now || now >= next_ngs || memcmp(&sig, &sig_sent, sizeof(sig)) != 0) {
            send_ngs(&s_peer);
            sig_sent = sig;
            s_ngs_now = false;
            next_ngs = now + NGS_PERIOD_US;
        }

        // Новые события — сразу.
        uint32_t last = events_last();
        while (s_sent_ev < last) {
            event_t ev;
            if (events_get(++s_sent_ev, &ev)) {
                send_nge(&s_peer, &ev);
            }
        }
    }
}

// ---- Wi-Fi ----

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case WIFI_EVENT_AP_START:
        // NET-5: без энергосбережения, иначе пакеты задерживаются на сотни миллисекунд.
        esp_wifi_set_ps(WIFI_PS_NONE);
        set_wifi(true, false);
        ESP_LOGI(TAG, "точка доступа поднята");
        break;
    case WIFI_EVENT_AP_STOP:
        set_wifi(false, false);
        break;
    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *e = data;
        ESP_LOGI(TAG, "подключился " MACSTR, MAC2STR(e->mac));
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *e = data;
        ESP_LOGI(TAG, "отключился " MACSTR, MAC2STR(e->mac));
        break;
    }
    default:
        break;
    }
}

static esp_err_t wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    // NET-2: адрес точки доступа по умолчанию 192.168.4.1/24, DHCP-сервер включён.
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));

    if (strcmp(config_get_str("wifi_mode"), "ap") != 0) {
        ESP_LOGW(TAG, "wifi_mode=%s пока не поддерживается (второй этап), поднимаю точку доступа",
                 config_get_str("wifi_mode"));
    }

    wifi_config_t wc = {
        .ap = {
            .channel = 1,
            .max_connection = MAX_CLIENTS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.required = false},
        },
    };
    // NET-1: NoGPS-XXXX, XXXX — последние два байта MAC.
    const char *ssid = config_get_str("wifi_ssid");
    if (ssid[0]) {
        strlcpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid));
    } else {
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
        snprintf((char *)wc.ap.ssid, sizeof(wc.ap.ssid), "NoGPS-%02X%02X", mac[4], mac[5]);
    }
    wc.ap.ssid_len = strlen((char *)wc.ap.ssid);
    strlcpy((char *)wc.ap.password, config_get_str("wifi_pass"), sizeof(wc.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    err = esp_wifi_start();
    if (err != ESP_OK) {
        set_wifi(false, true);
        return err;
    }
    s_wifi_on = true;
    ESP_LOGI(TAG, "сеть %s, адрес 192.168.4.1, UDP %d", wc.ap.ssid, NET_PORT);
    return ESP_OK;
}

void net_set_wifi(bool on)
{
    if (on == s_wifi_on || s_sock < 0) {
        return;
    }
    esp_err_t err = on ? esp_wifi_start() : esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi %s: %s", on ? "start" : "stop", esp_err_to_name(err));
        set_wifi(false, true);
        return;
    }
    s_wifi_on = on;
}

esp_err_t net_start(const char *fw_version)
{
    s_fw = fw_version;
    esp_err_t err = wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi: %s", esp_err_to_name(err));
        return err;
    }
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) {
        return ESP_FAIL;
    }
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(NET_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind %d", NET_PORT);
        return ESP_FAIL;
    }
    return xTaskCreatePinnedToCore(net_task, "net_task", 6144, NULL, 10, NULL, 0) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}
