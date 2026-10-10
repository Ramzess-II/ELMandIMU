#include "net.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_event.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "ble.h"
#include "config.h"
#include "events.h"
#include "imu.h"
#include "motion.h"
#include "obd.h"
#include "power.h"
#include "proto.h"
#include "update.h"

#define TAG "net"

#define HELLO_TIMEOUT_US  3000000   // 9.2 п. 3
#define NGS_PERIOD_US     1000000
#define IMU_TEST_US       10000000
#define NGT_PERIOD_US     100000
#define MAX_WAIT_US       50000
#define MAX_CLIENTS       2         // NET-4
#define BT_SLOW_US        10000000  // событие о потерянных уведомлениях — не чаще раза в 10 с
#define NGO_PERIOD_US     250000    // NGO во время обновления — раз в 0.25 с
#define NGO_RESEND_US     200000    // просьба вернуться и повторить — не чаще
#define OTA_MIN_MV        11800     // обновление не начинается при напряжении ниже 11.8 В
#define RESTART_AFTER_US  1000000   // перезагрузка после удачного обновления
#define BQ_LEN            12        // строк в очереди для Bluetooth

// Биты NGD, которые не знает motion (9.3).
#define FLAG_IMU_OK     0x001
#define FLAG_OBD_ABSENT 0x200
#define FLAG_ERROR      0x400

// Кому идут строки: телефон в сети блока (UDP) или телефон по Bluetooth. Протокол один и тот же.
typedef enum { LINK_NONE, LINK_UDP, LINK_BLE } link_t;

typedef struct {
    link_t link;
    struct sockaddr_in addr;    // для LINK_UDP
} peer_t;

static const char *s_fw;
static char s_id[5];            // XXXX из имени NoGPS-XXXX
static char s_name[16];
static int s_sock = -1;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static net_state_t s_state;

// Состояние net_task.
static peer_t  s_peer;                  // кому идут данные: последний HELLO
static int64_t s_last_hello;
static bool    s_phone;
static uint32_t s_seq;
static uint32_t s_sent_ev;
static peer_t  s_cal_addr;              // кому отвечать на калибровку
static peer_t  s_test_addr;
static int64_t s_test_until;
static int64_t s_next_ngt;
static bool    s_ngs_now;
static bool    s_ngi_now;
static char    s_out[PROTO_LINE_MAX];
static bool    s_radio_on;
static uint32_t s_bt_drops;

// Журнал по команде EVENTS уходит не сразу весь, а по мере того, как у получателя есть место.
static struct {
    bool     active;
    peer_t   to;
    int      seq;
    uint32_t next, last;
} s_replay;

// Обновление прошивки: что телефон уже знает о его ходе.
static peer_t   s_ota_peer;
static update_state_t s_ota_state;      // UPDATE_IDLE — сообщать нечего
static uint32_t s_ota_reported;
static bool     s_ota_resend;
static int64_t  s_ota_ngo_at;
static int      s_ota_end_seq = -1;     // OTA_END, на который ещё не отвечено
static int64_t  s_restart_at;

void net_get_state(net_state_t *out)
{
    ble_state_t b;
    ble_get_state(&b);
    taskENTER_CRITICAL(&s_lock);
    *out = s_state;
    taskEXIT_CRITICAL(&s_lock);
    out->bt_up = b.up;
    out->bt_adv = b.advertising;
    out->bt_conn = b.connected;
    out->bt_link = b.secure;
    out->bt_pairing = b.pairing;
    out->bt_bonds = b.bonds;
}

static void set_wifi(bool up, bool error)
{
    taskENTER_CRITICAL(&s_lock);
    s_state.wifi_up = up;
    s_state.wifi_error = error;
    taskEXIT_CRITICAL(&s_lock);
}

// ---- очередь строк для Bluetooth ----
// По Bluetooth строка уходит не мгновенно: у стека конечная очередь уведомлений. Пропадать может
// только NGD (и NGT): в них накопленные итоги, следующая строка всё возмещает. Остальные строки ждут
// здесь и уходят раньше NGD. Из NGS в очереди держится только самая свежая.

typedef struct {
    uint16_t len, sent;
    bool     ngs;
    char     data[PROTO_LINE_MAX];
} bq_item_t;

static bq_item_t s_bq[BQ_LEN];
static int s_bq_head, s_bq_count;

static bq_item_t *bq_at(int i)
{
    return &s_bq[(s_bq_head + i) % BQ_LEN];
}

static void bq_flush(void)
{
    while (s_bq_count) {
        bq_item_t *it = bq_at(0);
        int n = ble_send(it->data + it->sent, it->len - it->sent);
        if (n < 0) {
            s_bq_count = 0;     // связи нет — слать некому
            return;
        }
        it->sent += n;
        if (it->sent < it->len) {
            return;             // очередь стека заполнена, остальное позже
        }
        s_bq_head = (s_bq_head + 1) % BQ_LEN;
        s_bq_count--;
    }
}

static void bq_put(const char *line, int len, int sent)
{
    bool ngs = strncmp(line, "$NGS", 4) == 0;
    bq_item_t *it = NULL;
    int old_ngs = -1;
    for (int i = 0; i < s_bq_count; i++) {
        if (bq_at(i)->ngs && bq_at(i)->sent == 0) {
            old_ngs = i;
            break;
        }
    }
    if (ngs && old_ngs >= 0) {
        it = bq_at(old_ngs);    // устаревшую NGS заменяет новая
    } else if (s_bq_count == BQ_LEN) {
        // Места нет: выбросить NGS, которая ещё не начала уходить, — следующая её заменит.
        if (old_ngs < 0) {
            s_bt_drops++;
            return;
        }
        for (int k = old_ngs; k < s_bq_count - 1; k++) {
            *bq_at(k) = *bq_at(k + 1);
        }
        s_bq_count--;
    }
    if (!it) {
        it = bq_at(s_bq_count++);
    }
    memcpy(it->data, line, len);
    it->len = len;
    it->sent = sent;
    it->ngs = ngs;
}

// Можно ли сейчас добавить получателю ещё одну строку из журнала, не вытесняя ответы на команды.
static bool has_room(const peer_t *to)
{
    return to->link != LINK_BLE || s_bq_count < BQ_LEN - 3;
}

static void send_to(const peer_t *to, int len)
{
    if (len <= 0) {
        return;
    }
    if (to->link == LINK_UDP) {
        sendto(s_sock, s_out, len, 0, (const struct sockaddr *)&to->addr, sizeof(to->addr));
    } else if (to->link == LINK_BLE) {
        bq_flush();
        bool droppable = strncmp(s_out, "$NGD", 4) == 0 || strncmp(s_out, "$NGT", 4) == 0;
        if (!droppable) {
            bq_put(s_out, len, 0);
            bq_flush();
        } else if (s_bq_count) {
            s_bt_drops++;       // сначала должны уйти строки из очереди
        } else {
            int n = ble_send(s_out, len);
            if (n == 0) {
                s_bt_drops++;
            } else if (n > 0 && n < len) {
                bq_put(s_out, len, n);      // начатую строку надо дослать целиком
            }
        }
    }
}

static bool same_peer(const peer_t *a, const peer_t *b)
{
    return a->link == b->link &&
           (a->link != LINK_UDP || (a->addr.sin_addr.s_addr == b->addr.sin_addr.s_addr &&
                                    a->addr.sin_port == b->addr.sin_port));
}

static const char *peer_name(const peer_t *p, char *buf, size_t size)
{
    if (p->link == LINK_BLE) {
        strlcpy(buf, "ble", size);
    } else {
        snprintf(buf, size, "%s:%u", inet_ntoa(p->addr.sin_addr), ntohs(p->addr.sin_port));
    }
    return buf;
}

static void send_reply(const peer_t *to, int seq, const char *what)
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
    uint32_t flags = full_flags(p.flags);
    // Поля 11–14 — ускорения; пустые без откалиброванной вертикали или без гироскопа.
    char acc[64] = ",,,";
    if (p.has_acc && (flags & FLAG_IMU_OK) && (flags & MOTION_UP_OK)) {
        snprintf(acc, sizeof(acc), "%ld,%ld,%ld,%ld", (long)p.acc_h1_mms2, (long)p.acc_h2_mms2,
                 (long)p.acc_up_mms2, (long)p.jolt_mms2);
    }
    int len = proto_line(s_out, sizeof(s_out), "NGD,%d,%lu,%lu,%lld,%ld,%llu,%d,%d,%lX,%s",
                         PROTO_VERSION, (unsigned long)s_seq++, (unsigned long)p.t_ms,
                         (long long)p.yaw_mdeg, (long)p.rate_mdps, (unsigned long long)p.dist_mm,
                         p.speed_kmh, p.speed_age_ms, (unsigned long)flags, acc);
    send_to(&s_peer, len);
}

static void send_ngs(const peer_t *to)
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

static void send_nge(const peer_t *to, const event_t *ev)
{
    send_to(to, proto_line(s_out, sizeof(s_out), "NGE,%d,%lu,%lu,%c,%s,%s", PROTO_VERSION,
                           (unsigned long)ev->num, (unsigned long)ev->t_ms, ev->level, ev->code,
                           ev->text));
}

// Сведения о блоке, которые не меняются на ходу: один раз после первого HELLO и по команде INFO.
static void send_ngi(const peer_t *to)
{
    update_info_t u;
    update_get_info(&u);
    uint32_t flash = 0;
    esp_flash_get_size(NULL, &flash);
    send_to(to, proto_line(s_out, sizeof(s_out), "NGI,%d,%s,%s,%s,%s,%s,%lu,%lu,%s,%s,%s",
                           PROTO_VERSION, u.version, u.build, CONFIG_IDF_TARGET, CONFIG_NOGPS_BOARD,
                           s_id, (unsigned long)(flash / 1024), (unsigned long)u.max_image, u.slot,
                           u.slot_state, u.can_update ? "WIFI BLE OTA" : "WIFI BLE"));
}

// ---- обновление прошивки (раздел 18) ----

static void ota_begin(char **f, int n, const peer_t *from, int seq)
{
    obd_status_t obd;
    power_status_t pw;
    obd_get_status(&obd);
    power_get_status(&pw);
    int mv = pw.adc_mv >= 0 ? pw.adc_mv : obd.ecu_mv >= 0 ? obd.ecu_mv : obd.voltage_mv;
    const char *err;
    if (n < 6) {
        err = "VALUE";
    } else if (from->link != LINK_BLE && !cfg_get_int("bench")) {
        err = "LINK";           // куски прошивки идут только по Bluetooth; по Wi-Fi — на плате для стола
    } else if (s_restart_at) {
        err = "BUSY";           // обновление только что принято, блок перезагружается
    } else if (motion_speed_kmh() > 0) {
        err = "MOVING";
    } else if (mv >= 0 && mv < OTA_MIN_MV) {
        err = "LOW_VOLTAGE";    // напряжение неизвестно — не мешаем
    } else {
        err = update_begin(strtoul(f[3], NULL, 10), f[4], f[5]);
    }
    char r[40];
    if (err) {
        snprintf(r, sizeof(r), "ERR,%s", err);
        send_reply(from, seq, r);
        return;
    }
    // Кусок по Bluetooth — сколько влезает в одну запись: MTU минус заголовок записи и смещение.
    ble_state_t b;
    ble_get_state(&b);
    int chunk = from->link == LINK_BLE ? b.mtu - 3 - 4 : UPDATE_CHUNK_MAX;
    snprintf(r, sizeof(r), "OK,%d,%d", chunk > UPDATE_CHUNK_MAX ? UPDATE_CHUNK_MAX : chunk, UPDATE_WINDOW);
    send_reply(from, seq, r);
    s_ota_peer = *from;
    s_ota_state = UPDATE_RECV;
    s_ota_reported = 0;
    s_ota_resend = false;
    s_ota_ngo_at = 0;
    s_ota_end_seq = -1;
}

// Сообщает телефону ход обновления строками NGO и отвечает на OTA_END, когда закончилась проверка.
static void ota_poll(int64_t now)
{
    if (s_ota_state == UPDATE_IDLE) {
        return;
    }
    update_progress_t p;
    update_get_progress(&p);
    s_ota_resend = s_ota_resend || p.resend;
    bool changed = p.state != s_ota_state;
    bool due = changed;
    if (p.state == UPDATE_RECV) {
        due = due || p.received - s_ota_reported >= UPDATE_WINDOW / 2 ||
              now - s_ota_ngo_at >= NGO_PERIOD_US ||
              (s_ota_resend && now - s_ota_ngo_at >= NGO_RESEND_US);
    } else if (p.state == UPDATE_VERIFY) {
        due = due || now - s_ota_ngo_at >= NGO_PERIOD_US;
    }
    if (due) {
        char st[20];
        switch (p.state) {
        case UPDATE_RECV:    strlcpy(st, s_ota_resend ? "RESEND" : "RECV", sizeof(st)); break;
        case UPDATE_VERIFY:  strlcpy(st, "VERIFY", sizeof(st)); break;
        case UPDATE_DONE:    strlcpy(st, "DONE", sizeof(st)); break;
        case UPDATE_ABORTED: strlcpy(st, "ABORTED", sizeof(st)); break;
        default:             snprintf(st, sizeof(st), "ERR_%s", p.err); break;
        }
        send_to(&s_ota_peer, proto_line(s_out, sizeof(s_out), "NGO,%d,%lu,%s", PROTO_VERSION,
                                        (unsigned long)p.received, st));
        s_ota_reported = p.received;
        s_ota_resend = false;
        s_ota_ngo_at = now;
    }
    if (!changed) {
        return;
    }
    s_ota_state = p.state;
    if (p.state == UPDATE_DONE) {
        if (s_ota_end_seq >= 0) {
            send_reply(&s_ota_peer, s_ota_end_seq, "OK");
        }
        s_restart_at = now + RESTART_AFTER_US;
    } else if (p.state == UPDATE_ERROR && s_ota_end_seq >= 0) {
        char r[24];
        snprintf(r, sizeof(r), "ERR,%s", p.err);
        send_reply(&s_ota_peer, s_ota_end_seq, r);
    }
    if (p.state != UPDATE_RECV && p.state != UPDATE_VERIFY) {
        s_ota_state = UPDATE_IDLE;
        s_ota_end_seq = -1;
    }
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

static void handle_cmd(char **f, int n, const peer_t *from, int64_t now)
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
        bool other = !same_peer(&s_peer, from);
        char who[32];
        s_peer = *from;
        s_last_hello = now;
        if (!s_phone) {
            s_phone = true;
            s_sent_ev = events_last();
            events_add('I', "PHONE_CONNECTED", "%s", peer_name(from, who, sizeof(who)));
            s_ngs_now = s_ngi_now = true;
        } else if (other) {
            ESP_LOGI(TAG, "данные теперь идут на %s", peer_name(from, who, sizeof(who)));
            s_ngs_now = s_ngi_now = true;
        }
        return;
    }
    if (strcmp(cmd, "STATUS") == 0) {
        send_ngs(from);
        return;
    }
    if (strcmp(cmd, "INFO") == 0) {
        send_ngi(from);
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
        // Строки NGE и ответ OK шлёт net_task: по Bluetooth журнал уходит не за один раз.
        if (s_replay.active) {
            send_reply(&s_replay.to, s_replay.seq, "OK");
        }
        s_replay.active = true;
        s_replay.to = *from;
        s_replay.seq = seq;
        s_replay.next = n > 3 ? strtoul(f[3], NULL, 10) : 1;
        s_replay.last = events_last();
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
        esp_err_t err = cfg_set(f[3], f[4]);
        send_reply(from, seq, err == ESP_OK ? "OK" : err == ESP_ERR_NOT_FOUND ? "ERR,KEY"
                                                   : err == ESP_ERR_INVALID_ARG ? "ERR,VALUE" : "ERR,NVS");
    } else if (strcmp(cmd, "OTA_BEGIN") == 0) {
        ota_begin(f, n, from, seq);
    } else if (strcmp(cmd, "OTA_END") == 0) {
        imu_status_t imu;
        imu_get_status(&imu);
        const char *err = update_end(imu.state == IMU_STATE_OK);
        if (err) {
            char r[24];
            snprintf(r, sizeof(r), "ERR,%s", err);
            send_reply(from, seq, r);
        } else {
            s_ota_end_seq = seq;    // ответ — когда закончится проверка образа
        }
    } else if (strcmp(cmd, "OTA_ABORT") == 0) {
        update_abort(NULL);
        send_reply(from, seq, "OK");
    } else if (strcmp(cmd, "BT_PAIR") == 0) {
        // Команда пришла по доверенной связи (Wi-Fi с паролем или привязанный телефон): на 2 минуты
        // разрешаем привязать новый телефон.
        ble_pair_open(BLE_PAIR_WINDOW);
        send_reply(from, seq, "OK");
    } else if (strcmp(cmd, "BT_FORGET") == 0) {
        int was = ble_forget();
        if (was < 0) {
            send_reply(from, seq, "ERR,BT");
        } else {
            events_add('I', "BT_FORGET", "%d", was);
            send_reply(from, seq, "OK");
        }
    } else if (strcmp(cmd, "REBOOT") == 0) {
        send_reply(from, seq, "OK");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        send_reply(from, seq, "ERR,UNKNOWN");
    }
}

// Обычно одна строка на датаграмму или на запись (PR-1), но разрезать по \n не мешает.
static void handle_lines(char *buf, const peer_t *from, int64_t now)
{
    char *save = NULL;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        char *f[8];
        int n;
        if (proto_parse(line, f, 8, &n)) {
            handle_cmd(f, n, from, now);
        }
    }
}

static void receive(int64_t now)
{
    char buf[512];
    peer_t from = {.link = LINK_UDP};
    socklen_t fl = sizeof(from.addr);
    int len = recvfrom(s_sock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (struct sockaddr *)&from.addr, &fl);
    if (len <= 0) {
        return;
    }
    // Плата для стола (настройка bench): кусок прошивки по Wi-Fi — '#', смещение, данные.
    if (buf[0] == '#') {
        if (len > 5 && s_ota_state != UPDATE_IDLE && same_peer(&s_ota_peer, &from)) {
            update_chunk((const uint8_t *)buf + 1, len - 1);
        }
        return;
    }
    buf[len] = '\0';
    handle_lines(buf, &from, now);
}

static void receive_ble(int64_t now)
{
    char buf[BLE_RX_MAX];
    peer_t from = {.link = LINK_BLE};
    while (ble_recv(buf, sizeof(buf))) {
        handle_lines(buf, &from, now);
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
    int64_t next_ngd = 0, next_ngs = 0, bt_slow_at = 0;
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
        // Идёт обновление или в очереди Bluetooth есть строки — заглядывать чаще.
        if ((update_active() || s_bq_count || s_replay.active) && due > now + 5000) {
            due = now + 5000;
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

        // Bluetooth: записи в RX складываются в очередь, разбираем их здесь же, что и UDP.
        ble_poll();
        receive_ble(now);
        bq_flush();
        ota_poll(now);
        // После удачного обновления: дать уйти последним строкам и перезагрузиться в новую прошивку.
        if (s_restart_at && now >= s_restart_at && (s_bq_count == 0 || now >= s_restart_at + 1000000)) {
            esp_restart();
        }
        if (s_bt_drops && now - bt_slow_at >= BT_SLOW_US) {
            events_add('W', "BT_SLOW", "%lu lines dropped", (unsigned long)s_bt_drops);
            s_bt_drops = 0;
            bt_slow_at = now;
        }

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

        // Журнал по команде EVENTS.
        while (s_replay.active && has_room(&s_replay.to)) {
            if (s_replay.next > s_replay.last) {
                send_reply(&s_replay.to, s_replay.seq, "OK");
                s_replay.active = false;
                break;
            }
            event_t ev;
            if (events_get(s_replay.next++, &ev)) {
                send_nge(&s_replay.to, &ev);
            }
        }

        // IMU_TEST: строки NGT раз в 0.1 с.
        if (s_test_until > now && now >= s_next_ngt) {
            send_ngt();
            s_next_ngt += NGT_PERIOD_US;
        }

        // Сессия: нет HELLO 3 с — данные больше не шлём.
        if (s_phone && now - s_last_hello > HELLO_TIMEOUT_US) {
            s_phone = false;
            events_add('I', "PHONE_LOST", "%s",
                       s_peer.link == LINK_BLE ? "ble" : inet_ntoa(s_peer.addr.sin_addr));
            update_abort("LINK");   // обновление без телефона не продолжаем
        }
        taskENTER_CRITICAL(&s_lock);
        s_state.phone = s_phone;
        taskEXIT_CRITICAL(&s_lock);
        if (!s_phone) {
            next_ngd = next_ngs = now;
            continue;
        }

        if (now >= next_ngd) {
            // OTA-5: пока идёт обновление, поток NGD стоит, NGS и события идут.
            if (!update_active()) {
                send_ngd();
            }
            int64_t period = 1000000 / cfg_get_int("data_hz");
            next_ngd += period;
            if (next_ngd < now) {
                next_ngd = now + period;    // отстали — не догонять пачкой
            }
        }

        if (s_ngi_now) {
            send_ngi(&s_peer);
            s_ngi_now = false;
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
        while (s_sent_ev < last && has_room(&s_peer)) {
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
        // NET-5 (без энергосбережения Wi-Fi) у точки доступа выполняется само: режимы сна есть только
        // у клиента. Явно WIFI_PS_NONE не ставим: вместе с Bluetooth драйвер его не допускает.
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

    if (strcmp(cfg_get_str("wifi_mode"), "ap") != 0) {
        ESP_LOGW(TAG, "wifi_mode=%s пока не поддерживается (второй этап), поднимаю точку доступа",
                 cfg_get_str("wifi_mode"));
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
    const char *ssid = cfg_get_str("wifi_ssid");
    if (ssid[0]) {
        strlcpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid));
    } else {
        strlcpy((char *)wc.ap.ssid, s_name, sizeof(wc.ap.ssid));
    }
    wc.ap.ssid_len = strlen((char *)wc.ap.ssid);
    strlcpy((char *)wc.ap.password, cfg_get_str("wifi_pass"), sizeof(wc.ap.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    err = esp_wifi_start();
    if (err != ESP_OK) {
        set_wifi(false, true);
        return err;
    }
    ESP_LOGI(TAG, "сеть %s, адрес 192.168.4.1, UDP %d", wc.ap.ssid, NET_PORT);
    return ESP_OK;
}

void net_set_radio(bool on)
{
    if (on == s_radio_on || s_sock < 0) {
        return;
    }
    s_radio_on = on;
    esp_err_t err = on ? esp_wifi_start() : esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi %s: %s", on ? "start" : "stop", esp_err_to_name(err));
        set_wifi(false, true);
    }
    ble_set_enabled(on);
}

esp_err_t net_start(const char *fw_version)
{
    s_fw = fw_version;
    // NET-1, BLE-1: NoGPS-XXXX, XXXX — последние два байта MAC. Имя одно для сети и для Bluetooth.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_id, sizeof(s_id), "%02X%02X", mac[4], mac[5]);
    snprintf(s_name, sizeof(s_name), "NoGPS-%s", s_id);

    // Отказ одного радио не останавливает другое.
    esp_err_t err = wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi: %s", esp_err_to_name(err));
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
    // Новый телефон привязывается только в «окне сопряжения». Оно открывается, когда на блок подали
    // питание или его перезагрузили через USB: и то и другое требует доступа к блоку.
    ble_set_ota_cb(update_chunk);
    esp_reset_reason_t why = esp_reset_reason();
    err = ble_start(s_name, why == ESP_RST_POWERON || why == ESP_RST_USB);
    if (err != ESP_OK) {
        events_add('E', "BT_ERROR", "%s", esp_err_to_name(err));
    }
    s_radio_on = true;
    return xTaskCreatePinnedToCore(net_task, "net_task", 6144, NULL, 10, NULL, 0) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}
