#include "net.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_flash.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
#define BT_SLOW_US        10000000  // событие о потерянных уведомлениях — не чаще раза в 10 с
#define NGO_PERIOD_US     250000    // NGO во время обновления — раз в 0.25 с
#define NGO_RESEND_US     200000    // просьба вернуться и повторить — не чаще
#define OTA_MIN_MV        11800     // обновление не начинается при напряжении ниже 11.8 В
#define RESTART_AFTER_US  1000000   // перезагрузка после удачного обновления
#define BQ_LEN            12        // строк в очереди для Bluetooth
#define QUICK_RUN_US      60000000  // a power-on that has run this long is not a quick one (BLE-16)
#define SEEN_WRITE_S      3600      // the time a phone was seen is written to the flash not more often
#define PEER_MAX          200       // longest PEER payload
#define NO_CONN           0xFFFE    // not BLE_TO_ALL: a line to nobody is dropped

// Биты NGD, которые не знает motion (9.3).
#define FLAG_IMU_OK     0x001
#define FLAG_OBD_ABSENT 0x200
#define FLAG_ERROR      0x400

static const char *s_fw;
static char s_id[5];            // XXXX из имени NoGPS-XXXX
static char s_name[16];
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static net_state_t s_state;

// Состояние net_task.
// The phones sending HELLO, one for each link. The one that has sent it first changes the box (spec 9.5).
typedef struct {
    uint16_t conn;      // NO_CONN: free
    int64_t  since, last_hello;
} phone_t;
static phone_t s_phones[BLE_LINKS];
static bool    s_phone;
static uint16_t s_cal_conn = NO_CONN;
// The phone that has forgotten itself: its link is dropped after the reply has gone.
static uint16_t s_drop_conn = NO_CONN;
static int64_t  s_drop_at;
#define DROP_AFTER_US     500000
static uint32_t s_seq;
static uint32_t s_sent_ev;
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
    int      seq;
    uint16_t conn;
    uint32_t next, last;
} s_replay;

// Обновление прошивки: что телефон уже знает о его ходе.
static update_state_t s_ota_state;      // UPDATE_IDLE — сообщать нечего
static uint32_t s_ota_reported;
static bool     s_ota_resend;
static int64_t  s_ota_ngo_at;
static int      s_ota_end_seq = -1;     // OTA_END, на который ещё не отвечено
static volatile uint16_t s_ota_conn = NO_CONN;  // the phone sending the firmware
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

// ---- очередь строк ----
// По Bluetooth строка уходит не мгновенно: у стека конечная очередь уведомлений. Пропадать может
// только NGD (и NGT): в них накопленные итоги, следующая строка всё возмещает. Остальные строки ждут
// здесь и уходят раньше NGD. Из NGS в очереди держится только самая свежая.

typedef struct {
    uint16_t len;
    uint16_t to;        // a link or BLE_TO_ALL
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
        // -1: the phone has gone, the line is dropped.
        if (ble_send(it->to, it->data, it->len) == 0) {
            return;             // очередь стека заполнена, остальное позже
        }
        s_bq_head = (s_bq_head + 1) % BQ_LEN;
        s_bq_count--;
    }
}

static void bq_put(uint16_t to, const char *line, int len)
{
    bool ngs = strncmp(line, "$NGS", 4) == 0 && to == BLE_TO_ALL;
    bq_item_t *it = NULL;
    int old_ngs = -1;
    for (int i = 0; i < s_bq_count; i++) {
        if (bq_at(i)->ngs) {
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
    it->to = to;
    it->ngs = ngs;
}

// Можно ли сейчас добавить ещё одну строку из журнала, не вытесняя ответы на команды.
static bool has_room(void)
{
    return s_bq_count < BQ_LEN - 3;
}

// Строка из s_out — the phone of the link |to| or all of them. Если их нет, очередь сама опустеет: слать некому.
static void send_line(uint16_t to, int len)
{
    if (len <= 0) {
        return;
    }
    bq_flush();
    bool droppable = strncmp(s_out, "$NGD", 4) == 0 || strncmp(s_out, "$NGT", 4) == 0 ||
                     strncmp(s_out, "$NGX", 4) == 0;
    if (!droppable) {
        bq_put(to, s_out, len);
        bq_flush();
    } else {
        // NGX may be lost when the queue is busy: that is normal peer traffic, not a link fault, so it is not counted.
        bool peer = strncmp(s_out, "$NGX", 4) == 0;
        if (s_bq_count) {
            s_bt_drops += !peer;       // сначала должны уйти строки из очереди
        } else if (ble_send(to, s_out, len) == 0) {
            s_bt_drops += !peer;
        }
    }
}

// The reply goes only to the phone that sent the command: each phone numbers its commands from 1.
static void send_reply(uint16_t to, int seq, const char *what)
{
    send_line(to, proto_line(s_out, sizeof(s_out), "NGA,%d,%s", seq, what));
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
    // Field 15: the shake of the box, with the IMU working whether the vertical is calibrated or not.
    char shake[12] = "";
    if (p.shake_pct >= 0 && (flags & FLAG_IMU_OK)) {
        snprintf(shake, sizeof(shake), "%d", p.shake_pct);
    }
    int len = proto_line(s_out, sizeof(s_out), "NGD,%d,%lu,%lu,%lld,%ld,%llu,%d,%d,%lX,%s,%s",
                         PROTO_VERSION, (unsigned long)s_seq++, (unsigned long)p.t_ms,
                         (long long)p.yaw_mdeg, (long)p.rate_mdps, (unsigned long long)p.dist_mm,
                         p.speed_kmh, p.speed_age_ms, (unsigned long)flags, acc, shake);
    send_line(BLE_TO_ALL, len);
}

static void send_ngs(uint16_t to)
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
    send_line(to, len);
}

static void send_nge(uint16_t to, const event_t *ev)
{
    send_line(to, proto_line(s_out, sizeof(s_out), "NGE,%d,%lu,%lu,%c,%s,%s", PROTO_VERSION,
                         (unsigned long)ev->num, (unsigned long)ev->t_ms, ev->level, ev->code,
                         ev->text));
}

// Сведения о блоке, которые не меняются на ходу: один раз после первого HELLO и по команде INFO.
static void send_ngi(uint16_t to)
{
    update_info_t u;
    update_get_info(&u);
    uint32_t flash = 0;
    esp_flash_get_size(NULL, &flash);
    send_line(to, proto_line(s_out, sizeof(s_out), "NGI,%d,%s,%s,%s,%s,%s,%lu,%lu,%s,%s,%s,%lu,%s",
                         PROTO_VERSION, u.version, u.build, CONFIG_IDF_TARGET, CONFIG_NOGPS_BOARD,
                         s_id, (unsigned long)(flash / 1024), (unsigned long)u.max_image, u.slot,
                         u.slot_state, u.can_update ? "BLE OTA CAR PHONES MULTI PEER RECOVERY"
                                                       : "BLE CAR PHONES MULTI PEER RECOVERY",
                         (unsigned long)cfg_power_ons(), CONFIG_NOGPS_HW_REVISION));
}

// ---- обновление прошивки (раздел 18) ----

static void ota_begin(uint16_t from, char **f, int n, int seq)
{
    obd_status_t obd;
    power_status_t pw;
    obd_get_status(&obd);
    power_get_status(&pw);
    int mv = pw.adc_mv >= 0 ? pw.adc_mv : obd.ecu_mv >= 0 ? obd.ecu_mv : obd.voltage_mv;
    const char *err;
    if (n < 6) {
        err = "VALUE";
    } else if (ble_link_mtu(from) <= 0) {
        err = "LINK";           // the link is gone: do not start an update nobody can receive
    } else if (s_restart_at) {
        err = "BUSY";           // обновление только что принято, блок перезагружается
    } else if (motion_speed_kmh() > 0) {
        err = "MOVING";
    } else if (mv >= 0 && mv < OTA_MIN_MV) {
        err = "LOW_VOLTAGE";    // напряжение неизвестно — не мешаем
    } else {
        // Гироскоп найден, если микросхема опознана — пусть даже в эту секунду обмен с ней сорвался.
        imu_status_t imu;
        imu_get_status(&imu);
        bool imu_found = imu.state == IMU_STATE_OK || imu.state == IMU_STATE_BUS_ERROR ||
                         imu.state == IMU_STATE_NO_DATA;
        err = update_begin(strtoul(f[3], NULL, 10), f[4], f[5], imu_found);
    }
    char r[40];
    if (err) {
        snprintf(r, sizeof(r), "ERR,%s", err);
        send_reply(from, seq, r);
        return;
    }
    // Кусок — сколько влезает в одну запись: MTU минус заголовок записи и смещение.
    int chunk = ble_link_mtu(from) - 4;
    snprintf(r, sizeof(r), "OK,%d,%d", chunk > UPDATE_CHUNK_MAX ? UPDATE_CHUNK_MAX : chunk, UPDATE_WINDOW);
    send_reply(from, seq, r);
    s_ota_conn = from;
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
        send_line(s_ota_conn, proto_line(s_out, sizeof(s_out), "NGO,%d,%lu,%s", PROTO_VERSION,
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
            send_reply(s_ota_conn, s_ota_end_seq, "OK");
        }
        s_restart_at = now + RESTART_AFTER_US;
    } else if (p.state == UPDATE_ERROR && s_ota_end_seq >= 0) {
        char r[24];
        snprintf(r, sizeof(r), "ERR,%s", p.err);
        send_reply(s_ota_conn, s_ota_end_seq, r);
    }
    if (p.state != UPDATE_RECV && p.state != UPDATE_VERIFY) {
        s_ota_state = UPDATE_IDLE;
        s_ota_end_seq = -1;
        s_ota_conn = NO_CONN;
    }
}

static void send_ngt(void)
{
    imu_sample_t x;
    if (!motion_last_sample(&x)) {
        return;
    }
    const float r2md = 180000.0f / 3.14159265f;
    send_line(BLE_TO_ALL, proto_line(s_out, sizeof(s_out), "NGT,%d,%lu,%ld,%ld,%ld,%ld,%ld,%ld", PROTO_VERSION,
                         (unsigned long)(x.t_us / 1000), lroundf(x.gyro[0] * r2md),
                         lroundf(x.gyro[1] * r2md), lroundf(x.gyro[2] * r2md),
                         lroundf(x.accel[0] * 1000), lroundf(x.accel[1] * 1000),
                         lroundf(x.accel[2] * 1000)));
}

// ---- телефоны ----

static phone_t *phone_find(uint16_t conn)
{
    for (int i = 0; i < BLE_LINKS; i++) {
        if (s_phones[i].conn == conn && conn != NO_CONN) {
            return &s_phones[i];
        }
    }
    return NULL;
}

// The phone that sent HELLO first changes the box. A link without HELLO does it only when no phone sends HELLO.
static bool is_owner(uint16_t conn)
{
    phone_t *me = phone_find(conn);
    for (int i = 0; i < BLE_LINKS; i++) {
        phone_t *p = &s_phones[i];
        if (p->conn != NO_CONN && p->conn != conn && (!me || p->since < me->since)) {
            return false;
        }
    }
    return true;
}

// Printable ASCII without the separators of a line and of BT_LIST, up to PHONE_NAME_MAX characters.
static void clean_name(const char *in, char *out, size_t size)
{
    size_t n = 0;
    for (const char *c = in; *c && n + 1 < size && n < PHONE_NAME_MAX; c++) {
        bool bad = *c < 0x20 || *c > 0x7E || strchr(",*$|;", *c);
        out[n++] = bad ? '_' : *c;
    }
    out[n] = '\0';
}

// HELLO,<name>,<Unix time, s>: the box keeps the name and the time of each paired phone for BT_LIST.
static void remember_phone(uint16_t conn, const char *name_in, const char *time_in)
{
    char addr[BLE_ADDR_LEN];
    if (!name_in || !ble_link_addr(conn, addr)) {
        return;
    }
    char name[PHONE_NAME_MAX + 1], old[PHONE_NAME_MAX + 1];
    clean_name(name_in, name, sizeof(name));
    uint32_t seen = time_in ? strtoul(time_in, NULL, 10) : 0, old_seen = 0;
    bool known = cfg_phone_get(addr, old, sizeof(old), &old_seen);
    if (!known || strcmp(name, old) != 0 || seen > old_seen + SEEN_WRITE_S || seen + SEEN_WRITE_S < old_seen) {
        cfg_phone_set(addr, name, seen);
    }
}

static void hello(uint16_t from, char **f, int n, int64_t now)
{
    phone_t *p = phone_find(from);
    if (!p) {
        for (int i = 0; i < BLE_LINKS && !p; i++) {
            if (s_phones[i].conn == NO_CONN) {
                p = &s_phones[i];
            }
        }
        if (!p) {
            return;
        }
        if (!s_phone) {
            s_sent_ev = events_last();      // a phone that joins later asks EVENTS for the older ones
        }
        p->conn = from;
        p->since = now;
        s_phone = true;
        events_add('I', "PHONE_CONNECTED", "ble");
        s_ngs_now = s_ngi_now = true;
        remember_phone(from, n > 3 ? f[3] : NULL, n > 4 ? f[4] : NULL);
        cfg_quick_power_ons_clear();
    }
    p->last_hello = now;
}

// The stack gives the handle of a link that has gone to the next one: nothing meant for the old phone goes there.
static void forget_conn(uint16_t conn)
{
    int k = 0;
    for (int i = 0; i < s_bq_count; i++) {
        if (bq_at(i)->to != conn) {
            *bq_at(k++) = *bq_at(i);
        }
    }
    s_bq_count = k;
    if (s_cal_conn == conn) {
        s_cal_conn = NO_CONN;
    }
    if (s_ota_conn == conn) {
        s_ota_conn = NO_CONN;
    }
    if (s_replay.active && s_replay.conn == conn) {
        s_replay.active = false;
    }
}

// The phone has left: no HELLO for 3 s, or its link has dropped.
static void phones_tick(int64_t now)
{
    s_phone = false;
    for (int i = 0; i < BLE_LINKS; i++) {
        phone_t *p = &s_phones[i];
        if (p->conn == NO_CONN) {
            continue;
        }
        if (now - p->last_hello > HELLO_TIMEOUT_US || !ble_link_up(p->conn)) {
            events_add('I', "PHONE_LOST", "ble");
            if (p->conn == s_ota_conn) {
                update_abort("LINK");   // обновление без телефона не продолжаем
            }
            forget_conn(p->conn);
            p->conn = NO_CONN;
        } else {
            s_phone = true;
        }
    }
}

static void ota_chunk(uint16_t conn, const uint8_t *msg, size_t len)
{
    if (conn == s_ota_conn) {
        update_chunk(msg, len);
    }
}

// The link of the phone that sent HELLO first, NO_CONN when no phone did.
static uint16_t owner_conn(void)
{
    const phone_t *first = NULL;
    for (int i = 0; i < BLE_LINKS; i++) {
        const phone_t *p = &s_phones[i];
        if (p->conn != NO_CONN && (!first || p->since < first->since)) {
            first = p;
        }
    }
    return first ? first->conn : NO_CONN;
}

// BT_LIST: OK,<addr>|<name>|<Unix time seen>|<1 if this link>|<1 if the box takes changes from it>;...,
// <1 if this link changes the box>
static void list_phones(uint16_t from, int seq)
{
    char addrs[BLE_MAX_BONDS][BLE_ADDR_LEN];
    int k = ble_bonded(addrs, BLE_MAX_BONDS);
    if (k < 0) {
        send_reply(from, seq, "ERR,BT");
        return;
    }
    char me[BLE_ADDR_LEN] = "";
    ble_link_addr(from, me);
    char owner[BLE_ADDR_LEN] = "";
    uint16_t owner_link = owner_conn();
    if (owner_link != NO_CONN) {
        ble_link_addr(owner_link, owner);
    }
    static char r[PROTO_LINE_MAX];
    size_t len = strlcpy(r, "OK,", sizeof(r));
    for (int i = 0; i < k && len < sizeof(r); i++) {
        char name[PHONE_NAME_MAX + 1], seen[12] = "";
        uint32_t t = 0;
        if (cfg_phone_get(addrs[i], name, sizeof(name), &t) && t) {
            snprintf(seen, sizeof(seen), "%lu", (unsigned long)t);
        }
        len += snprintf(r + len, sizeof(r) - len, "%s%s|%s|%s|%d|%d", i ? ";" : "", addrs[i], name, seen,
                        strcasecmp(addrs[i], me) == 0, owner[0] && strcasecmp(addrs[i], owner) == 0);
    }
    if (len < sizeof(r)) {
        snprintf(r + len, sizeof(r) - len, ",%d", is_owner(from));
    }
    send_reply(from, seq, r);
}

// ---- команды NGC (9.5) ----

// Commands that change the box: only from the phone that came first, the others get ERR,BUSY.
static bool changes_box(const char *cmd)
{
    static const char *const k[] = {"SET", "CAL_BIAS", "CAL_UP", "CAL_UP2", "CAL_RESET", "CAL_FWD", "CAR_SET",
                                    "OTA_BEGIN", "OTA_END", "OTA_ABORT", "BT_PAIR", "BT_FORGET", "BT_RECOVERY", "REBOOT"};
    for (size_t i = 0; i < sizeof(k) / sizeof(k[0]); i++) {
        if (strcmp(cmd, k[i]) == 0) {
            return true;
        }
    }
    return false;
}

static bool code_ok(const char *code)
{
    return strlen(code) == 6 && strspn(code, "0123456789") == 6;
}

static void handle_cmd(uint16_t from, char **f, int n, int64_t now)
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
        hello(from, f, n, now);
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
    if (changes_box(cmd) && !is_owner(from)) {
        send_reply(from, seq, "ERR,BUSY");
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
            s_cal_conn = from;      // the replies of the calibration go to it
        }
        return;
    }

    if (strcmp(cmd, "CAL_FWD") == 0) {
        // 7.4: «вперёд» учится только по скорости из OBD — второй этап.
        motion_status_t mo;
        motion_get_status(&mo);
        send_reply(from, seq, (mo.flags & MOTION_UP_OK) ? "ERR,NO_OBD" : "ERR,NO_UP");
    } else if (strcmp(cmd, "EVENTS") == 0) {
        // Строки NGE и ответ OK шлёт net_task: журнал уходит не за один раз.
        if (s_replay.active) {
            send_reply(s_replay.conn, s_replay.seq, "OK");
        }
        s_replay.active = true;
        s_replay.seq = seq;
        s_replay.conn = from;
        s_replay.next = n > 3 ? strtoul(f[3], NULL, 10) : 1;
        s_replay.last = events_last();
    } else if (strcmp(cmd, "IMU_TEST") == 0) {
        imu_status_t imu;
        imu_get_status(&imu);
        if (imu.state != IMU_STATE_OK) {
            send_reply(from, seq, "ERR,IMU");
            return;
        }
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
    } else if (strcmp(cmd, "CAR_GET") == 0) {
        static char v[CAR_VALUE_MAX + 1];
        static char r[CAR_VALUE_MAX + 8];
        const char *err = n < 4 ? "KEY" : cfg_car_get(f[3], v, sizeof(v));
        if (err) {
            snprintf(r, sizeof(r), "ERR,%s", err);
        } else if (v[0]) {
            snprintf(r, sizeof(r), "OK,%s", v);
        } else {
            strlcpy(r, "OK", sizeof(r));
        }
        send_reply(from, seq, r);
    } else if (strcmp(cmd, "CAR_SET") == 0) {
        // "CAR_SET,key," erases the key: its value is the empty fifth field.
        const char *err = n < 4 ? "KEY" : n < 5 ? "VALUE" : cfg_car_set(f[3], f[4]);
        char r[16] = "OK";
        if (err) {
            snprintf(r, sizeof(r), "ERR,%s", err);
        }
        send_reply(from, seq, r);
    } else if (strcmp(cmd, "OTA_BEGIN") == 0) {
        ota_begin(from, f, n, seq);
    } else if (strcmp(cmd, "OTA_END") == 0) {
        const char *err = update_end();
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
        // Команда пришла от привязанного телефона: на 2 минуты разрешаем привязать ещё один, with the code given
        // for one phone or with bt_pin.
        ble_state_t b;
        ble_get_state(&b);
        if (n > 3 && !code_ok(f[3])) {
            send_reply(from, seq, "ERR,VALUE");
        } else if (b.bonds >= BLE_MAX_BONDS) {
            send_reply(from, seq, "ERR,FULL");
        } else {
            ble_pair_open(BLE_PAIR_WINDOW, n > 3 ? f[3] : NULL);
            send_reply(from, seq, "OK");
        }
    } else if (strcmp(cmd, "BT_FORGET") == 0 && n > 3) {
        // A phone that forgets itself gets the reply before its link drops.
        char me[BLE_ADDR_LEN] = "";
        bool self = ble_link_addr(from, me) && strcasecmp(me, f[3]) == 0;
        int rc = ble_forget_one(f[3], self ? from : NO_CONN);
        if (rc == 0) {
            events_add('I', "BT_FORGET", "%s", f[3]);
        }
        send_reply(from, seq, rc == 0 ? "OK" : rc == 1 ? "ERR,ADDR" : "ERR,BT");
        if (self && rc == 0) {
            s_drop_conn = from;
            s_drop_at = now + DROP_AFTER_US;
        }
    } else if (strcmp(cmd, "BT_FORGET") == 0) {
        int was = ble_forget();
        if (was < 0) {
            send_reply(from, seq, "ERR,BT");
        } else {
            events_add('I', "BT_FORGET", "%d", was);
            send_reply(from, seq, "OK");
        }
    } else if (strcmp(cmd, "BT_RECOVERY") == 0) {
        // The code of the recovery window, for the owner phone to keep (BLE-16). It never goes into an event.
        char code[CFG_RECOVERY_CODE_LEN + 1], r[16];
        if (cfg_recovery_code(code, false)) {
            snprintf(r, sizeof(r), "OK,%s", code);
        } else {
            strlcpy(r, "ERR,NVS", sizeof(r));
        }
        send_reply(from, seq, r);
    } else if (strcmp(cmd, "BT_LIST") == 0) {
        list_phones(from, seq);
    } else if (strcmp(cmd, "REBOOT") == 0) {
        send_reply(from, seq, "OK");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else {
        send_reply(from, seq, "ERR,UNKNOWN");
    }
}

// PEER,<payload>: the payload goes as $NGX,1,<payload> to the other phones, never back, with no reply. It does
// not change the box, so either phone may send it, but only after HELLO. A bad payload is dropped. |p| is the text after "PEER,".
static void relay_peer(uint16_t from, const char *p)
{
    size_t len = strlen(p);
    if (!phone_find(from) || len == 0 || len > PEER_MAX || strpbrk(p, "*$")) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        if (p[i] < 0x20 || p[i] > 0x7E) {
            return;
        }
    }
    int n = proto_line(s_out, sizeof(s_out), "NGX,%d,%s", PROTO_VERSION, p);
    for (int i = 0; i < BLE_LINKS; i++) {
        uint16_t to = s_phones[i].conn;
        if (to != NO_CONN && to != from) {
            send_line(to, n);
        }
    }
}

// "NGC,<seq>,PEER,": its payload has any number of commas, so the line is not split into fields.
static const char *peer_payload(const char *body)
{
    if (strncmp(body, "NGC,", 4) != 0) {
        return NULL;
    }
    const char *p = body + 4;
    p += strspn(p, "0123456789");
    return strncmp(p, ",PEER,", 6) == 0 && p > body + 4 ? p + 6 : NULL;
}

// Обычно одна строка на запись (BLE-5), но разрезать по \n не мешает.
static void handle_lines(uint16_t from, char *buf, int64_t now)
{
    char *save = NULL;
    for (char *line = strtok_r(buf, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        char *f[8];
        int n;
        char *body;
        if (!proto_check(line, &body)) {
            continue;
        }
        const char *peer = peer_payload(body);
        if (peer) {
            relay_peer(from, peer);
        } else if (proto_split(body, f, 8, &n)) {
            handle_cmd(from, f, n, now);
        }
    }
}

static void receive(int64_t now)
{
    char buf[BLE_RX_MAX];
    uint16_t from;
    while (ble_recv(buf, sizeof(buf), &from)) {
        handle_lines(from, buf, now);
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
        // Идёт обновление или в очереди есть строки — заглядывать чаще.
        if ((update_active() || s_bq_count || s_replay.active) && due > now + 5000) {
            due = now + 5000;
        }
        int64_t wait = due - now;
        if (wait > 0) {
            int ms = (int)((wait + 500) / 1000);
            vTaskDelay(pdMS_TO_TICKS(ms > 0 ? ms : 1));
        }
        now = esp_timer_get_time();

        // Записи в RX стек Bluetooth складывает в очередь, разбираются они здесь.
        ble_poll();
        receive(now);
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
            send_reply(s_cal_conn, r.seq, what);
        }

        // Журнал по команде EVENTS.
        while (s_replay.active && has_room()) {
            if (s_replay.next > s_replay.last) {
                send_reply(s_replay.conn, s_replay.seq, "OK");
                s_replay.active = false;
                break;
            }
            event_t ev;
            if (events_get(s_replay.next++, &ev)) {
                send_nge(s_replay.conn, &ev);
            }
        }

        // IMU_TEST: строки NGT раз в 0.1 с.
        if (s_test_until > now && now >= s_next_ngt) {
            send_ngt();
            s_next_ngt += NGT_PERIOD_US;
        }

        // Сессия: нет HELLO 3 с — данные больше не шлём.
        phones_tick(now);
        if (s_drop_conn != NO_CONN && now >= s_drop_at) {
            ble_drop(s_drop_conn);
            s_drop_conn = NO_CONN;
        }
        if (now >= QUICK_RUN_US) {
            cfg_quick_power_ons_clear();
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
            send_ngi(BLE_TO_ALL);
            s_ngi_now = false;
        }

        ngs_sig_t sig;
        ngs_sig(&sig);
        if (s_ngs_now || now >= next_ngs || memcmp(&sig, &sig_sent, sizeof(sig)) != 0) {
            send_ngs(BLE_TO_ALL);
            sig_sent = sig;
            s_ngs_now = false;
            next_ngs = now + NGS_PERIOD_US;
        }

        // Новые события — сразу.
        uint32_t last = events_last();
        while (s_sent_ev < last && has_room()) {
            event_t ev;
            if (events_get(++s_sent_ev, &ev)) {
                send_nge(BLE_TO_ALL, &ev);
            }
        }
    }
}

// ---- запуск ----

void net_set_radio(bool on)
{
    if (on == s_radio_on) {
        return;
    }
    s_radio_on = on;
    ble_set_enabled(on);
}

esp_err_t net_start(const char *fw_version)
{
    s_fw = fw_version;
    // NET-9, BLE-2: NoGPS-XXXX, XXXX — два последних байта адреса, который был у точки доступа Wi-Fi
    // блока. Wi-Fi в блоке больше нет, а имя и номер прежние: по ним телефон узнаёт привязанный блок
    // и помнит его счётчик включений. Адрес берётся из eFuse, радио для этого не нужно.
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_id, sizeof(s_id), "%02X%02X", mac[4], mac[5]);
    snprintf(s_name, sizeof(s_name), "NoGPS-%s", s_id);

    // Новый телефон привязывается только в «окне сопряжения». Оно открывается, когда на блок подали
    // питание или его перезагрузили через USB: и то и другое требует доступа к блоку.
    // Новый телефон привязывается в окне сопряжения after a power-on only while the box has no phones. Three quick
    // power-ons in a row open the recovery window: a new phone paired in it replaces all the old ones (BLE-16).
    for (int i = 0; i < BLE_LINKS; i++) {
        s_phones[i].conn = NO_CONN;
    }
    ble_set_ota_cb(ota_chunk);
    esp_reset_reason_t why = esp_reset_reason();
    bool reset = why == ESP_RST_POWERON && cfg_quick_power_ons() >= QUICK_POWER_ONS_RESET;
    if (reset) {
        cfg_quick_power_ons_clear();
    }
    esp_err_t err = ble_start(s_name, why == ESP_RST_POWERON || why == ESP_RST_USB, reset);
    if (err != ESP_OK) {
        events_add('E', "BT_ERROR", "%s", esp_err_to_name(err));
    }
    s_radio_on = true;
    return xTaskCreatePinnedToCore(net_task, "net_task", 6144, NULL, 10, NULL, 0) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}
