#include "ble.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "config.h"
#include "events.h"

#define TAG "ble"

#define SECURE_DEADLINE_S   60      // подключился и за минуту не прошёл проверку — отключить
#define SECREQ_DELAY_MS     1500    // столько ждём, что телефон включит шифрование сам
#define LOCKOUT_S           60      // пауза после трёх неверных кодов подряд
#define MAX_FAILS           3
#define RX_QUEUE            4
#define MSYS_RESERVE        6       // столько буферов стека оставляем ему самому

// Хвост у всех UUID одинаковый, отличается число n: 02CB000n-C0C3-40D0-819C-5A85998DCD99.
#define NOGPS_UUID(n) BLE_UUID128_INIT(0x99, 0xCD, 0x8D, 0x99, 0x85, 0x5A, 0x9C, 0x81, \
                                       0xD0, 0x40, 0xC3, 0xC0, (n), 0x00, 0xCB, 0x02)

static const ble_uuid128_t s_uuid_svc = NOGPS_UUID(1);
static const ble_uuid128_t s_uuid_tx  = NOGPS_UUID(2);
static const ble_uuid128_t s_uuid_rx  = NOGPS_UUID(3);
static const ble_uuid128_t s_uuid_ota = NOGPS_UUID(4);

typedef struct {
    uint16_t conn;
    char data[BLE_RX_MAX];
} rx_item_t;

// A connection. Changed in the task of the stack, read in net_task.
typedef struct {
    uint16_t conn;              // BLE_HS_CONN_HANDLE_NONE: the slot is free
    bool secure, subscribed, pairing;
    bool peer_bonded, secreq_sent;
    uint8_t tune_step;          // сколько просьб о настройке связи уже отправлено
    uint16_t mtu;
    uint32_t at_s;              // секунды от включения
    uint32_t at_ms, secure_ms;  // миллисекунды от включения
} link_t;

void ble_store_config_init(void);

// Стек нельзя останавливать, пока другая задача шлёт через него уведомление.
static SemaphoreHandle_t s_lock;
static bool s_up;
static volatile bool s_stopping;
static QueueHandle_t s_rx;
static char s_name[32];
static uint8_t s_addr_type;
static uint16_t s_tx_handle, s_rx_handle, s_ota_handle;
static void (*s_ota_cb)(uint16_t conn, const uint8_t *msg, size_t len);
// Данные производителя в рекламе: компания 0xFFFF, версия формата, флаги (ble.h).
#define ADV_OPEN 0x01
#define ADV_FULL 0x02
#define ADV_PHONES 0x04
static uint8_t s_mfg[4] = {0xFF, 0xFF, 1, 0};
static volatile uint8_t s_adv_flags;

static link_t s_links[BLE_LINKS];
static volatile uint32_t s_window_until, s_lock_until;    // секунды от включения
// The code of the pairing window, empty when it is closed. A code from BT_PAIR,<code> works for one phone.
static char s_code[7];
static bool s_code_once;
// The code is set in net_task and read in the task of the stack.
static portMUX_TYPE s_code_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int s_bonds;
static int s_fails;
// Done once the stack is up: the box was powered on, erase the phones.
static bool s_power_window, s_reset;

// ---- окно сопряжения ----

static uint32_t now_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000);
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static bool pairing_allowed(void)
{
    uint32_t now = now_s();
    return now < s_window_until && now >= s_lock_until;
}

static bool full(void)
{
    return s_bonds >= BLE_MAX_BONDS;
}

static link_t *link_find(uint16_t conn)
{
    for (int i = 0; i < BLE_LINKS; i++) {
        if (s_links[i].conn == conn && conn != BLE_HS_CONN_HANDLE_NONE) {
            return &s_links[i];
        }
    }
    return NULL;
}

static int links_used(void)
{
    int n = 0;
    for (int i = 0; i < BLE_LINKS; i++) {
        n += s_links[i].conn != BLE_HS_CONN_HANDLE_NONE;
    }
    return n;
}

static void link_clear(link_t *l)
{
    memset(l, 0, sizeof(*l));
    l->conn = BLE_HS_CONN_HANDLE_NONE;
    l->mtu = BLE_ATT_MTU_DFLT;
}

static void addr_str(const ble_addr_t *a, char out[BLE_ADDR_LEN])
{
    snprintf(out, BLE_ADDR_LEN, "%02X:%02X:%02X:%02X:%02X:%02X", a->val[5], a->val[4], a->val[3], a->val[2],
             a->val[1], a->val[0]);
}

// Уровень 1 — стек отклоняет любой запрос сопряжения; уровень 3 — принимает только сопряжение с
// кодом. Уже привязанные телефоны шифруют связь сохранённым ключом при любом уровне.
static void gate(void)
{
    ble_hs_cfg.sm_sec_lvl = pairing_allowed() ? 3 : 1;
}

static void pair_failed(int status)
{
    s_fails++;
    events_add('W', "BT_PAIR_FAIL", "status %d", status);
    if (s_fails >= MAX_FAILS) {
        s_fails = 0;
        s_lock_until = now_s() + LOCKOUT_S;
        events_add('W', "BT_LOCKOUT", "%d s", LOCKOUT_S);
    }
    gate();
}

// Штатная обработка освобождает место под новую привязку, удаляя самую старую. A full box refuses a new phone
// before this (PASSKEY_ACTION): a paired phone is removed only on purpose, by BT_FORGET.
static int on_store_status(struct ble_store_status_event *ev, void *arg)
{
    if (ev->event_code == BLE_STORE_EVENT_OVERFLOW) {
        ESP_LOGW(TAG, "нет места для привязки");
        return BLE_HS_ENOMEM;
    }
    return ble_store_util_status_rr(ev, arg);
}

static bool is_bonded(const ble_addr_t *id)
{
    struct ble_store_key_sec key = {0};
    struct ble_store_value_sec val;
    key.peer_addr = *id;
    return ble_store_read_peer_sec(&key, &val) == 0;
}

static void count_bonds(void)
{
    int n = 0;
    ble_store_util_count(BLE_STORE_OBJ_TYPE_PEER_SEC, &n);
    s_bonds = n;
}

// ---- реклама и соединение ----

static int gap_event(struct ble_gap_event *ev, void *arg);

// BLE-1: в рекламном пакете — UUID сервиса и байт состояния, имя — в ответе на сканирование.
// Данные можно менять и на ходу, не останавливая рекламу.
static uint8_t adv_flags(void)
{
    return (pairing_allowed() ? ADV_OPEN : 0) | (full() ? ADV_FULL : 0) | (s_bonds > 0 ? ADV_PHONES : 0);
}

static int set_adv_data(void)
{
    uint8_t flags = adv_flags();
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128 = &s_uuid_svc;
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    s_mfg[3] = flags;
    f.mfg_data = s_mfg;
    f.mfg_data_len = sizeof(s_mfg);
    int rc = ble_gap_adv_set_fields(&f);
    if (rc == 0) {
        s_adv_flags = flags;
    }
    return rc;
}

// The stack stops advertising when a phone connects: start it again while there is room for one more.
static void advertise(void)
{
    if (s_stopping || links_used() >= BLE_LINKS || ble_gap_adv_active()) {
        return;
    }
    struct ble_hs_adv_fields r = {0};
    r.name = (const uint8_t *)s_name;
    r.name_len = strlen(s_name);
    r.name_is_complete = 1;
    struct ble_gap_adv_params p = {0};
    p.conn_mode = BLE_GAP_CONN_MODE_UND;
    p.disc_mode = BLE_GAP_DISC_MODE_GEN;
    p.itvl_min = BLE_GAP_ADV_ITVL_MS(100);
    p.itvl_max = BLE_GAP_ADV_ITVL_MS(150);
    int rc = set_adv_data();
    if (rc == 0) {
        rc = ble_gap_adv_rsp_set_fields(&r);
    }
    if (rc == 0) {
        rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &p, gap_event, NULL);
    }
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "реклама не запустилась: %d", rc);
    }
}

// Что блок просит у телефона после соединения. Сразу после соединения — ничего: телефон в это время
// сам включает шифрование для привязанного устройства, и встречные запросы блока с ним сталкиваются
// (связь с уже привязанным телефоном рвалась через 6 с). Поэтому:
//  - шифрование просим, только если телефон за полторы секунды не включил его сам;
//  - настройку связи (BLE-5: интервал 15–30 мс, длинные пакеты, 2M PHY) просим после того, как
//    шифрование включилось, по одной просьбе с паузами. Телефон вправе отказать.
// Зовётся из net_task под s_lock.
static void link_steps(link_t *l)
{
    uint16_t conn = l->conn;
    uint32_t now = now_ms();
    if (!l->secure) {
        if (!l->secreq_sent && !l->pairing && now - l->at_ms >= SECREQ_DELAY_MS &&
            (l->peer_bonded || pairing_allowed())) {
            l->secreq_sent = true;
            int rc = ble_gap_security_initiate(conn);
            ESP_LOGI(TAG, "телефон шифрование не включил — прошу сам (%d)", rc);
        }
        return;
    }
    static const uint16_t at_ms[] = {500, 1500, 2500};
    if (l->tune_step >= 3 || now - l->secure_ms < at_ms[l->tune_step]) {
        return;
    }
    struct ble_gap_conn_desc d;
    int rc = 0;
    switch (l->tune_step++) {
    case 0:
        // Интервал уже подходит (у Android и iOS обычно 30 мс) — не трогаем.
        if (ble_gap_conn_find(conn, &d) == 0 &&
            (d.conn_itvl < BLE_GAP_CONN_ITVL_MS(15) || d.conn_itvl > BLE_GAP_CONN_ITVL_MS(30) ||
             d.conn_latency != 0)) {
            struct ble_gap_upd_params p = {
                .itvl_min = BLE_GAP_CONN_ITVL_MS(15),
                .itvl_max = BLE_GAP_CONN_ITVL_MS(30),
                .latency = 0,
                .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(4000),
            };
            rc = ble_gap_update_params(conn, &p);
            ESP_LOGI(TAG, "прошу интервал 15–30 мс (%d)", rc);
        }
        break;
    case 1:
        rc = ble_gap_set_data_len(conn, 251, 2120);
        ESP_LOGI(TAG, "прошу длинные пакеты (%d)", rc);
        break;
    default:
        rc = ble_gap_set_prefered_le_phy(conn, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
        ESP_LOGI(TAG, "прошу 2M PHY (%d)", rc);
        break;
    }
}

static int gap_event(struct ble_gap_event *ev, void *arg)
{
    struct ble_gap_conn_desc d;
    link_t *l;

    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status != 0) {
            advertise();
            return 0;
        }
        l = NULL;
        for (int i = 0; i < BLE_LINKS && !l; i++) {
            if (s_links[i].conn == BLE_HS_CONN_HANDLE_NONE) {
                l = &s_links[i];
            }
        }
        if (!l) {
            ble_gap_terminate(ev->connect.conn_handle, BLE_ERR_CONN_LIMIT);
            return 0;
        }
        link_clear(l);
        l->conn = ev->connect.conn_handle;
        l->at_s = now_s();
        l->at_ms = now_ms();
        gate();
        if (ble_gap_conn_find(ev->connect.conn_handle, &d) == 0) {
            bool bonded = is_bonded(&d.peer_id_addr);
            l->peer_bonded = bonded;
            ESP_LOGI(TAG, "подключился %02X:%02X:%02X:%02X:%02X:%02X, %s, интервал %d.%02d мс, links %d",
                     d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
                     d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0],
                     bonded ? "привязан" : pairing_allowed() ? "новый, окно сопряжения открыто"
                                                             : "новый, окно сопряжения закрыто",
                     d.conn_itvl * 125 / 100, d.conn_itvl * 125 % 100, links_used());
        }
        advertise();
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        l = link_find(ev->disconnect.conn.conn_handle);
        if (!l) {
            advertise();
            return 0;
        }
        ESP_LOGI(TAG, "отключился через %lu мс, причина %d (0x%X)%s",
                 (unsigned long)(now_ms() - l->at_ms), ev->disconnect.reason, ev->disconnect.reason,
                 l->secure ? "" : ", шифрование так и не включилось");
        if (l->pairing) {
            pair_failed(ev->disconnect.reason);
        }
        link_clear(l);
        advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        l = link_find(ev->enc_change.conn_handle);
        if (!l) {
            return 0;
        }
        bool was_pairing = l->pairing;
        l->pairing = false;
        if (ev->enc_change.status != 0) {
            if (was_pairing) {
                pair_failed(ev->enc_change.status);
            } else {
                ESP_LOGW(TAG, "шифрование не включилось: %d", ev->enc_change.status);
            }
            return 0;
        }
        if (ble_gap_conn_find(ev->enc_change.conn_handle, &d) != 0) {
            return 0;
        }
        if (!d.sec_state.encrypted || !d.sec_state.authenticated) {
            // Сопряжение без кода не принимаем и привязку от него не оставляем.
            ESP_LOGW(TAG, "сопряжение без кода — отклонено");
            ble_store_util_delete_peer(&d.peer_id_addr);
            ble_gap_terminate(ev->enc_change.conn_handle, BLE_ERR_AUTH_FAIL);
            return 0;
        }
        s_fails = 0;
        l->secure_ms = now_ms();
        l->secure = true;
        count_bonds();
        if (was_pairing) {
            char a[BLE_ADDR_LEN];
            addr_str(&d.peer_id_addr, a);
            // A new phone has no name until its HELLO: drop the record of a phone forgotten with the same address.
            cfg_phone_erase(a);
            events_add('I', "BT_PAIRED", "%s bonds %d", a, (int)s_bonds);
            if (s_code_once) {
                taskENTER_CRITICAL(&s_code_lock);
                s_code[0] = '\0';
                s_window_until = now_s();
                taskEXIT_CRITICAL(&s_code_lock);
                gate();
            }
        } else {
            ESP_LOGI(TAG, "связь зашифрована через %lu мс после соединения",
                     (unsigned long)(l->secure_ms - l->at_ms));
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        l = link_find(ev->subscribe.conn_handle);
        if (l && ev->subscribe.attr_handle == s_tx_handle) {
            l->subscribed = ev->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        l = link_find(ev->mtu.conn_handle);
        if (l) {
            l->mtu = ev->mtu.value;
        }
        ESP_LOGI(TAG, "MTU %d", ev->mtu.value);
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        if (ble_gap_conn_find(ev->conn_update.conn_handle, &d) == 0) {
            ESP_LOGI(TAG, "интервал соединения %d.%02d мс", d.conn_itvl * 125 / 100,
                     d.conn_itvl * 125 % 100);
        }
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        // Привязанный телефон сопрягается заново (на нём привязку удалили). Сюда доходит только при
        // открытом окне сопряжения: при закрытом запрос отклонён раньше.
        if (ble_gap_conn_find(ev->repeat_pairing.conn_handle, &d) == 0) {
            ble_store_util_delete_peer(&d.peer_id_addr);
            count_bonds();
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        // Блок «показывает» код, человек вводит его на телефоне: the code of the window.
        l = link_find(ev->passkey.conn_handle);
        if (ev->passkey.params.action != BLE_SM_IOACT_DISP || !pairing_allowed() || !l || !s_code[0]) {
            ble_gap_terminate(ev->passkey.conn_handle, BLE_ERR_AUTH_FAIL);
        } else if (full() && !l->peer_bonded) {
            // A new phone doesn't push out a paired one.
            events_add('W', "BT_FULL", "%d", (int)s_bonds);
            ble_gap_terminate(ev->passkey.conn_handle, BLE_ERR_AUTH_FAIL);
        } else {
            taskENTER_CRITICAL(&s_code_lock);
            uint32_t passkey = (uint32_t)atoi(s_code);
            taskEXIT_CRITICAL(&s_code_lock);
            struct ble_sm_io io = {
                .action = BLE_SM_IOACT_DISP,
                .passkey = passkey,
            };
            l->pairing = true;
            ble_sm_inject_io(ev->passkey.conn_handle, &io);
        }
        return 0;

    default:
        return 0;
    }
}

// ---- GATT ----

static int chr_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }
    link_t *l = link_find(conn);
    if (!l || !l->secure) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    if (attr == s_rx_handle) {
        rx_item_t it;
        it.conn = conn;
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= sizeof(it.data) ||
            ble_hs_mbuf_to_flat(ctxt->om, it.data, sizeof(it.data) - 1, &len) != 0) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        it.data[len] = '\0';
        xQueueSend(s_rx, &it, 0);
    } else if (attr == s_ota_handle && s_ota_cb) {
        uint8_t msg[BLE_RX_MAX];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len <= sizeof(msg) && ble_hs_mbuf_to_flat(ctxt->om, msg, sizeof(msg), &len) == 0) {
            s_ota_cb(conn, msg, len);
        }
    }
    return 0;
}

static const struct ble_gatt_svc_def s_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_uuid_svc.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_uuid_tx.u,
                .access_cb = chr_access,
                .val_handle = &s_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &s_uuid_rx.u,
                .access_cb = chr_access,
                .val_handle = &s_rx_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {
                .uuid = &s_uuid_ota.u,
                .access_cb = chr_access,
                .val_handle = &s_ota_handle,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC |
                         BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {0},
        },
    },
    {0},
};

// ---- стек ----

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "стек сброшен, причина %d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &s_addr_type);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "нет адреса: %d", rc);
        return;
    }
    count_bonds();
    // Once, at the first start of the stack after a power-on.
    if (s_reset) {
        s_reset = false;
        int was = s_bonds;
        if (ble_store_clear() == 0) {
            cfg_phone_erase(NULL);
            count_bonds();
            events_add('W', "BT_RESET", "%d", was);
        }
    }
    if (s_power_window) {
        s_power_window = false;
        // A box with phones adds a new one only by BT_PAIR from a paired phone.
        if (s_bonds == 0) {
            ble_pair_open(BLE_PAIR_WINDOW, NULL);
        }
    }
    advertise();
    ESP_LOGI(TAG, "реклама %s, привязано телефонов: %d", s_name, (int)s_bonds);
}

static void host_task(void *arg)
{
    nimble_port_run();      // возвращается после nimble_port_stop()
    nimble_port_freertos_deinit();
}

static esp_err_t stack_up(void)
{
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        return err;
    }
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = on_store_status;
    // BLE-9: сопряжение с запоминанием, только LE Secure Connections, код вводится на телефоне.
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_DISPLAY_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_sc_only = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    gate();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_svcs);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_svcs);
    }
    if (rc == 0) {
        rc = ble_svc_gap_device_name_set(s_name);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "сервис не зарегистрирован: %d", rc);
        nimble_port_deinit();
        return ESP_FAIL;
    }
    ble_store_config_init();
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}

esp_err_t ble_start(const char *name, bool pair_window, bool reset)
{
    strlcpy(s_name, name, sizeof(s_name));
    for (int i = 0; i < BLE_LINKS; i++) {
        link_clear(&s_links[i]);
    }
    s_lock = xSemaphoreCreateMutex();
    s_rx = xQueueCreate(RX_QUEUE, sizeof(rx_item_t));
    if (!s_lock || !s_rx) {
        return ESP_ERR_NO_MEM;
    }
    s_power_window = pair_window;
    s_reset = reset;
    esp_err_t err = stack_up();
    s_up = err == ESP_OK;
    return err;
}

void ble_set_enabled(bool on)
{
    if (!s_lock) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (on && !s_up) {
        esp_err_t err = stack_up();
        s_up = err == ESP_OK;
        if (!s_up) {
            ESP_LOGE(TAG, "Bluetooth не включился: %s", esp_err_to_name(err));
        }
    } else if (!on && s_up) {
        // Останавливает рекламу, рвёт соединение и выключает контроллер.
        s_stopping = true;
        int rc = nimble_port_stop();
        if (rc == 0) {
            nimble_port_deinit();
            s_up = false;
        } else {
            ESP_LOGE(TAG, "Bluetooth не выключился: %d", rc);
        }
        for (int i = 0; i < BLE_LINKS; i++) {
            link_clear(&s_links[i]);
        }
        s_stopping = false;
    }
    xSemaphoreGive(s_lock);
}

void ble_get_state(ble_state_t *out)
{
    out->up = s_up;
    out->advertising = s_up && ble_gap_adv_active();
    out->connected = false;
    out->secure = false;
    for (int i = 0; i < BLE_LINKS; i++) {
        out->connected |= s_links[i].conn != BLE_HS_CONN_HANDLE_NONE;
        out->secure |= s_links[i].secure;
    }
    out->pairing = pairing_allowed();
    out->bonds = s_bonds;
}

bool ble_link_up(uint16_t conn)
{
    link_t *l = link_find(conn);
    return l && l->secure;
}

int ble_link_mtu(uint16_t conn)
{
    link_t *l = link_find(conn);
    return l ? l->mtu - 3 : 0;
}

bool ble_link_addr(uint16_t conn, char addr[BLE_ADDR_LEN])
{
    struct ble_gap_conn_desc d;
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        return false;
    }
    bool ok = s_up && link_find(conn) && ble_gap_conn_find(conn, &d) == 0;
    if (ok) {
        addr_str(&d.peer_id_addr, addr);
    }
    xSemaphoreGive(s_lock);
    return ok;
}

static int chunks(int len, int max)
{
    return (len + max - 1) / max;
}

int ble_send(uint16_t to, const char *data, int len)
{
    if (!s_lock || xSemaphoreTake(s_lock, 0) != pdTRUE) {
        return 0;
    }
    int sent = -1;
    int need = 0;
    for (int i = 0; s_up && i < BLE_LINKS; i++) {
        link_t *l = &s_links[i];
        if (l->secure && l->subscribed && (to == BLE_TO_ALL || to == l->conn)) {
            // A notification may take two buffers of the stack.
            need += 2 * chunks(len, l->mtu - 3);
        }
    }
    if (need > 0) {
        sent = 0;
    }
    // BLE-3: поток байт. Строка, которая влезает в уведомление, не делится. A line goes to all links or to none:
    // a link the stack fails in the middle gets a cut line, the phone drops it by its checksum.
    if (need > 0 && os_msys_num_free() >= MSYS_RESERVE + need) {
        for (int i = 0; i < BLE_LINKS; i++) {
            link_t *l = &s_links[i];
            if (!l->secure || !l->subscribed || (to != BLE_TO_ALL && to != l->conn)) {
                continue;
            }
            int max = l->mtu - 3;
            for (int off = 0; off < len; off += max) {
                int n = len - off < max ? len - off : max;
                struct os_mbuf *om = ble_hs_mbuf_from_flat(data + off, n);
                // Буфер функция освобождает сама, в том числе при ошибке.
                if (!om || ble_gatts_notify_custom(l->conn, s_tx_handle, om) != 0) {
                    break;
                }
            }
        }
        sent = len;
    }
    xSemaphoreGive(s_lock);
    return sent;
}

bool ble_recv(char *buf, size_t size, uint16_t *from)
{
    rx_item_t it;
    if (!s_rx || xQueueReceive(s_rx, &it, 0) != pdTRUE) {
        return false;
    }
    strlcpy(buf, it.data, size);
    *from = it.conn;
    return true;
}

void ble_poll(void)
{
    if (!s_lock) {
        return;
    }
    gate();
    uint32_t now = now_s();
    if (now >= s_window_until && s_code[0]) {
        taskENTER_CRITICAL(&s_code_lock);
        s_code[0] = '\0';    // the code of a closed window doesn't work any more
        taskEXIT_CRITICAL(&s_code_lock);
    }
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) {
        return;
    }
    if (s_up) {
        // Окно сопряжения открылось или закрылось, the box got full or has room again — поправить рекламу.
        if (adv_flags() != s_adv_flags) {
            set_adv_data();
        }
        for (int i = 0; i < BLE_LINKS; i++) {
            link_t *l = &s_links[i];
            if (l->conn == BLE_HS_CONN_HANDLE_NONE) {
                continue;
            }
            link_steps(l);
            // Чужой, который подключился и молчит, не должен занимать соединение вечно.
            if (!l->secure && now - l->at_s >= SECURE_DEADLINE_S) {
                ESP_LOGW(TAG, "проверка не пройдена за %d с — отключаю", SECURE_DEADLINE_S);
                ble_gap_terminate(l->conn, BLE_ERR_REM_USER_CONN_TERM);
                l->at_s = now;
            }
        }
    }
    xSemaphoreGive(s_lock);
}

void ble_set_ota_cb(void (*cb)(uint16_t conn, const uint8_t *msg, size_t len))
{
    s_ota_cb = cb;
}

void ble_pair_open(int seconds, const char *code)
{
    taskENTER_CRITICAL(&s_code_lock);
    strlcpy(s_code, code ? code : cfg_get_str("bt_pin"), sizeof(s_code));
    s_code_once = code != NULL;
    s_window_until = now_s() + seconds;
    taskEXIT_CRITICAL(&s_code_lock);
    gate();
    events_add('I', "BT_PAIR_OPEN", "%d s%s", seconds, code ? " code" : "");
}

int ble_bonded(char addrs[][BLE_ADDR_LEN], int max)
{
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        return -1;
    }
    ble_addr_t peers[BLE_MAX_BONDS];
    int n = -1;
    if (s_up && ble_store_util_bonded_peers(peers, &n, BLE_MAX_BONDS) != 0) {
        n = -1;
    }
    for (int i = 0; i < n && i < max; i++) {
        addr_str(&peers[i], addrs[i]);
    }
    xSemaphoreGive(s_lock);
    return n < max ? n : max;
}

int ble_forget_one(const char *addr, uint16_t keep)
{
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        return -1;
    }
    int rc = -1;
    ble_addr_t peers[BLE_MAX_BONDS];
    int n = 0;
    if (s_up && ble_store_util_bonded_peers(peers, &n, BLE_MAX_BONDS) == 0) {
        rc = 1;
        for (int i = 0; i < n && rc == 1; i++) {
            char a[BLE_ADDR_LEN];
            addr_str(&peers[i], a);
            if (strcasecmp(a, addr) != 0) {
                continue;
            }
            // Its link goes too: a phone the box has forgotten gets no data.
            for (int k = 0; k < BLE_LINKS; k++) {
                struct ble_gap_conn_desc d;
                if (s_links[k].conn != BLE_HS_CONN_HANDLE_NONE && s_links[k].conn != keep &&
                    ble_gap_conn_find(s_links[k].conn, &d) == 0 &&
                    ble_addr_cmp(&d.peer_id_addr, &peers[i]) == 0) {
                    ble_gap_terminate(s_links[k].conn, BLE_ERR_REM_USER_CONN_TERM);
                }
            }
            rc = ble_store_util_delete_peer(&peers[i]) == 0 ? 0 : -1;
            if (rc == 0) {
                cfg_phone_erase(a);
            }
        }
        count_bonds();
    }
    xSemaphoreGive(s_lock);
    return rc;
}

void ble_drop(uint16_t conn)
{
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        return;
    }
    if (s_up && link_find(conn)) {
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
    }
    xSemaphoreGive(s_lock);
}

int ble_forget(void)
{
    if (!s_lock || xSemaphoreTake(s_lock, pdMS_TO_TICKS(500)) != pdTRUE) {
        return -1;
    }
    int n = -1;
    if (s_up) {
        n = s_bonds;
        if (ble_store_clear() != 0) {
            n = -1;
        } else {
            cfg_phone_erase(NULL);
        }
        count_bonds();
    }
    xSemaphoreGive(s_lock);
    return n;
}
