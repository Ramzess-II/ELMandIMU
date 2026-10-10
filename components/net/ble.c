#include "ble.h"

#include <stdlib.h>
#include <string.h>

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
    char data[BLE_RX_MAX];
} rx_item_t;

void ble_store_config_init(void);

// Стек нельзя останавливать, пока другая задача шлёт через него уведомление.
static SemaphoreHandle_t s_lock;
static bool s_up;
static volatile bool s_stopping;
static QueueHandle_t s_rx;
static char s_name[32];
static uint8_t s_addr_type;
static uint16_t s_tx_handle, s_rx_handle, s_ota_handle;
static void (*s_ota_cb)(const uint8_t *msg, size_t len);
// Данные производителя в рекламе: компания 0xFFFF, версия формата, флаги (бит 0 — окно открыто).
static uint8_t s_mfg[4] = {0xFF, 0xFF, 1, 0};
static volatile bool s_adv_open;

// Меняются в задаче стека, читаются из net_task.
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_secure, s_subscribed, s_pairing;
static volatile uint16_t s_mtu = BLE_ATT_MTU_DFLT;
static volatile uint32_t s_conn_at, s_window_until, s_lock_until;    // секунды от включения
static volatile uint32_t s_conn_ms, s_secure_ms;                     // миллисекунды от включения
static volatile bool s_peer_bonded, s_secreq_sent;
static volatile uint8_t s_tune_step;    // сколько просьб о настройке связи уже отправлено
static volatile int s_bonds;
static int s_fails;

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

// Штатная обработка освобождает место под новую привязку, удаляя самую старую, уже по одному запросу
// сопряжения. Вне окна место не освобождаем: иначе посторонний вытеснил бы привязанные телефоны.
static int on_store_status(struct ble_store_status_event *ev, void *arg)
{
    if (ev->event_code == BLE_STORE_EVENT_OVERFLOW && !pairing_allowed()) {
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
static int set_adv_data(void)
{
    bool open = pairing_allowed();
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids128 = &s_uuid_svc;
    f.num_uuids128 = 1;
    f.uuids128_is_complete = 1;
    s_mfg[3] = open ? 0x01 : 0x00;
    f.mfg_data = s_mfg;
    f.mfg_data_len = sizeof(s_mfg);
    int rc = ble_gap_adv_set_fields(&f);
    if (rc == 0) {
        s_adv_open = open;
    }
    return rc;
}

static void advertise(void)
{
    if (s_stopping) {
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
static void link_steps(uint16_t conn)
{
    uint32_t now = now_ms();
    if (!s_secure) {
        if (!s_secreq_sent && !s_pairing && now - s_conn_ms >= SECREQ_DELAY_MS &&
            (s_peer_bonded || pairing_allowed())) {
            s_secreq_sent = true;
            int rc = ble_gap_security_initiate(conn);
            ESP_LOGI(TAG, "телефон шифрование не включил — прошу сам (%d)", rc);
        }
        return;
    }
    static const uint16_t at_ms[] = {500, 1500, 2500};
    if (s_tune_step >= 3 || now - s_secure_ms < at_ms[s_tune_step]) {
        return;
    }
    struct ble_gap_conn_desc d;
    int rc = 0;
    switch (s_tune_step++) {
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

    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status != 0) {
            advertise();
            return 0;
        }
        s_secure = false;
        s_subscribed = false;
        s_pairing = false;
        s_mtu = BLE_ATT_MTU_DFLT;
        s_conn_at = now_s();
        s_conn_ms = now_ms();
        s_secreq_sent = false;
        s_tune_step = 0;
        s_peer_bonded = false;
        s_conn = ev->connect.conn_handle;
        gate();
        if (ble_gap_conn_find(ev->connect.conn_handle, &d) == 0) {
            bool bonded = is_bonded(&d.peer_id_addr);
            s_peer_bonded = bonded;
            ESP_LOGI(TAG, "подключился %02X:%02X:%02X:%02X:%02X:%02X, %s, интервал %d.%02d мс",
                     d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
                     d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0],
                     bonded ? "привязан" : pairing_allowed() ? "новый, окно сопряжения открыто"
                                                             : "новый, окно сопряжения закрыто",
                     d.conn_itvl * 125 / 100, d.conn_itvl * 125 % 100);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "отключился через %lu мс, причина %d (0x%X)%s",
                 (unsigned long)(now_ms() - s_conn_ms), ev->disconnect.reason, ev->disconnect.reason,
                 s_secure ? "" : ", шифрование так и не включилось");
        if (s_pairing) {
            s_pairing = false;
            pair_failed(ev->disconnect.reason);
        }
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_secure = false;
        s_subscribed = false;
        advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        bool was_pairing = s_pairing;
        s_pairing = false;
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
        s_secure_ms = now_ms();
        s_secure = true;
        count_bonds();
        if (was_pairing) {
            events_add('I', "BT_PAIRED", "%02X:%02X:%02X:%02X:%02X:%02X bonds %d",
                       d.peer_id_addr.val[5], d.peer_id_addr.val[4], d.peer_id_addr.val[3],
                       d.peer_id_addr.val[2], d.peer_id_addr.val[1], d.peer_id_addr.val[0],
                       (int)s_bonds);
        } else {
            ESP_LOGI(TAG, "связь зашифрована через %lu мс после соединения",
                     (unsigned long)(s_secure_ms - s_conn_ms));
        }
        return 0;
    }

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (ev->subscribe.attr_handle == s_tx_handle) {
            s_subscribed = ev->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        s_mtu = ev->mtu.value;
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
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    case BLE_GAP_EVENT_PASSKEY_ACTION:
        // Блок «показывает» код (он задан настройкой bt_pin), человек вводит его на телефоне.
        if (ev->passkey.params.action == BLE_SM_IOACT_DISP && pairing_allowed()) {
            struct ble_sm_io io = {
                .action = BLE_SM_IOACT_DISP,
                .passkey = (uint32_t)atoi(cfg_get_str("bt_pin")),
            };
            s_pairing = true;
            ble_sm_inject_io(ev->passkey.conn_handle, &io);
        } else {
            ble_gap_terminate(ev->passkey.conn_handle, BLE_ERR_AUTH_FAIL);
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
    if (!s_secure) {
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }
    if (attr == s_rx_handle) {
        rx_item_t it;
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
            s_ota_cb(msg, len);
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

esp_err_t ble_start(const char *name, bool pair_window)
{
    strlcpy(s_name, name, sizeof(s_name));
    s_lock = xSemaphoreCreateMutex();
    s_rx = xQueueCreate(RX_QUEUE, sizeof(rx_item_t));
    if (!s_lock || !s_rx) {
        return ESP_ERR_NO_MEM;
    }
    if (pair_window) {
        ble_pair_open(BLE_PAIR_WINDOW);
    }
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
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_secure = false;
        s_subscribed = false;
        s_pairing = false;
        s_stopping = false;
    }
    xSemaphoreGive(s_lock);
}

void ble_get_state(ble_state_t *out)
{
    out->up = s_up;
    out->advertising = s_up && ble_gap_adv_active();
    out->connected = s_conn != BLE_HS_CONN_HANDLE_NONE;
    out->secure = s_secure;
    out->pairing = pairing_allowed();
    out->mtu = s_mtu;
    out->bonds = s_bonds;
}

int ble_send(const char *data, int len)
{
    if (!s_lock || !s_secure || !s_subscribed) {
        return -1;
    }
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) {
        return -1;
    }
    int sent = s_up ? 0 : -1;
    // BLE-3: поток байт. Строка, которая влезает в уведомление, не делится.
    int max = s_mtu - 3;
    while (sent >= 0 && sent < len && os_msys_num_free() >= MSYS_RESERVE) {
        int n = len - sent < max ? len - sent : max;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(data + sent, n);
        // Буфер функция освобождает сама, в том числе при ошибке.
        if (!om || ble_gatts_notify_custom(s_conn, s_tx_handle, om) != 0) {
            break;
        }
        sent += n;
    }
    xSemaphoreGive(s_lock);
    return sent;
}

bool ble_recv(char *buf, size_t size)
{
    rx_item_t it;
    if (!s_rx || xQueueReceive(s_rx, &it, 0) != pdTRUE) {
        return false;
    }
    strlcpy(buf, it.data, size);
    return true;
}

void ble_poll(void)
{
    if (!s_lock) {
        return;
    }
    gate();
    uint16_t conn = s_conn;
    uint32_t now = now_s();
    // Окно сопряжения открылось или закрылось — поправить байт состояния в рекламе.
    if (conn == BLE_HS_CONN_HANDLE_NONE && pairing_allowed() != s_adv_open &&
        xSemaphoreTake(s_lock, 0) == pdTRUE) {
        if (s_up) {
            set_adv_data();
        }
        xSemaphoreGive(s_lock);
    }
    if (conn != BLE_HS_CONN_HANDLE_NONE && xSemaphoreTake(s_lock, 0) == pdTRUE) {
        if (s_up) {
            link_steps(conn);
        }
        xSemaphoreGive(s_lock);
    }
    if (conn == BLE_HS_CONN_HANDLE_NONE || s_secure || now - s_conn_at < SECURE_DEADLINE_S) {
        return;
    }
    // Соединение одно: чужой, который подключился и молчит, не должен занимать его вечно.
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) {
        return;
    }
    if (s_up) {
        ESP_LOGW(TAG, "проверка не пройдена за %d с — отключаю", SECURE_DEADLINE_S);
        ble_gap_terminate(conn, BLE_ERR_REM_USER_CONN_TERM);
        s_conn_at = now;
    }
    xSemaphoreGive(s_lock);
}

void ble_set_ota_cb(void (*cb)(const uint8_t *msg, size_t len))
{
    s_ota_cb = cb;
}

void ble_pair_open(int seconds)
{
    s_window_until = now_s() + seconds;
    gate();
    events_add('I', "BT_PAIR_OPEN", "%d s", seconds);
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
        }
        count_bonds();
    }
    xSemaphoreGive(s_lock);
    return n;
}
