#include "update.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"
#include "nvs.h"

#include "config.h"
#include "events.h"

#define TAG "update"

#define CONFIRM_US      600000000LL     // 10 минут на подтверждение
#define NVS_KEY         "ota_imu"       // был ли гироскоп найден перед обновлением
#define DATA_TIMEOUT_US 30000000LL      // нет кусков 30 с — приём отменяется
#define MSG_MAX         (4 + UPDATE_CHUNK_MAX)
// Очередь кусков между задачей Bluetooth и записью во флеш. Окно телефона — 4096 байт; даже кусками
// по 16 байт (MTU 23) вместе со служебными полями это меньше 8 КБ.
#define QUEUE_BYTES     8192

static const esp_partition_t *s_running;
static const char *s_state = "UNDEFINED";
static volatile bool s_pending;
static bool s_need_imu;

// ---- подтверждение и откат ----

void update_init(void)
{
    s_running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (s_running && esp_ota_get_state_partition(s_running, &st) == ESP_OK) {
        s_pending = st == ESP_OTA_IMG_PENDING_VERIFY;
        s_state = s_pending ? "PENDING" : st == ESP_OTA_IMG_VALID ? "VALID" : "UNDEFINED";
    }
    const esp_app_desc_t *d = esp_app_get_description();
    ESP_LOGI(TAG, "прошивка %s, раздел %s, состояние %s", d->version,
             s_running ? s_running->label : "?", s_state);
    if (!s_pending) {
        return;
    }
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 0;
        s_need_imu = nvs_get_u8(h, NVS_KEY, &v) == ESP_OK && v;
        nvs_close(h);
    }
    events_add('W', "FW_PENDING", "%s %s", d->version, s_need_imu ? "phone+imu" : "phone");
}

bool update_supported(void)
{
#if CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    return next && next != s_running;
#else
    // Без проверки подписи прошивку по радио не принимаем вовсе.
    return false;
#endif
}

void update_get_info(update_info_t *out)
{
    memset(out, 0, sizeof(*out));
    const esp_app_desc_t *d = esp_app_get_description();
    out->version = d->version;
    // "Oct  8 2026" и "20:15:03" -> 2026-10-08T20:15
    static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mon[4] = "";
    int day = 0, year = 0;
    if (sscanf(d->date, "%3s %d %d", mon, &day, &year) == 3) {
        const char *p = strstr(months, mon);
        snprintf(out->build, sizeof(out->build), "%04d-%02d-%02dT%.5s", year,
                 p ? (int)(p - months) / 3 + 1 : 0, day, d->time);
    }
    strlcpy(out->slot, s_running ? s_running->label : "", sizeof(out->slot));
    out->slot_state = s_state;
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    out->max_image = next && next != s_running ? next->size : 0;
    out->can_update = update_supported();
}

void update_tick(bool phone, bool imu_ok)
{
    if (!s_pending) {
        return;
    }
    if (phone && (imu_ok || !s_need_imu)) {
        esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "подтверждение прошивки: %s", esp_err_to_name(err));
            return;
        }
        s_state = "VALID";
        s_pending = false;
        events_add('I', "FW_CONFIRMED", "%s", esp_app_get_description()->version);
    } else if (esp_timer_get_time() > CONFIRM_US) {
        events_add('E', "FW_ROLLBACK", "%s", !phone ? "no phone" : "no imu");
        vTaskDelay(pdMS_TO_TICKS(300));
        // Возвращается, только если откатываться некуда.
        esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
        ESP_LOGE(TAG, "откат не удался: %s", esp_err_to_name(err));
        s_pending = false;
    }
}

bool update_hold_awake(void)
{
    return s_pending || update_active();
}

// ---- приём новой прошивки ----

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static update_progress_t s_prog;        // под s_mux
static MessageBufferHandle_t s_queue;   // создаётся при первом обновлении и остаётся
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_target;
static uint8_t s_sha_want[32];
static char s_version[32];
static volatile bool s_end_req, s_abort_req;
static char s_abort_err[12];
static bool s_imu_ok;

static void set_state(update_state_t st, const char *err)
{
    taskENTER_CRITICAL(&s_mux);
    s_prog.state = st;
    strlcpy(s_prog.err, err ? err : "", sizeof(s_prog.err));
    taskEXIT_CRITICAL(&s_mux);
}

static update_state_t get_state(void)
{
    taskENTER_CRITICAL(&s_mux);
    update_state_t st = s_prog.state;
    taskEXIT_CRITICAL(&s_mux);
    return st;
}

static bool parse_hex(const char *hex, uint8_t *out, size_t n)
{
    if (strlen(hex) != n * 2) {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        unsigned v;
        char pair[3] = {hex[2 * i], hex[2 * i + 1], 0};
        if (strspn(pair, "0123456789abcdefABCDEF") != 2 || sscanf(pair, "%x", &v) != 1) {
            return false;
        }
        out[i] = (uint8_t)v;
    }
    return true;
}

// Начало образа: файл должен быть прошивкой для этого чипа. Остальное (целостность, подпись) проверит
// esp_ota_end, но уже после того, как всё передано.
static bool header_ok(const uint8_t *data, size_t len)
{
    if (len < sizeof(esp_image_header_t)) {
        return data[0] == ESP_IMAGE_HEADER_MAGIC;
    }
    const esp_image_header_t *h = (const esp_image_header_t *)data;
    return h->magic == ESP_IMAGE_HEADER_MAGIC && h->chip_id == CONFIG_IDF_FIRMWARE_CHIP_ID;
}

static void save_imu_flag(bool imu_ok)
{
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY, imu_ok ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}

// Пишет куски во флеш. Стирание идёт по мере записи, поэтому до конца приёма во втором разделе
// лежит неполный образ, а работающая прошивка не тронута.
static void ota_task(void *arg)
{
    static uint8_t msg[MSG_MAX];
    mbedtls_sha256_context sha;
    uint32_t received = 0, size = s_prog.size;
    const char *err = NULL;
    bool done = false, aborted = false;
    int64_t last_data = esp_timer_get_time();

    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts(&sha, 0);

    while (!err && !done && !aborted) {
        size_t n = xMessageBufferReceive(s_queue, msg, sizeof(msg), pdMS_TO_TICKS(50));
        int64_t now = esp_timer_get_time();
        if (s_abort_req) {
            aborted = true;
            err = s_abort_err[0] ? s_abort_err : NULL;
            break;
        }
        if (n > 4) {
            uint32_t off = msg[0] | msg[1] << 8 | msg[2] << 16 | (uint32_t)msg[3] << 24;
            const uint8_t *data = msg + 4;
            size_t len = n - 4;
            if (off > received) {
                // Кусок дальше ожидаемого места — что-то потерялось: пропускаем и просим телефон
                // вернуться.
                taskENTER_CRITICAL(&s_mux);
                s_prog.resend = true;
                taskEXIT_CRITICAL(&s_mux);
                continue;
            }
            if (off + len <= received) {
                continue;       // повтор уже принятого
            }
            // Повтор, который захватывает и новое: берём только новую часть.
            data += received - off;
            len -= received - off;
            if (received + len > size) {
                err = "SIZE";
            } else if (received == 0 && !header_ok(data, len)) {
                err = "CHIP";
            } else if (esp_ota_write(s_handle, data, len) != ESP_OK) {
                err = "WRITE";
            } else {
                mbedtls_sha256_update(&sha, data, len);
                received += len;
                last_data = now;
                taskENTER_CRITICAL(&s_mux);
                s_prog.received = received;
                taskEXIT_CRITICAL(&s_mux);
            }
            continue;
        }
        // Очередь пуста.
        if (s_end_req) {
            if (received != size) {
                err = "SIZE";
                break;
            }
            set_state(UPDATE_VERIFY, NULL);
            uint8_t got[32];
            mbedtls_sha256_finish(&sha, got);
            if (memcmp(got, s_sha_want, sizeof(got)) != 0) {
                err = "HASH";
                break;
            }
            // Целостность образа и подпись: тем же ключом, что и у работающей прошивки.
            esp_err_t e = esp_ota_end(s_handle);
            s_handle = 0;
            if (e == ESP_OK) {
                e = esp_ota_set_boot_partition(s_target);
            }
            if (e != ESP_OK) {
                ESP_LOGE(TAG, "проверка образа: %s", esp_err_to_name(e));
                err = e == ESP_ERR_OTA_VALIDATE_FAILED ? "SIGNATURE" : "WRITE";
                break;
            }
            save_imu_flag(s_imu_ok);
            done = true;
        } else if (now - last_data > DATA_TIMEOUT_US) {
            err = "TIMEOUT";
        }
    }

    mbedtls_sha256_free(&sha);
    if (s_handle) {
        esp_ota_abort(s_handle);
        s_handle = 0;
    }
    if (done) {
        events_add('I', "OTA_DONE", "%s %lu bytes", s_version, (unsigned long)received);
        set_state(UPDATE_DONE, NULL);
    } else if (err) {
        events_add('W', "OTA_FAIL", "%s at %lu", err, (unsigned long)received);
        set_state(UPDATE_ERROR, err);
    } else {
        events_add('I', "OTA_ABORTED", "at %lu", (unsigned long)received);
        set_state(UPDATE_ABORTED, NULL);
    }
    vTaskDelete(NULL);
}

const char *update_begin(uint32_t size, const char *sha256_hex, const char *version)
{
    if (!update_supported()) {
        return "UNSUPPORTED";
    }
    if (update_active()) {
        return "BUSY";
    }
    s_target = esp_ota_get_next_update_partition(NULL);
    if (size < sizeof(esp_image_header_t) || size > s_target->size) {
        return "SIZE";
    }
    if (!parse_hex(sha256_hex, s_sha_want, sizeof(s_sha_want))) {
        return "VALUE";
    }
    if (!s_queue) {
        s_queue = xMessageBufferCreate(QUEUE_BYTES);
        if (!s_queue) {
            return "WRITE";
        }
    }
    xMessageBufferReset(s_queue);
    // Стирать раздел заранее не нужно: он стирается по мере записи.
    esp_err_t e = esp_ota_begin(s_target, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin: %s", esp_err_to_name(e));
        // Работающая прошивка сама ещё не подтверждена: сначала она должна дождаться телефона.
        return e == ESP_ERR_OTA_ROLLBACK_INVALID_STATE ? "PENDING" : "WRITE";
    }
    strlcpy(s_version, version, sizeof(s_version));
    s_end_req = s_abort_req = false;
    taskENTER_CRITICAL(&s_mux);
    memset(&s_prog, 0, sizeof(s_prog));
    s_prog.size = size;
    s_prog.state = UPDATE_RECV;
    taskEXIT_CRITICAL(&s_mux);
    if (xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL) != pdPASS) {
        esp_ota_abort(s_handle);
        s_handle = 0;
        set_state(UPDATE_IDLE, NULL);
        return "WRITE";
    }
    events_add('I', "OTA_BEGIN", "%s %lu bytes to %s", s_version, (unsigned long)size, s_target->label);
    return NULL;
}

void update_chunk(const uint8_t *msg, size_t len)
{
    // Не влезло в очередь — кусок пропадает; ota_task увидит разрыв и попросит повторить.
    if (len > 4 && len <= MSG_MAX && s_queue && get_state() == UPDATE_RECV) {
        xMessageBufferSend(s_queue, msg, len, 0);
    }
}

const char *update_end(bool imu_ok)
{
    if (get_state() != UPDATE_RECV) {
        return "STATE";
    }
    s_imu_ok = imu_ok;
    s_end_req = true;
    return NULL;
}

void update_abort(const char *err)
{
    if (!update_active()) {
        return;
    }
    strlcpy(s_abort_err, err ? err : "", sizeof(s_abort_err));
    s_abort_req = true;
}

void update_get_progress(update_progress_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_prog;
    s_prog.resend = false;
    taskEXIT_CRITICAL(&s_mux);
}

bool update_active(void)
{
    update_state_t st = get_state();
    return st == UPDATE_RECV || st == UPDATE_VERIFY;
}
