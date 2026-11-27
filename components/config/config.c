#include "config.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_random.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

#define TAG "config"
#define VALUE_MAX 64
#define KEY_POWER_ONS "power_ons"
#define KEY_QUICK_ONS "quick_ons"
#define KEY_OLD_RECOVERY_CODE "rec_code"   // 0.5.x: the random recovery code, gone in 0.6.0

static uint32_t s_power_ons;
static uint32_t s_quick_ons;

typedef enum { T_STR, T_INT, T_FLOAT } type_t;

typedef struct {
    const char *key;
    const char *def;
    type_t      type;
    float       min, max;     // для чисел — диапазон, для строк — длина
    bool        now;          // действует сразу
    char        str[VALUE_MAX];
    float       num;
} item_t;

static item_t s_items[] = {
    {"imu_driver",    "auto", T_STR,   1, 31,      false, "", 0},
    {"imu_odr",       NULL,   T_INT,   200, 1000,  false, "", 0},
    {"still_acc",     "0.08", T_FLOAT, 0.005, 2,   true, "", 0},
    {"still_gyro",    "0.3",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"still_rate",    "0.5",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"bias_max_jump", "0.5",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"obd_enabled",   "1",    T_INT,   0, 1,       false, "", 0},
    {"obd_baud",      "0",    T_INT,   0, 2000000, false, "", 0},
    {"obd_proto",     "0",    T_INT,   0, 12,      false, "", 0},
    // 1 — плата для проверки на столе: вместо ELM327 скорость по синусу. В машине должно быть 0.
    {"bench",         "0",    T_INT,   0, 1,       false, "", 0},
    {"data_hz",       "50",   T_INT,   10, 100,    true, "", 0},
    // Экономия на стоянке: через сколько секунд после остановки двигателя выключить радио; 0 — никогда.
    // Имя осталось от Wi-Fi: на нём написаны журналы и разборщики.
    {"wifi_off_s",    "60",   T_INT,   0, 3600,    true, "", 0},
    // Через сколько секунд после остановки двигателя усыпить ESP32 и гироскоп (блок просыпается раз
    // в 2 с измерить напряжение); 0 — не усыплять.
    // Above 3600 means nothing: the box sleeps an hour after the engine stops whatever this says.
    {"sleep_after_s", "600",  T_INT,   0, 3600,    true, "", 0},
};
#define N_ITEMS (sizeof(s_items) / sizeof(s_items[0]))

static item_t *find(const char *key)
{
    for (size_t i = 0; i < N_ITEMS; i++) {
        if (strcmp(s_items[i].key, key) == 0) {
            return &s_items[i];
        }
    }
    return NULL;
}

static bool valid(const item_t *it, const char *v, float *num)
{
    size_t len = strlen(v);
    if (len >= VALUE_MAX) {
        return false;
    }
    if (it->type == T_STR) {
        if (len < it->min || len > it->max) {
            return false;
        }
        return true;
    }
    char *end;
    float x = it->type == T_INT ? (float)strtol(v, &end, 10) : strtof(v, &end);
    if (len == 0 || *end || x < it->min || x > it->max) {
        return false;
    }
    *num = x;
    return true;
}

static void apply(item_t *it, const char *v, float num)
{
    strlcpy(it->str, v, sizeof(it->str));
    it->num = num;
}

// Одна запись во флеш на каждое включение питанием. Блок в разъёме OBD под питанием постоянно, так что
// это редкость: вынули и вставили.
static void count_power_on(bool power_on)
{
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, power_on ? NVS_READWRITE : NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    nvs_get_u32(h, KEY_POWER_ONS, &s_power_ons);
    nvs_get_u32(h, KEY_QUICK_ONS, &s_quick_ons);
    if (power_on) {
        s_power_ons++;
        s_quick_ons++;
        if (nvs_set_u32(h, KEY_POWER_ONS, s_power_ons) != ESP_OK ||
            nvs_set_u32(h, KEY_QUICK_ONS, s_quick_ons) != ESP_OK || nvs_commit(h) != ESP_OK) {
            ESP_LOGE(TAG, "счётчик включений не записан");
        }
    }
    nvs_close(h);
}

uint32_t cfg_power_ons(void)
{
    return s_power_ons;
}

uint32_t cfg_quick_power_ons(void)
{
    return s_quick_ons;
}

void cfg_quick_power_ons_clear(void)
{
    if (s_quick_ons == 0) {
        return;
    }
    s_quick_ons = 0;
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_set_u32(h, KEY_QUICK_ONS, 0) != ESP_OK || nvs_commit(h) != ESP_OK) {
        ESP_LOGE(TAG, "quick power-ons: NVS write failed");
    }
    nvs_close(h);
}

// The first boot of 0.6.0 erases the recovery code of 0.5.x: nothing reads it, and it should not stay in the flash.
static void erase_old_recovery_code(void)
{
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (nvs_erase_key(h, KEY_OLD_RECOVERY_CODE) == ESP_OK) {
        nvs_commit(h);
    }
    nvs_close(h);
}

void cfg_init(int default_imu_odr, bool power_on)
{
    erase_old_recovery_code();
    count_power_on(power_on);
    static char odr[12];
    snprintf(odr, sizeof(odr), "%d", default_imu_odr);
    find("imu_odr")->def = odr;

    nvs_handle_t h;
    bool have_nvs = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK;
    for (size_t i = 0; i < N_ITEMS; i++) {
        item_t *it = &s_items[i];
        float num = 0;
        valid(it, it->def, &num);
        apply(it, it->def, num);
        if (!have_nvs) {
            continue;
        }
        char v[VALUE_MAX];
        size_t len = sizeof(v);
        if (nvs_get_str(h, it->key, v, &len) == ESP_OK) {
            if (valid(it, v, &num)) {
                apply(it, v, num);
            } else {
                ESP_LOGW(TAG, "%s: неверное значение в NVS, беру %s", it->key, it->def);
            }
        }
    }
    if (have_nvs) {
        nvs_close(h);
    }
}

const char *cfg_get_str(const char *key)
{
    item_t *it = find(key);
    return it ? it->str : "";
}

int cfg_get_int(const char *key)
{
    item_t *it = find(key);
    return it ? (int)it->num : 0;
}

float cfg_get_float(const char *key)
{
    item_t *it = find(key);
    return it ? it->num : 0;
}

const char *cfg_get(const char *key)
{
    item_t *it = find(key);
    return it ? it->str : NULL;
}

static bool is_default(const item_t *it)
{
    return it->type == T_STR ? strcmp(it->str, it->def) == 0 : it->num == strtof(it->def, NULL);
}

// ";+<N>" for N up to 99.
#define LEFT_OUT_MAX 4
_Static_assert(N_ITEMS <= 99, "LEFT_OUT_MAX holds two digits");

void cfg_get_changed(char *out, size_t size)
{
    size_t cap = size - 1, len = 0;
    int left = 0;
    for (size_t i = 0; i < N_ITEMS; i++) {
        const item_t *it = &s_items[i];
        if (is_default(it)) {
            continue;
        }
        if (left == 0) {
            size_t more = 0;
            for (size_t j = i + 1; j < N_ITEMS; j++) {
                more += !is_default(&s_items[j]);
            }
            size_t need = (len ? 1 : 0) + strlen(it->key) + 1 + strlen(it->str);
            if (len + need + (more ? LEFT_OUT_MAX : 0) <= cap) {
                len += snprintf(out + len, size - len, "%s%s=%s", len ? ";" : "", it->key, it->str);
                continue;
            }
        }
        left++;     // from the first key that does not fit, the rest are left out too
    }
    out[len] = 0;
    if (left) {
        snprintf(out + len, size - len, "%s+%d", len ? ";" : "", left);
    }
}

esp_err_t cfg_set(const char *key, const char *value)
{
    item_t *it = find(key);
    if (!it) {
        return ESP_ERR_NOT_FOUND;
    }
    float num = 0;
    if (!valid(it, value, &num)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_str(h, key, value);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: запись в NVS: %s", key, esp_err_to_name(err));
        return err;
    }
    if (it->now) {
        apply(it, value, num);
    }
    ESP_LOGI(TAG, "%s = %s%s", key, value, it->now ? "" : " (после перезагрузки)");
    return ESP_OK;
}

// ---- values of the car ----

// An NVS key: 1-15 characters.
static bool car_key_ok(const char *key)
{
    size_t len = strlen(key);
    return len >= 1 && len <= 15 && strspn(key, "abcdefghijklmnopqrstuvwxyz0123456789_") == len;
}

// Printable ASCII without the separators of a line.
static bool car_value_ok(const char *v)
{
    if (strlen(v) > CAR_VALUE_MAX) {
        return false;
    }
    for (const char *p = v; *p; p++) {
        unsigned char c = (unsigned char)*p;
        if (c < 0x20 || c > 0x7E || c == ',' || c == '*' || c == '$') {
            return false;
        }
    }
    return true;
}

static int car_count(void)
{
    int n = 0;
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, CAR_NVS_NAMESPACE, NVS_TYPE_STR, &it);
    while (err == ESP_OK) {
        n++;
        err = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);
    return n;
}

const char *cfg_car_get(const char *key, char *out, size_t size)
{
    out[0] = '\0';
    if (!car_key_ok(key)) {
        return "KEY";
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(CAR_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return NULL;    // nothing written yet
    }
    if (err != ESP_OK) {
        return "NVS";
    }
    size_t len = size;
    err = nvs_get_str(h, key, out, &len);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        out[0] = '\0';
        return NULL;
    }
    return err == ESP_OK ? NULL : "NVS";
}

const char *cfg_car_set(const char *key, const char *value)
{
    if (!car_key_ok(key)) {
        return "KEY";
    }
    if (!car_value_ok(value)) {
        return "VALUE";
    }
    // The app writes every few minutes while the car drives: the same value is not written to the flash again.
    char old[CAR_VALUE_MAX + 1];
    if (cfg_car_get(key, old, sizeof(old))) {
        return "NVS";
    }
    if (strcmp(old, value) == 0) {
        ESP_LOGI(TAG, "car %s: unchanged", key);
        return NULL;
    }
    if (!old[0] && car_count() >= CAR_KEYS_MAX) {
        return "FULL";
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(CAR_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = value[0] ? nvs_set_str(h, key, value) : nvs_erase_key(h, key);
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "car %s: NVS write: %s", key, esp_err_to_name(err));
        return "NVS";
    }
    // The place of the car is not logged.
    ESP_LOGI(TAG, "car %s: %u bytes", key, (unsigned)strlen(value));
    return NULL;
}

// ---- phones ----

// The NVS key of a phone: its address without the colons, 12 characters.
static bool phone_key(const char *addr, char key[13])
{
    int n = 0;
    for (const char *p = addr; *p && n < 12; p++) {
        if (*p != ':') {
            key[n++] = *p;
        }
    }
    key[n] = '\0';
    return n == 12;
}

bool cfg_phone_get(const char *addr, char *name, size_t size, uint32_t *seen)
{
    name[0] = '\0';
    *seen = 0;
    char key[13];
    nvs_handle_t h;
    if (!phone_key(addr, key) || nvs_open(PHONE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    // "<seen>|<name>"
    char v[PHONE_NAME_MAX + 16];
    size_t len = sizeof(v);
    esp_err_t err = nvs_get_str(h, key, v, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return false;
    }
    char *bar = strchr(v, '|');
    *seen = strtoul(v, NULL, 10);
    strlcpy(name, bar ? bar + 1 : "", size);
    return true;
}

void cfg_phone_set(const char *addr, const char *name, uint32_t seen)
{
    char key[13];
    nvs_handle_t h;
    if (!phone_key(addr, key) || nvs_open(PHONE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    char v[PHONE_NAME_MAX + 16];
    snprintf(v, sizeof(v), "%lu|%s", (unsigned long)seen, name);
    if (nvs_set_str(h, key, v) != ESP_OK || nvs_commit(h) != ESP_OK) {
        ESP_LOGE(TAG, "phone %s: NVS write failed", key);
    }
    nvs_close(h);
}

// The key of the phone is a blob under its name with "k" after it.
static bool phone_auth_key(const char *addr, char key[14])
{
    if (!phone_key(addr, key)) {
        return false;
    }
    strcat(key, "k");
    return true;
}

bool cfg_phone_key_get(const char *addr, uint8_t key[PHONE_AUTH_KEY_LEN])
{
    char name[14];
    nvs_handle_t h;
    if (!phone_auth_key(addr, name) || nvs_open(PHONE_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    size_t len = PHONE_AUTH_KEY_LEN;
    esp_err_t err = nvs_get_blob(h, name, key, &len);
    nvs_close(h);
    return err == ESP_OK && len == PHONE_AUTH_KEY_LEN;
}

bool cfg_phone_key_set(const char *addr, const uint8_t key[PHONE_AUTH_KEY_LEN])
{
    char name[14];
    nvs_handle_t h;
    if (!phone_auth_key(addr, name) || nvs_open(PHONE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(h, name, key, PHONE_AUTH_KEY_LEN);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "phone %s: key NVS write: %s", name, esp_err_to_name(err));
    }
    return err == ESP_OK;
}

void cfg_phone_erase(const char *addr)
{
    char key[13], auth[14];
    nvs_handle_t h;
    if ((addr && (!phone_key(addr, key) || !phone_auth_key(addr, auth))) ||
        nvs_open(PHONE_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (addr) {
        nvs_erase_key(h, key);
        nvs_erase_key(h, auth);
    } else {
        nvs_erase_all(h);
    }
    nvs_commit(h);
    nvs_close(h);
}

// ---- factory reset ----

#define FACTORY_MAGIC 0x4E474652u   // "NGFR"
// RTC memory is kept over a software reset, not over a power-on.
static RTC_NOINIT_ATTR uint32_t s_factory_magic;

void cfg_factory_reset(void)
{
    nvs_flash_deinit();
    esp_err_t err = nvs_flash_erase();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "factory reset: NVS erase: %s", esp_err_to_name(err));
    }
    s_factory_magic = FACTORY_MAGIC;
}

bool cfg_after_factory_reset(void)
{
    static int after = -1;
    if (after < 0) {
        after = esp_reset_reason() == ESP_RST_SW && s_factory_magic == FACTORY_MAGIC;
        s_factory_magic = 0;
    }
    return after;
}
