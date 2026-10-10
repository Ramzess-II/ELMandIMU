#include "config.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#define TAG "config"
#define VALUE_MAX 64

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
    {"wifi_mode",     "ap",   T_STR,   2, 3,       false, "", 0},
    {"wifi_ssid",     "",     T_STR,   0, 32,      false, "", 0},  // пусто — NoGPS-XXXX
    {"wifi_pass",     NULL,   T_STR,   8, 63,      false, "", 0},
    // Код, который телефон спрашивает при сопряжении по Bluetooth: ровно 6 цифр.
    {"bt_pin",        NULL,   T_STR,   6, 6,       true, "", 0},
    {"imu_driver",    "auto", T_STR,   1, 31,      false, "", 0},
    {"imu_odr",       NULL,   T_INT,   200, 1000,  false, "", 0},
    {"still_acc",     "0.08", T_FLOAT, 0.005, 2,   true, "", 0},
    {"still_gyro",    "0.3",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"still_rate",    "0.5",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"bias_max_jump", "0.5",  T_FLOAT, 0.02, 5,    true, "", 0},
    {"obd_enabled",   "1",    T_INT,   0, 1,       false, "", 0},
    {"obd_baud",      "0",    T_INT,   0, 2000000, false, "", 0},
    {"obd_proto",     "0",    T_INT,   0, 12,      false, "", 0},
    // 1 — плата для проверки на столе: вместо ELM327 скорость по синусу, а обновление прошивки
    // принимается и по Wi-Fi. В машине должно быть 0.
    {"bench",         "0",    T_INT,   0, 1,       false, "", 0},
    {"data_hz",       "50",   T_INT,   10, 100,    true, "", 0},
    // Экономия на стоянке: через сколько секунд после остановки двигателя выключить Wi-Fi; 0 — никогда.
    {"wifi_off_s",    "60",   T_INT,   0, 3600,    true, "", 0},
    // Через сколько секунд после остановки двигателя усыпить ESP32 и гироскоп (блок просыпается раз
    // в 2 с измерить напряжение); 0 — не усыплять.
    {"sleep_after_s", "600",  T_INT,   0, 86400,   true, "", 0},
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
        if (strcmp(it->key, "wifi_mode") == 0) {
            return strcmp(v, "ap") == 0 || strcmp(v, "sta") == 0;
        }
        if (strcmp(it->key, "bt_pin") == 0) {
            return strspn(v, "0123456789") == len;
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

void cfg_init(const char *default_wifi_pass, const char *default_bt_pin, int default_imu_odr)
{
    static char odr[12];
    snprintf(odr, sizeof(odr), "%d", default_imu_odr);
    find("wifi_pass")->def = default_wifi_pass;
    find("bt_pin")->def = default_bt_pin;
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
