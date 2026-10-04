#include "calib.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#include "config.h"

#define TAG "calib"
#define KEY "calib"
#define VERSION 1

typedef struct {
    uint32_t     version;
    calib_data_t data;
} blob_t;

void calib_load(calib_data_t *out)
{
    memset(out, 0, sizeof(*out));
    nvs_handle_t h;
    if (nvs_open(CONFIG_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    blob_t b;
    size_t len = sizeof(b);
    if (nvs_get_blob(h, KEY, &b, &len) == ESP_OK && len == sizeof(b) && b.version == VERSION) {
        *out = b.data;
    }
    nvs_close(h);
}

esp_err_t calib_save(const calib_data_t *in)
{
    blob_t b = {.version = VERSION, .data = *in};
    nvs_handle_t h;
    esp_err_t err = nvs_open(CONFIG_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, KEY, &b, sizeof(b));
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "запись: %s", esp_err_to_name(err));
    }
    return err;
}
