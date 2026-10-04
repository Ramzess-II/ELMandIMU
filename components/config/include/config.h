// Настройки в NVS, пространство имён "nogps" (ТЗ, раздел 12).
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_NVS_NAMESPACE "nogps"

// Прочитать настройки из NVS. NVS уже инициализирован.
// Значения по умолчанию для пароля и частоты IMU берутся из сборки.
void config_init(const char *default_wifi_pass, int default_imu_odr);

// Текущие значения. Настройки «сразу» меняются командой SET на ходу, остальные — после перезагрузки.
const char *config_get_str(const char *key);
int         config_get_int(const char *key);
float       config_get_float(const char *key);

// SET: ESP_ERR_NOT_FOUND — нет такого ключа, ESP_ERR_INVALID_ARG — неверное значение.
esp_err_t config_set(const char *key, const char *value);

#ifdef __cplusplus
}
#endif
