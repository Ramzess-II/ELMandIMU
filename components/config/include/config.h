// Настройки в NVS, пространство имён "nogps" (ТЗ, раздел 12). Функции начинаются с cfg_, а не с
// config_: имя config_get_int занято в библиотеке Bluetooth из ESP-IDF.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_NVS_NAMESPACE "nogps"

// Прочитать настройки из NVS. NVS уже инициализирован.
// Значения по умолчанию для кода сопряжения и частоты IMU берутся из сборки.
// power_on — блок только что включили питанием: счётчик включений увеличивается и пишется в NVS.
void cfg_init(const char *default_bt_pin, int default_imu_odr, bool power_on);

// Сколько раз блок включали питанием (вынимали из разъёма и вставляли). Перезагрузки не считаются.
// По этому числу приложение узнаёт, что блок могли переставить и нужна калибровка.
uint32_t cfg_power_ons(void);

// Текущие значения. Настройки «сразу» меняются командой SET на ходу, остальные — после перезагрузки.
const char *cfg_get_str(const char *key);
int         cfg_get_int(const char *key);
float       cfg_get_float(const char *key);

// SET: ESP_ERR_NOT_FOUND — нет такого ключа, ESP_ERR_INVALID_ARG — неверное значение.
esp_err_t cfg_set(const char *key, const char *value);

// Values of the car the app keeps on the box for every phone in the car (spec 9.5, 12): opaque strings in their own
// NVS namespace. CAL_RESET and BT_FORGET leave them, a firmware update keeps them.
#define CAR_NVS_NAMESPACE "nogps_car"
#define CAR_KEYS_MAX      16
#define CAR_VALUE_MAX     200

// CAR_GET: |out| gets the value, empty if the box has none. Returns NULL or the error of the reply: "KEY", "NVS".
const char *cfg_car_get(const char *key, char *out, size_t size);
// CAR_SET: writes the value only if it differs, an empty one erases the key.
// Returns NULL or the error of the reply: "KEY", "VALUE", "FULL", "NVS".
const char *cfg_car_set(const char *key, const char *value);

#ifdef __cplusplus
}
#endif
