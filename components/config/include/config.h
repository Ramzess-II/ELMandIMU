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
// Значение по умолчанию частоты IMU берётся из сборки.
// power_on — блок только что включили питанием: счётчик включений увеличивается и пишется в NVS.
void cfg_init(int default_imu_odr, bool power_on);

// Сколько раз блок включали питанием (вынимали из разъёма и вставляли). Перезагрузки не считаются.
// По этому числу приложение узнаёт, что блок могли переставить и нужна калибровка.
uint32_t cfg_power_ons(void);

// Power-ons in a row, each ended before a minute passed and before a phone linked. The third one opens the recovery
// window (spec BLE-16): the way back for an owner who has lost all phones; a phone that pairs in it erases nobody.
#define QUICK_POWER_ONS_RESET 3
uint32_t cfg_quick_power_ons(void);
// The box has run a minute or a phone has linked: this power-on is not a quick one.
void cfg_quick_power_ons_clear(void);

// FACTORY_RESET: erases the whole NVS partition (settings, phones, bonds, car values, counters). The firmware, the OTA
// slots and the eFuse stay. The caller reboots; that boot counts as a power-on (cfg_after_factory_reset).
void cfg_factory_reset(void);
// True on the boot that follows cfg_factory_reset.
bool cfg_after_factory_reset(void);

// Текущие значения. Настройки «сразу» меняются командой SET на ходу, остальные — после перезагрузки.
const char *cfg_get_str(const char *key);
int         cfg_get_int(const char *key);
float       cfg_get_float(const char *key);

// GET,<key>: the value the box works with, as text; NULL if there is no such key. A setting that is not "now"
// shows the old value until the reboot.
const char *cfg_get(const char *key);
// Bare GET: "key=value;..." of the settings that differ from their default, in the order of the table, written into
// |out| of |size| bytes. What does not fit is replaced by ";+<N>" for the N keys left out. Empty if all are default.
void cfg_get_changed(char *out, size_t size);

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

// Phones paired with the box (spec 9.5, BT_LIST): the name each one sends in HELLO and the Unix time it gave when it
// connected, by its Bluetooth identity address "AA:BB:CC:DD:EE:FF", in their own NVS namespace.
#define PHONE_NVS_NAMESPACE "nogps_ph"
#define PHONE_NAME_MAX      20

// False if the box has no record of the phone. |seen| is 0 if the phone has not told the time.
bool cfg_phone_get(const char *addr, char *name, size_t size, uint32_t *seen);
void cfg_phone_set(const char *addr, const char *name, uint32_t seen);
// NULL erases the records of all phones, with their keys.
void cfg_phone_erase(const char *addr);

// The key the phone gave the box to have it prove itself (spec 9.5, AUTH_SET): 16 bytes kept with the record of the
// phone, so it goes away with the bond.
#define PHONE_AUTH_KEY_LEN 16
// False if the phone has no key.
bool cfg_phone_key_get(const char *addr, uint8_t key[PHONE_AUTH_KEY_LEN]);
// True if the key is written. A phone gives a key once: the caller checks cfg_phone_key_get first.
bool cfg_phone_key_set(const char *addr, const uint8_t key[PHONE_AUTH_KEY_LEN]);

#ifdef __cplusplus
}
#endif
