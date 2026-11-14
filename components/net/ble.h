// Связь по Bluetooth LE: строки протокола едут уведомлениями GATT.
// Блок — периферийное устройство. Сервис 02CB0001-C0C3-40D0-819C-5A85998DCD99:
//   TX  02CB0002-…  Notify                 строки от блока, поток байт: строка может быть разрезана
//   RX  02CB0003-…  Write / Write No Rsp   команды NGC, по одной строке на запись
//   OTA 02CB0004-…  Write No Rsp           куски прошивки
// Up to BLE_LINKS phones are connected at once (spec BLE-1). Data go only to a paired phone over an encrypted
// link. A new phone pairs only in the pairing window: 2 minutes after a power-on of a box with no phones, or after
// BT_PAIR from a paired phone, with the code BT_PAIR gave.
// В рекламном пакете, кроме UUID сервиса, — данные производителя: компания 0xFFFF, байт версии
// формата (1) и байт флагов: bit 0 — the pairing window is open, bit 1 — the box has no room for a new phone,
// bit 2 — the box has phones: a new one is added from them.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_RX_MAX       256    // наибольшая запись в RX вместе с завершающим нулём
#define BLE_PAIR_WINDOW  120    // окно сопряжения, с
#define BLE_LINKS        CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define BLE_MAX_BONDS    CONFIG_BT_NIMBLE_MAX_BONDS
#define BLE_TO_ALL       0xFFFF // ble_send: to every phone linked
#define BLE_ADDR_LEN     18     // "AA:BB:CC:DD:EE:FF" with the terminating zero

typedef struct {
    bool up;          // стек запущен
    bool advertising; // реклама идёт на самом деле (по данным стека)
    bool connected;   // someone is connected
    bool secure;      // a paired phone is connected over an encrypted link: data go
    bool pairing;     // окно сопряжения открыто
    int  bonds;       // сколько телефонов привязано
} ble_state_t;

// name — имя в рекламе (NoGPS-XXXX). pair_window — the box was powered on: open the pairing window if it has no
// phones. reset — erase all phones first (BLE-16).
esp_err_t ble_start(const char *name, bool pair_window, bool reset);

// Стоянка: выключить радио совсем (реклама и соединение) или включить обратно.
void ble_set_enabled(bool on);
void ble_get_state(ble_state_t *out);

// Sends a line by TX notifications to the link |to| or to all of them (BLE_TO_ALL). Returns len if it went, 0 if the
// queue of the stack has no room for it (nothing went), -1 if there is nobody to send it to.
int ble_send(uint16_t to, const char *data, int len);

// The next write to RX with a terminating zero, and the link it came from. false — пусто.
bool ble_recv(char *buf, size_t size, uint16_t *from);

// Звать часто (десятки миллисекунд): следит за окном сопряжения и рвёт связь с тем, кто подключился
// и не прошёл проверку.
void ble_poll(void);

// The link is connected and encrypted.
bool ble_link_up(uint16_t conn);
// The identity address of the phone of the link. false if there is no such link.
bool ble_link_addr(uint16_t conn, char addr[BLE_ADDR_LEN]);
// The payload of one notification or write of the link.
int ble_link_mtu(uint16_t conn);

// Куда отдавать записи в характеристику OTA: link, смещение (4 байта) и данные. Зовётся из задачи стека.
void ble_set_ota_cb(void (*cb)(uint16_t conn, const uint8_t *msg, size_t len));

// Opens the pairing window with a code of 6 digits that works for one phone, or with the bt_pin setting if |code| is
// NULL.
void ble_pair_open(int seconds, const char *code);
// The addresses of the paired phones. Returns how many, -1 on error.
int  ble_bonded(char addrs[][BLE_ADDR_LEN], int max);
// Forgets one phone and drops its links but |keep|: 0, 1 if the box doesn't know it, -1 on error.
int  ble_forget_one(const char *addr, uint16_t keep);
// Drops the link.
void ble_drop(uint16_t conn);
// Забыть все привязанные телефоны. Возвращает, сколько их было, -1 при ошибке.
int  ble_forget(void);

#ifdef __cplusplus
}
#endif
