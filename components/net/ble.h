// Связь по Bluetooth LE: те же строки протокола, что по UDP, только едут уведомлениями GATT.
// Блок — периферийное устройство с одним соединением. Сервис 02CB0001-C0C3-40D0-819C-5A85998DCD99:
//   TX  02CB0002-…  Notify                 строки от блока, поток байт: строка может быть разрезана
//   RX  02CB0003-…  Write / Write No Rsp   команды NGC, по одной строке на запись
//   OTA 02CB0004-…  Write No Rsp           куски прошивки
// Данные идут только привязанному телефону по шифрованной связи. Привязать новый телефон можно
// только в «окне сопряжения»: первые 2 минуты после подачи питания или после команды BT_PAIR.
// В рекламном пакете, кроме UUID сервиса, — данные производителя: компания 0xFFFF, байт версии
// формата (1) и байт флагов, бит 0 — «окно сопряжения открыто».
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_RX_MAX       256    // наибольшая запись в RX вместе с завершающим нулём
#define BLE_PAIR_WINDOW  120    // окно сопряжения, с

typedef struct {
    bool up;          // стек запущен
    bool advertising; // реклама идёт на самом деле (по данным стека)
    bool connected;
    bool secure;      // связь зашифрована, телефон привязан — данные идут
    bool pairing;     // окно сопряжения открыто
    int  mtu;
    int  bonds;       // сколько телефонов привязано
} ble_state_t;

// name — имя в рекламе (NoGPS-XXXX). pair_window — сразу открыть окно сопряжения.
esp_err_t ble_start(const char *name, bool pair_window);

// Стоянка: выключить радио совсем (реклама и соединение) или включить обратно.
void ble_set_enabled(bool on);
void ble_get_state(ble_state_t *out);

// Послать байты уведомлениями TX. Возвращает, сколько байт ушло: меньше len — очередь стека
// заполнена, остаток надо послать позже. -1 — слать некому.
int ble_send(const char *data, int len);

// Очередная запись в RX, с завершающим нулём. false — пусто.
bool ble_recv(char *buf, size_t size);

// Звать часто (десятки миллисекунд): следит за окном сопряжения и рвёт связь с тем, кто подключился
// и не прошёл проверку.
void ble_poll(void);

// Куда отдавать записи в характеристику OTA: смещение (4 байта) и данные. Зовётся из задачи стека.
void ble_set_ota_cb(void (*cb)(const uint8_t *msg, size_t len));

void ble_pair_open(int seconds);
// Забыть все привязанные телефоны. Возвращает, сколько их было, -1 при ошибке.
int  ble_forget(void);

#ifdef __cplusplus
}
#endif
