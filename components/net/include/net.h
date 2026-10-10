// Связь с телефоном (ТЗ, разделы 8 и 9): строки NGD/NGS/NGE/NGA/NGT/NGI по двум транспортам сразу —
// Wi-Fi (точка доступа NoGPS-XXXX, UDP 192.168.4.1:4210) и Bluetooth LE (ble.h). Данные идут тому,
// кто последним прислал HELLO.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NET_PORT 4210

typedef struct {
    bool wifi_up;     // точка доступа поднята
    bool wifi_error;
    bool phone;       // HELLO приходил не позже 3 с назад, по любому транспорту
    bool bt_up;       // Bluetooth включён
    bool bt_adv;      // блок рекламируется: телефон может его найти
    bool bt_conn;     // кто-то подключён по Bluetooth (проверку мог ещё не пройти)
    bool bt_link;     // привязанный телефон подключён по Bluetooth, связь зашифрована
    bool bt_pairing;  // открыто окно сопряжения: можно привязать новый телефон
    int  bt_bonds;    // сколько телефонов привязано
} net_state_t;

// NVS, config, events и motion уже запущены.
esp_err_t net_start(const char *fw_version);
void net_get_state(net_state_t *out);

// Включить или выключить всё радио — точку доступа Wi-Fi и Bluetooth (рекламу и соединение): на
// стоянке с заглушенным двигателем оно не нужно. Возвращается, когда радио действительно выключено.
// Журнал событий при этом копится, телефон заберёт его после включения.
void net_set_radio(bool on);

#ifdef __cplusplus
}
#endif
