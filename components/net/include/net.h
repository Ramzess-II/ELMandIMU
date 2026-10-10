// Связь с телефоном (ТЗ, разделы 9 и 17): строки NGD/NGS/NGE/NGA/NGT/NGI/NGO по Bluetooth LE (ble.h).
// Wi-Fi в блоке нет (NET-8). Данные идут, пока телефон раз в секунду шлёт HELLO.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool phone;       // HELLO приходил не позже 3 с назад
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

// Включить или выключить радио — Bluetooth целиком, рекламу и соединение: на стоянке с заглушенным
// двигателем оно не нужно. Возвращается, когда радио действительно выключено.
// Журнал событий при этом копится, телефон заберёт его после включения.
void net_set_radio(bool on);

#ifdef __cplusplus
}
#endif
