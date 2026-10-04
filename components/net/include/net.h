// Wi-Fi и UDP (ТЗ, разделы 8 и 9): точка доступа NoGPS-XXXX, 192.168.4.1:4210, строки NGD/NGS/NGE/NGA/NGT.
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
    bool phone;       // HELLO приходил не позже 3 с назад
} net_state_t;

// NVS, config, events и motion уже запущены.
esp_err_t net_start(const char *fw_version);
void net_get_state(net_state_t *out);

// Включить или выключить точку доступа: на стоянке с заглушенным двигателем Wi-Fi не нужен.
// Журнал событий при этом копится, телефон заберёт его после включения.
void net_set_wifi(bool on);

#ifdef __cplusplus
}
#endif
