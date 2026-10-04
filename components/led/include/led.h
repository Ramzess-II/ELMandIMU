// Светодиод состояния (ТЗ, раздел 11). Что показывать, решает main; здесь только мигание.
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_OFF,
    LED_FAST,   // 5 раз в секунду: Wi-Fi поднимается или ошибка Wi-Fi
    LED_IDLE,   // 0.1 с горит, 0.9 с нет: ждём телефон
    LED_ON,     // горит: телефон подключён
} led_base_t;

esp_err_t led_start(void);

// error_blinks: 0 — без ошибки, 2–4 — столько раз за 2 с гаснет (на LED_ON) или вспыхивает (на LED_IDLE).
void led_set(led_base_t base, int error_blinks);

#ifdef __cplusplus
}
#endif
