// Скорость из OBD-II через ELM327 (ТЗ, раздел 4). Компонент необязательный (раздел 5):
// ни один другой компонент не знает про ELM327, скорость уходит наружу через колбэк.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBD_STATE_DISABLED,
    OBD_STATE_NO_ADAPTER,
    OBD_STATE_INIT,
    OBD_STATE_SEARCHING,
    OBD_STATE_NO_CAR,
    OBD_STATE_NO_DATA,
    OBD_STATE_ERROR,
    OBD_STATE_OK,
    OBD_STATE_SLEEP,   // стоянка: блок ничего не шлёт ни адаптеру, ни машине, чтобы не будить её
} obd_state_t;

typedef enum {
    OBD_ERROR_NONE,
    OBD_ERROR_NO_ADAPTER,
    OBD_ERROR_NO_CAR,
    OBD_ERROR_NO_SPEED,
    OBD_ERROR_BUS_ERROR,
    OBD_ERROR_TIMEOUT,
    OBD_ERROR_BAD_REPLY,
} obd_error_t;

// Работает ли двигатель по данным машины: скорость больше нуля или обороты больше нуля.
// UNKNOWN — машина не отвечает или не отдаёт обороты; тогда решает напряжение (компонент power).
typedef enum {
    OBD_ENGINE_UNKNOWN,
    OBD_ENGINE_OFF,
    OBD_ENGINE_RUN,
} obd_engine_t;

typedef struct {
    obd_state_t  state;
    obd_error_t  error;
    char         protocol[48];  // ответ ATDP, пусто если неизвестен
    int          baud;          // найденная скорость UART, 0 если нет
    obd_engine_t engine;
    // Обороты и напряжения спрашиваются только при скорости 0: на ходу адаптер занят одной скоростью.
    int          rpm;           // обороты (PID 0C), -1 — нет или машина едет
    int          voltage_mv;    // напряжение по ATRV (его меряет сам адаптер), -1 если неизвестно
    int          ecu_mv;        // напряжение от блока управления машины (PID 42), -1 если не отдаёт
    uint32_t     volt_seq;      // растёт с каждым новым замером напряжений
} obd_status_t;

// Скорость, км/ч, и момент её измерения (esp_timer_get_time) — OBD-6.
typedef void (*obd_speed_cb_t)(int kmh, int64_t t_us);

// Запускает obd_task. Без вызова состояние остаётся DISABLED (NO-2).
esp_err_t   obd_start(obd_speed_cb_t cb);
void        obd_get_status(obd_status_t *out);
// Пауза на стоянке: состояние SLEEP, UART молчит. После снятия паузы — заново поиск протокола.
void        obd_set_paused(bool paused);
const char *obd_state_name(obd_state_t s);
const char *obd_error_name(obd_error_t e);

#ifdef __cplusplus
}
#endif
