// Хранение калибровки в NVS (ТЗ, раздел 7): вертикаль, направление вперёд, смещение нуля.
// Командой SET не меняется.
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool  up_ok;
    float up[3];        // единичный «вверх» в осях датчика
    float up1[3];       // первая точка CAL_UP, для CAL_UP2
    bool  fwd_ok;
    float fwd[3];
    bool  bias_ok;
    float bias[3];      // смещение нуля, рад/с
    bool  bias_has_temp;
    float bias_temp_c;  // температура при замере смещения
} calib_data_t;

// Пустая калибровка, если в NVS ничего нет.
void      calib_load(calib_data_t *out);
esp_err_t calib_save(const calib_data_t *in);

#ifdef __cplusplus
}
#endif
