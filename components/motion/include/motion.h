// Обработка отсчётов IMU (ТЗ, разделы 6 и 7): неподвижность, курс, путь, автообнуление, калибровка.
// Не знает, какой стоит датчик. Вся обработка идёт в imu_task через motion_on_samples.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

// Биты флагов NGD, которые выставляет motion (ТЗ 9.3).
#define MOTION_BIAS_OK     0x002
#define MOTION_UP_OK       0x004
#define MOTION_FWD_OK      0x008
#define MOTION_OBD_OK      0x010
#define MOTION_STILL       0x020
#define MOTION_CALIBRATING 0x040
#define MOTION_MOUNT_MOVED 0x080

typedef struct {
    uint32_t t_ms;          // время последнего отсчёта IMU
    int64_t  yaw_mdeg;      // накопленный поворот, направо плюс
    int32_t  rate_mdps;     // средняя скорость поворота с прошлого пакета
    uint64_t dist_mm;
    int      speed_kmh;     // -1 — нет
    int      speed_age_ms;  // -1 — нет
    uint32_t flags;         // биты MOTION_*
    // Ускорения за интервал пакета, мм/с², в осях машины: две горизонтальные и вертикаль (ТЗ 9.3, поля
    // 11–14). Есть только при откалиброванной вертикали и если за интервал были отсчёты.
    bool     has_acc;
    int32_t  acc_h1_mms2;   // среднее вдоль горизонтальной оси 1
    int32_t  acc_h2_mms2;   // среднее вдоль горизонтальной оси 2
    int32_t  acc_up_mms2;   // среднее вдоль «вверх» минус 9.81 м/с²
    int32_t  jolt_mms2;     // наибольшее отклонение вдоль «вверх» от среднего за интервал, без знака
} motion_packet_t;

typedef struct {
    int      bias_age_s;    // -1, если смещение не обновлялось с включения
    int      bias_up_mdps;  // смещение нуля по оси «вверх»
    bool     has_temp;
    float    temp_c;
    uint32_t flags;
    float    shake_gyro_dps;  // тряска за последние 2 с: наибольший по осям разброс гироскопа
    float    shake_acc;       // и разброс модуля ускорения, м/с² — для подбора still_gyro и still_acc
} motion_status_t;

typedef enum {
    MOTION_CMD_CAL_BIAS,
    MOTION_CMD_CAL_UP,
    MOTION_CMD_CAL_UP2,
    MOTION_CMD_CAL_RESET,
} motion_cmd_t;

// Ответ на команду: PROGRESS (progress 0..100) или окончательный OK / ERR.
typedef struct {
    int  seq;
    int  progress;   // -1 — окончательный ответ
    bool ok;
    char err[16];
} motion_reply_t;

esp_err_t motion_init(void);

// Колбэк для imu_start().
void motion_on_samples(const imu_sample_t *s, int n, void *ctx);

// Скорость из OBD, км/ч, и момент её получения (esp_timer_get_time).
void motion_set_speed(int kmh, int64_t t_us);

// Данные для NGD; сбрасывает накопление средней скорости поворота.
void motion_take_packet(motion_packet_t *out);
void motion_get_status(motion_status_t *out);

// Команда калибровки. NULL — принята, ответы придут через motion_get_reply; иначе код ошибки для ERR.
const char *motion_command(motion_cmd_t cmd, int seq);
bool motion_get_reply(motion_reply_t *out);

// Звать периодически: снимает зависшую калибровку, если IMU перестал присылать отсчёты.
void motion_tick(void);

// Последний сырой отсчёт, для IMU_TEST. false, если отсчётов ещё не было.
bool motion_last_sample(imu_sample_t *out);

#ifdef __cplusplus
}
#endif
