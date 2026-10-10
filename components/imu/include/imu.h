// Интерфейс IMU (ТЗ, раздел 3). Ни один компонент, кроме imu, не знает, какая стоит микросхема.
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   gyro[3];    // угловая скорость, рад/с, оси датчика, правая тройка
    float   accel[3];   // ускорение, м/с², оси датчика; неподвижный датчик даёт +9.81 по оси «вверх»
    float   temp_c;     // температура кристалла, °C, если has_temp
    bool    has_temp;
    int64_t t_us;       // момент отсчёта, esp_timer_get_time()
} imu_sample_t;

typedef enum {
    IMU_BUS_I2C,
    IMU_BUS_SPI,
} imu_bus_type_t;

typedef struct {
    imu_bus_type_t type;
    int port;           // номер I²C или SPI
    int pin_sda_mosi, pin_scl_sclk, pin_miso, pin_cs, pin_int;  // -1, если не используется
    uint8_t i2c_addr;   // 0 — перебрать адреса драйверов
    int freq_hz;
} imu_bus_config_t;

typedef struct {
    int odr_hz;          // желаемая частота отсчётов, не ниже 200
    int gyro_range_dps;  // 250 по умолчанию
    int accel_range_g;   // 4 по умолчанию
    int lpf_hz;          // встроенный фильтр датчика, 40–50 Гц
    imu_bus_config_t bus;
} imu_config_t;

// Описание драйвера. Чтобы добавить датчик: новый файл в drivers/ и строка в imu_registry.c.
typedef struct {
    const char    *name;        // "ICM20602" — уходит в NGS
    const uint8_t *i2c_addrs;   // адреса, на которых может стоять микросхема, оканчиваются 0
    uint8_t        id_reg;      // регистр WHO_AM_I
    const uint8_t *id_values;   // допустимые значения WHO_AM_I, оканчиваются 0
    // Шина уже открыта ядром на найденном адресе (imu_bus.h).
    esp_err_t (*init)(const imu_config_t *cfg, int *actual_odr_hz);
    // Читает накопленные отсчёты, не больше max. Число отсчётов, 0 если новых нет, -1 при ошибке шины.
    int       (*read)(imu_sample_t *out, int max);
    esp_err_t (*deinit)(void);
} imu_driver_t;

// Все драйверы, вкомпилированные в прошивку, в порядке перебора, оканчиваются NULL.
extern const imu_driver_t *const imu_drivers[];

typedef enum {
    IMU_STATE_INIT,
    IMU_STATE_NOT_FOUND,
    IMU_STATE_WRONG_CHIP,
    IMU_STATE_BUS_ERROR,
    IMU_STATE_NO_DATA,
    IMU_STATE_OK,
} imu_state_t;

typedef struct {
    imu_state_t state;
    const char *name;       // имя драйвера или NULL
    uint8_t     addr;       // I²C-адрес найденной микросхемы, 0 если нет
    int         chip_id;    // прочитанный WHO_AM_I, -1 если никто не ответил
    int         odr_hz;     // частота по настройке датчика
    float       odr_measured_hz;  // фактическая частота за последнюю секунду
    uint32_t    errors;     // ошибок чтения с включения
    bool        stalled;    // задача гироскопа перестала работать; state при этом NO_DATA
} imu_status_t;

// Вызывается из imu_task на каждую пачку отсчётов. Должен работать быстро.
typedef void (*imu_sample_cb_t)(const imu_sample_t *samples, int n, void *ctx);

// Запускает imu_task: автоопределение, чтение, восстановление после ошибок.
esp_err_t imu_start(const imu_config_t *cfg, imu_sample_cb_t cb, void *ctx);
void imu_get_status(imu_status_t *out);
const char *imu_state_name(imu_state_t s);

// Усыпить датчик на время сна блока или разбудить: после пробуждения — заново автоопределение.
void imu_set_suspended(bool suspended);
// То же на время приёма новой прошивки: запись во флеш почти не оставляет времени на обмен с датчиком.
void imu_set_paused(bool paused);

#ifdef __cplusplus
}
#endif
