// Реестр драйверов IMU. Новый датчик: extern его структуру и добавить строку в массив.
#include "imu.h"

extern const imu_driver_t imu_drv_icm20602;

const imu_driver_t *const imu_drivers[] = {
    &imu_drv_icm20602,
    NULL,
};
