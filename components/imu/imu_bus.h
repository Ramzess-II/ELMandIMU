// Доступ драйверов к шине. Шину открывает и устройство подключает ядро imu.c.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "imu.h"

esp_err_t imu_bus_open(const imu_bus_config_t *bus);
void      imu_bus_close(void);
// true, если по адресу кто-то ответил ACK.
bool      imu_bus_probe(uint8_t addr);
// Подключить устройство по адресу, через него идут все чтения и записи ниже.
esp_err_t imu_bus_attach(uint8_t addr);
void      imu_bus_detach(void);

esp_err_t imu_bus_read(uint8_t reg, uint8_t *buf, size_t len);
esp_err_t imu_bus_write(uint8_t reg, uint8_t val);
