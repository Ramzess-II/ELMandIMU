#include "imu_bus.h"

#include "driver/i2c_master.h"

#define IMU_BUS_TIMEOUT_MS 20

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static int s_freq_hz;

esp_err_t imu_bus_open(const imu_bus_config_t *bus)
{
    if (bus->type != IMU_BUS_I2C) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_bus) {
        return ESP_OK;
    }
    i2c_master_bus_config_t cfg = {
        .i2c_port = bus->port,
        .sda_io_num = bus->pin_sda_mosi,
        .scl_io_num = bus->pin_scl_sclk,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Внутренние подтяжки слабые, на плате датчика должны стоять свои 4.7–10 кОм.
        .flags.enable_internal_pullup = true,
    };
    s_freq_hz = bus->freq_hz;
    return i2c_new_master_bus(&cfg, &s_bus);
}

void imu_bus_close(void)
{
    imu_bus_detach();
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

bool imu_bus_probe(uint8_t addr)
{
    return s_bus && i2c_master_probe(s_bus, addr, IMU_BUS_TIMEOUT_MS) == ESP_OK;
}

esp_err_t imu_bus_attach(uint8_t addr)
{
    imu_bus_detach();
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = s_freq_hz,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, &s_dev);
}

void imu_bus_detach(void)
{
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
}

esp_err_t imu_bus_read(uint8_t reg, uint8_t *buf, size_t len)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, IMU_BUS_TIMEOUT_MS);
}

esp_err_t imu_bus_write(uint8_t reg, uint8_t val)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t b[2] = {reg, val};
    return i2c_master_transmit(s_dev, b, sizeof(b), IMU_BUS_TIMEOUT_MS);
}
