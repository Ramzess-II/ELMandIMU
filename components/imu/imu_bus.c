#include "imu_bus.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"

// Запись во флеш (NVS, новая прошивка) останавливает оба ядра на время стирания сектора, это десятки
// миллисекунд. Обмен, попавший на это время, должен дождаться конца, а не сорваться.
#define IMU_BUS_TIMEOUT_MS   100
// При поиске датчика перебирается больше ста адресов: здесь ждать долго нельзя.
#define IMU_PROBE_TIMEOUT_MS 20

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static imu_bus_config_t s_cfg;
static uint8_t s_addr;

static esp_err_t bus_new(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = s_cfg.port,
        .sda_io_num = s_cfg.pin_sda_mosi,
        .scl_io_num = s_cfg.pin_scl_sclk,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        // Внутренние подтяжки слабые, на плате датчика должны стоять свои 4.7–10 кОм.
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_bus);
}

static esp_err_t dev_add(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr,
        .scl_speed_hz = s_cfg.freq_hz,
    };
    return i2c_master_bus_add_device(s_bus, &cfg, &s_dev);
}

esp_err_t imu_bus_open(const imu_bus_config_t *bus)
{
    if (bus->type != IMU_BUS_I2C) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_bus) {
        return ESP_OK;
    }
    s_cfg = *bus;
    return bus_new();
}

void imu_bus_close(void)
{
    imu_bus_detach();
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

void imu_bus_reset(void)
{
    if (s_bus) {
        i2c_master_bus_reset(s_bus);
    }
}

// Датчик, обмен с которым оборвался посреди байта, держит SDA: тактируем SCL, пока он её не отпустит,
// и подаём STOP.
static void lines_free(void)
{
    int sda = s_cfg.pin_sda_mosi, scl = s_cfg.pin_scl_sclk;
    gpio_config_t io = {
        .pin_bit_mask = BIT64(sda) | BIT64(scl),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    for (int i = 0; i < 18 && !gpio_get_level(sda); i++) {
        gpio_set_level(scl, 0);
        esp_rom_delay_us(5);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(5);
    }
    gpio_set_level(scl, 0);
    esp_rom_delay_us(5);
    gpio_set_level(sda, 0);
    esp_rom_delay_us(5);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(5);
    gpio_set_level(sda, 1);
    esp_rom_delay_us(5);
}

// После сорванного обмена драйвер I²C из ESP-IDF может повиснуть на следующем же обмене навсегда: ждёт,
// пока шина освободится, без ограничения по времени. Поэтому после любой неудачи шина создаётся заново.
static void recover(void)
{
    bool had_dev = s_dev != NULL;
    imu_bus_detach();
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    lines_free();
    if (bus_new() == ESP_OK && had_dev) {
        dev_add();
    }
}

bool imu_bus_probe(uint8_t addr)
{
    return s_bus && i2c_master_probe(s_bus, addr, IMU_PROBE_TIMEOUT_MS) == ESP_OK;
}

esp_err_t imu_bus_attach(uint8_t addr)
{
    imu_bus_detach();
    if (!s_bus) {
        return ESP_ERR_INVALID_STATE;
    }
    s_addr = addr;
    return dev_add();
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
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, IMU_BUS_TIMEOUT_MS);
    if (err != ESP_OK) {
        recover();
    }
    return err;
}

esp_err_t imu_bus_write(uint8_t reg, uint8_t val)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t b[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(s_dev, b, sizeof(b), IMU_BUS_TIMEOUT_MS);
    if (err != ESP_OK) {
        recover();
    }
    return err;
}
