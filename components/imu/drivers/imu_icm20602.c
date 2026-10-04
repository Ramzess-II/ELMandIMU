// Драйвер ICM-20602 (TDK InvenSense), I²C. Ссылки — на даташит DS-000176 rev 1.0.
// Чтение через FIFO: акселерометр, температура и гироскоп, 14 байт на отсчёт.
#include <math.h>
#include <stddef.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "imu_bus.h"

#define REG_SMPLRT_DIV   0x19   // 9.15
#define REG_CONFIG       0x1A   // 9.16
#define REG_GYRO_CONFIG  0x1B   // 9.17
#define REG_ACCEL_CONFIG 0x1C   // 9.18
#define REG_ACCEL_CONFIG2 0x1D  // 9.19
#define REG_FIFO_EN      0x23   // 9.24
#define REG_INT_STATUS   0x3A   // 9.28
#define REG_USER_CTRL    0x6A   // 9.36
#define REG_PWR_MGMT_1   0x6B   // 9.37
#define REG_PWR_MGMT_2   0x6C   // 9.38
#define REG_FIFO_COUNTH  0x72   // 9.40
#define REG_FIFO_R_W     0x74   // 9.41
#define REG_WHO_AM_I     0x75   // 9.42

#define PWR1_DEVICE_RESET 0x80
#define PWR1_CLKSEL_AUTO  0x01  // 10.7: CLKSEL = 001 обязателен для полной точности гироскопа
#define FIFO_EN_GYRO      0x10
#define FIFO_EN_ACCEL     0x08
#define USER_CTRL_FIFO_EN 0x40
#define USER_CTRL_FIFO_RST 0x04
#define INT_FIFO_OFLOW    0x10

#define FRAME_BYTES 14          // ACCEL XYZ, TEMP, GYRO XYZ — 9.24, 9.41
#define FIFO_BYTES  1008        // 4.12
#define INTERNAL_RATE_HZ 1000   // при FCHOICE_B = 00 и DLPF_CFG 1..6, 9.15

#define G_MS2 9.80665f
#define DEG2RAD ((float)M_PI / 180.0f)

static const uint8_t s_addrs[] = {0x68, 0x69, 0};
static const uint8_t s_ids[] = {0x12, 0};

static float s_gyro_scale;   // рад/с на LSB
static float s_accel_scale;  // м/с² на LSB
static int64_t s_period_us;
static uint8_t s_buf[FIFO_BYTES];

// Полоса гироскопа по DLPF_CFG 1..6 (9.16), Гц.
static const int s_gyro_bw[] = {0, 176, 92, 41, 20, 10, 5};
// Полоса акселерометра по A_DLPF_CFG 1..6 (9.19), Гц, с округлением.
static const int s_accel_bw[] = {0, 218, 99, 45, 21, 10, 5};

// Самая узкая полоса, которая не уже запрошенной.
static uint8_t pick_dlpf(const int *bw, int lpf_hz)
{
    uint8_t cfg = 1;
    for (uint8_t i = 1; i <= 6; i++) {
        if (bw[i] >= lpf_hz) {
            cfg = i;
        }
    }
    return cfg;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

static esp_err_t fifo_reset(void)
{
    esp_err_t err = imu_bus_write(REG_USER_CTRL, USER_CTRL_FIFO_RST);
    if (err == ESP_OK) {
        err = imu_bus_write(REG_USER_CTRL, USER_CTRL_FIFO_EN);
    }
    return err;
}

#define CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) return e_; } while (0)

static esp_err_t icm20602_init(const imu_config_t *cfg, int *actual_odr_hz)
{
    // После включения чип спит, 9.1. Сброс и пробуждение с автовыбором тактирования.
    CHECK(imu_bus_write(REG_PWR_MGMT_1, PWR1_DEVICE_RESET));
    vTaskDelay(pdMS_TO_TICKS(100));
    CHECK(imu_bus_write(REG_PWR_MGMT_1, PWR1_CLKSEL_AUTO));
    vTaskDelay(pdMS_TO_TICKS(50));
    CHECK(imu_bus_write(REG_PWR_MGMT_2, 0x00));  // все оси включены

    // CONFIG: бит 7 после сброса 1, его нужно записать 0; FIFO_MODE = 0 — при переполнении
    // затираются старые отсчёты; DLPF гироскопа.
    CHECK(imu_bus_write(REG_CONFIG, pick_dlpf(s_gyro_bw, cfg->lpf_hz)));

    // ODR = 1000 / (1 + div), не ниже запрошенной.
    int odr = cfg->odr_hz > 0 ? cfg->odr_hz : 400;
    int div = INTERNAL_RATE_HZ / odr - 1;
    div = div < 0 ? 0 : (div > 255 ? 255 : div);
    CHECK(imu_bus_write(REG_SMPLRT_DIV, (uint8_t)div));
    *actual_odr_hz = INTERNAL_RATE_HZ / (1 + div);
    s_period_us = 1000000LL * (1 + div) / INTERNAL_RATE_HZ;

    // Диапазон гироскопа, FCHOICE_B = 00. Чувствительность 131 / 65.5 / 32.8 / 16.4 LSB/(°/с), 3.1.
    uint8_t fs;
    float lsb_per_dps;
    if (cfg->gyro_range_dps <= 250)       { fs = 0; lsb_per_dps = 131.0f; }
    else if (cfg->gyro_range_dps <= 500)  { fs = 1; lsb_per_dps = 65.5f; }
    else if (cfg->gyro_range_dps <= 1000) { fs = 2; lsb_per_dps = 32.8f; }
    else                                  { fs = 3; lsb_per_dps = 16.4f; }
    CHECK(imu_bus_write(REG_GYRO_CONFIG, fs << 3));
    s_gyro_scale = DEG2RAD / lsb_per_dps;

    // Диапазон акселерометра. 16384 / 8192 / 4096 / 2048 LSB/g, 3.2.
    uint8_t afs;
    if (cfg->accel_range_g <= 2)      afs = 0;
    else if (cfg->accel_range_g <= 4) afs = 1;
    else if (cfg->accel_range_g <= 8) afs = 2;
    else                              afs = 3;
    CHECK(imu_bus_write(REG_ACCEL_CONFIG, afs << 3));
    s_accel_scale = G_MS2 / (float)(16384 >> afs);

    // ACCEL_FCHOICE_B = 0, A_DLPF_CFG.
    CHECK(imu_bus_write(REG_ACCEL_CONFIG2, pick_dlpf(s_accel_bw, cfg->lpf_hz)));

    // Проверка, что записи дошли.
    uint8_t v;
    CHECK(imu_bus_read(REG_GYRO_CONFIG, &v, 1));
    if (v != (fs << 3)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    CHECK(imu_bus_write(REG_FIFO_EN, 0));
    CHECK(fifo_reset());
    CHECK(imu_bus_write(REG_FIFO_EN, FIFO_EN_GYRO | FIFO_EN_ACCEL));
    uint8_t st;
    imu_bus_read(REG_INT_STATUS, &st, 1);  // сбросить старые флаги
    return ESP_OK;
}

static int icm20602_read(imu_sample_t *out, int max)
{
    uint8_t c[2];
    if (imu_bus_read(REG_FIFO_COUNTH, c, 2) != ESP_OK) {
        return -1;
    }
    int64_t t_now = esp_timer_get_time();
    int count = (c[0] << 8) | c[1];

    // Переполнение: FIFO затёр старые отсчёты, граница кадров могла сбиться. Начать заново.
    if (count >= FIFO_BYTES - FRAME_BYTES) {
        uint8_t st;
        imu_bus_read(REG_INT_STATUS, &st, 1);
        return fifo_reset() == ESP_OK ? 0 : -1;
    }

    int n = count / FRAME_BYTES;
    if (n > max) {
        n = max;
    }
    if (n == 0) {
        return 0;
    }
    if (imu_bus_read(REG_FIFO_R_W, s_buf, n * FRAME_BYTES) != ESP_OK) {
        return -1;
    }
    // Последний отсчёт в FIFO считаем снятым в момент чтения счётчика, остальные — через период датчика.
    int left = count / FRAME_BYTES - n;
    for (int i = 0; i < n; i++) {
        const uint8_t *p = &s_buf[i * FRAME_BYTES];
        imu_sample_t *s = &out[i];
        for (int k = 0; k < 3; k++) {
            s->accel[k] = be16(p + 2 * k) * s_accel_scale;
            s->gyro[k] = be16(p + 8 + 2 * k) * s_gyro_scale;
        }
        s->temp_c = be16(p + 6) / 326.8f + 25.0f;  // 9.30
        s->has_temp = true;
        s->t_us = t_now - (int64_t)(n - 1 - i + left) * s_period_us;
    }
    return n;
}

static esp_err_t icm20602_deinit(void)
{
    imu_bus_write(REG_FIFO_EN, 0);
    return imu_bus_write(REG_PWR_MGMT_1, 0x40 | PWR1_CLKSEL_AUTO);  // SLEEP
}

const imu_driver_t imu_drv_icm20602 = {
    .name = "ICM20602",
    .i2c_addrs = s_addrs,
    .id_reg = REG_WHO_AM_I,
    .id_values = s_ids,
    .init = icm20602_init,
    .read = icm20602_read,
    .deinit = icm20602_deinit,
};
