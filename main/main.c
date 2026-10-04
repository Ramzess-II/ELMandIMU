// Блок датчиков NoGPS: запуск компонентов и светодиод состояния. Устройство прошивки — ТЗ, раздел 2.
#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "config.h"
#include "events.h"
#include "imu.h"
#include "led.h"
#include "motion.h"
#include "net.h"
#include "obd.h"
#include "power.h"

#define TAG "main"
#define FW_VERSION "0.2.0"
#define STATUS_LOG_MS 5000

static const char *reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_SW:       return "SW";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT:      return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_USB:      return "USB";
    default:               return "OTHER";
    }
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

// Экономия на стоянке — компонент power: он решает, нужен ли Wi-Fi, здесь решение только исполняется.
static bool s_wifi = true;

static void update_power(void)
{
    power_status_t pw;
    power_get_status(&pw);
    if (pw.wifi != s_wifi) {
        s_wifi = pw.wifi;
        net_set_wifi(s_wifi);
    }
}

// Раздел 11: если ошибок несколько, показывается первая по таблице.
static void update_led(void)
{
    if (!s_wifi) {
        led_set(LED_OFF, 0);
        return;
    }
    net_state_t net;
    imu_status_t imu;
    obd_status_t obd;
    motion_status_t mo;
    net_get_state(&net);
    imu_get_status(&imu);
    obd_get_status(&obd);
    motion_get_status(&mo);

    int blinks = 0;
    if (imu.state != IMU_STATE_OK && imu.state != IMU_STATE_INIT) {
        blinks = 2;
    } else if (obd.state != OBD_STATE_DISABLED && obd.state != OBD_STATE_OK &&
               obd.state != OBD_STATE_SLEEP) {
        blinks = 3;
    } else if (mo.flags & MOTION_MOUNT_MOVED) {
        blinks = 4;
    }
    led_base_t base = !net.wifi_up || net.wifi_error ? LED_FAST : net.phone ? LED_ON : LED_IDLE;
    led_set(base, base == LED_FAST ? 0 : blinks);
}

static void log_status(void)
{
    imu_status_t imu;
    motion_status_t mo;
    net_state_t net;
    imu_get_status(&imu);
    motion_get_status(&mo);
    net_get_state(&net);
    ESP_LOGI(TAG, "IMU %s %.0f Гц | телефон %s | тряска %.2f °/с %.3f м/с² (пороги %.2f %.3f) | "
             "смещение %d мдег/с (%s) | %s%s%s",
             imu_state_name(imu.state), imu.odr_measured_hz, net.phone ? "да" : "нет",
             mo.shake_gyro_dps, mo.shake_acc, config_get_float("still_gyro"),
             config_get_float("still_acc"),
             mo.bias_up_mdps, (mo.flags & MOTION_BIAS_OK) ? "есть" : "нет",
             (mo.flags & MOTION_UP_OK) ? "вертикаль есть" : "вертикали нет",
             (mo.flags & MOTION_STILL) ? ", стоит" : "",
             (mo.flags & MOTION_MOUNT_MOVED) ? ", БЛОК СДВИНУТ" : "");
    obd_status_t obd;
    obd_get_status(&obd);
    power_status_t pw;
    power_get_status(&pw);
    ESP_LOGI(TAG, "OBD %s | двигатель %s, обороты %d | сеть: АЦП %d мВ (%s, k %d), ELM %d, машина %d | "
             "%s, уровень засыпания %d мВ, Wi-Fi %s",
             obd_state_name(obd.state),
             pw.engine == OBD_ENGINE_RUN ? "работает" : pw.engine == OBD_ENGINE_OFF ? "заглушен"
                                                                                     : "неизвестно",
             obd.rpm, pw.adc_mv, pw.adc_trusted ? "подтверждён" : "не подтверждён", pw.k_milli,
             obd.voltage_mv, obd.ecu_mv, pw.parked ? "стоянка" : "работа", pw.base_mv,
             s_wifi ? "включён" : "выключен");
}

void app_main(void)
{
    init_nvs();
    config_init(CONFIG_NOGPS_WIFI_PASS, CONFIG_NOGPS_IMU_ODR);
    events_add('I', "BOOT", "fw %s reset %s", FW_VERSION, reset_reason());

    ESP_ERROR_CHECK(led_start());
    led_set(LED_FAST, 0);
    ESP_ERROR_CHECK(motion_init());

    imu_config_t cfg = {
        .odr_hz = config_get_int("imu_odr"),
        .gyro_range_dps = CONFIG_NOGPS_IMU_GYRO_RANGE,
        .accel_range_g = CONFIG_NOGPS_IMU_ACCEL_RANGE,
        .lpf_hz = CONFIG_NOGPS_IMU_LPF,
        .bus = {
            .type = IMU_BUS_I2C,
            .port = CONFIG_NOGPS_IMU_I2C_PORT,
            .pin_sda_mosi = CONFIG_NOGPS_IMU_SDA,
            .pin_scl_sclk = CONFIG_NOGPS_IMU_SCL,
            .pin_miso = -1,
            .pin_cs = -1,
            .pin_int = -1,
            .i2c_addr = 0,
            .freq_hz = CONFIG_NOGPS_IMU_I2C_FREQ,
        },
    };
    // Отказ одного компонента не останавливает остальные (ТЗ, раздел 2).
    if (imu_start(&cfg, motion_on_samples, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "IMU не запустился");
    }
    if (config_get_int("obd_enabled")) {
        obd_start(motion_set_speed);
    }
    if (net_start(FW_VERSION) != ESP_OK) {
        ESP_LOGE(TAG, "сеть не запустилась");
    }
    if (power_start() != ESP_OK) {
        ESP_LOGE(TAG, "контроль питания не запустился");
    }

    for (int t = 0;; t += 100) {
        update_power();
        update_led();
        if (t % STATUS_LOG_MS == 0) {
            log_status();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
