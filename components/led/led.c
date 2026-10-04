#include "led.h"

#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_NOGPS_LED_WS2812
#include "led_strip.h"
#else
#include "driver/gpio.h"
#endif

#define TAG "led"
#define SLOT_MS 100
#define SLOTS   20      // кадр 2 с

static volatile led_base_t s_base = LED_OFF;
static volatile int s_blinks;

#if CONFIG_NOGPS_LED_WS2812
static led_strip_handle_t s_strip;

static esp_err_t hw_init(void)
{
    led_strip_config_t sc = {
        .strip_gpio_num = CONFIG_NOGPS_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rc = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    return led_strip_new_rmt_device(&sc, &rc, &s_strip);
}

static void hw_set(bool on)
{
    if (on) {
        led_strip_set_pixel(s_strip, 0, 0, CONFIG_NOGPS_LED_BRIGHTNESS, 0);
        led_strip_refresh(s_strip);
    } else {
        led_strip_clear(s_strip);
    }
}
#else
#if CONFIG_NOGPS_LED_ACTIVE_LOW
#define LED_ON_LEVEL 0
#else
#define LED_ON_LEVEL 1
#endif

static void hw_set(bool on)
{
    gpio_set_level(CONFIG_NOGPS_LED_GPIO, on ? LED_ON_LEVEL : !LED_ON_LEVEL);
}

static esp_err_t hw_init(void)
{
    gpio_reset_pin(CONFIG_NOGPS_LED_GPIO);
    hw_set(false);      // сначала уровень «погашен», потом выход — без вспышки при включении
    return gpio_set_direction(CONFIG_NOGPS_LED_GPIO, GPIO_MODE_OUTPUT);
}
#endif

static bool lit(led_base_t base, int blinks, int slot)
{
    bool blink = blinks > 0 && slot < 2 * blinks && slot % 2 == 0;
    switch (base) {
    case LED_FAST:
        return slot % 2 == 0;
    case LED_IDLE:
        return blinks ? blink : slot % 10 == 0;   // 0.1 с горит, 0.9 с нет
    case LED_ON:
        return !blink;
    default:
        return false;
    }
}

static void led_task(void *arg)
{
    bool prev = false;
    hw_set(false);
    for (int slot = 0;; slot = (slot + 1) % SLOTS) {
        bool on = lit(s_base, s_blinks, slot);
        if (on != prev) {
            hw_set(on);
            prev = on;
        }
        vTaskDelay(pdMS_TO_TICKS(SLOT_MS));
    }
}

esp_err_t led_start(void)
{
    esp_err_t err = hw_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d: %s", CONFIG_NOGPS_LED_GPIO, esp_err_to_name(err));
        return err;
    }
    return xTaskCreate(led_task, "led_task", 3072, NULL, 1, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void led_set(led_base_t base, int error_blinks)
{
    s_base = base;
    s_blinks = error_blinks;
}
