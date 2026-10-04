#include "events.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "proto.h"

#define TAG "event"

static event_t s_ring[EVENTS_KEEP];
static uint32_t s_last;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void events_add(char level, const char *code, const char *fmt, ...)
{
    event_t ev = {
        .t_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .level = level,
    };
    strlcpy(ev.code, code, sizeof(ev.code));
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(ev.text, sizeof(ev.text), fmt, ap);
    va_end(ap);
    proto_clean_text(ev.text);

    taskENTER_CRITICAL(&s_lock);
    ev.num = ++s_last;
    s_ring[ev.num % EVENTS_KEEP] = ev;
    taskEXIT_CRITICAL(&s_lock);

    if (level == 'E') {
        ESP_LOGE(TAG, "#%lu %s %s", (unsigned long)ev.num, ev.code, ev.text);
    } else if (level == 'W') {
        ESP_LOGW(TAG, "#%lu %s %s", (unsigned long)ev.num, ev.code, ev.text);
    } else {
        ESP_LOGI(TAG, "#%lu %s %s", (unsigned long)ev.num, ev.code, ev.text);
    }
}

uint32_t events_last(void)
{
    taskENTER_CRITICAL(&s_lock);
    uint32_t n = s_last;
    taskEXIT_CRITICAL(&s_lock);
    return n;
}

bool events_get(uint32_t num, event_t *out)
{
    bool ok = false;
    taskENTER_CRITICAL(&s_lock);
    if (num >= 1 && num <= s_last && s_last - num < EVENTS_KEEP) {
        *out = s_ring[num % EVENTS_KEEP];
        ok = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}
