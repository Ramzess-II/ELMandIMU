// Журнал ошибок и событий (ТЗ, раздел 10): последние 32 в памяти, номера с 1 от включения.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EVENTS_KEEP 32

typedef struct {
    uint32_t num;
    uint32_t t_ms;
    char     level;      // 'E', 'W', 'I'
    char     code[24];
    char     text[72];
} event_t;

// Добавить событие и напечатать его в лог. Можно звать из любой задачи, не из прерывания.
void events_add(char level, const char *code, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

// Номер последнего события, 0 если их не было.
uint32_t events_last(void);

// Событие с номером num; false, если его нет или оно уже вытеснено из журнала.
bool events_get(uint32_t num, event_t *out);

#ifdef __cplusplus
}
#endif
