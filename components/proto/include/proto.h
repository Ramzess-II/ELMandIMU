// Строки протокола версии 1 (ТЗ, раздел 9): "$тело*CRC\r\n", CRC-16/CCITT-FALSE тела.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_VERSION  1
#define PROTO_LINE_MAX 256

uint16_t proto_crc16(const char *s, size_t n);

// Собирает строку: тело по формату, затем "*CRC\r\n". Длина строки, 0 если не влезла.
int proto_line(char *buf, size_t size, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

// Заменяет в текстовом поле запятые, '*' и непечатаемые символы на пробел (PR-2).
void proto_clean_text(char *s);

// Проверяет строку и режет её на поля на месте. fields[0] — тип строки ("NGC").
// false, если нет '$', '*', CRC не сходится или полей больше max.
bool proto_parse(char *line, char **fields, int max, int *n);

#ifdef __cplusplus
}
#endif
