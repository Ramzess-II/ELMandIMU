#include "proto.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint16_t proto_crc16(const char *s, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)(uint8_t)s[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

int proto_line(char *buf, size_t size, const char *fmt, ...)
{
    if (size < 8) {
        return 0;
    }
    buf[0] = '$';
    va_list ap;
    va_start(ap, fmt);
    int body = vsnprintf(buf + 1, size - 1, fmt, ap);
    va_end(ap);
    // '$' + тело + "*XXXX\r\n" + '\0'
    if (body < 0 || (size_t)body + 1 + 7 + 1 > size) {
        return 0;
    }
    uint16_t crc = proto_crc16(buf + 1, body);
    int len = 1 + body;
    len += snprintf(buf + len, size - len, "*%04X\r\n", crc);
    return len;
}

void proto_clean_text(char *s)
{
    for (; *s; s++) {
        if (*s == ',' || *s == '*' || *s == '$' || !isprint((unsigned char)*s)) {
            *s = ' ';
        }
    }
}

bool proto_parse(char *line, char **fields, int max, int *n)
{
    // Пробелы и \r\n по краям.
    while (*line && isspace((unsigned char)*line)) {
        line++;
    }
    size_t len = strlen(line);
    while (len && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }
    if (len < 6 || line[0] != '$') {
        return false;
    }
    char *star = strrchr(line, '*');
    if (!star || star + 5 != line + len) {
        return false;
    }
    char *end;
    unsigned long crc = strtoul(star + 1, &end, 16);
    if (end != line + len || crc != proto_crc16(line + 1, star - line - 1)) {
        return false;
    }
    *star = '\0';
    int k = 0;
    char *p = line + 1;
    for (;;) {
        if (k == max) {
            return false;
        }
        fields[k++] = p;
        char *comma = strchr(p, ',');
        if (!comma) {
            break;
        }
        *comma = '\0';
        p = comma + 1;
    }
    *n = k;
    return true;
}
