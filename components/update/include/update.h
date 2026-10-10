// Обновление прошивки из приложения. Загрузчик штатный, ESP-IDF: два раздела под прошивку, новая
// пишется в неработающий. Перед переключением проверяются размер, SHA-256 переданного файла и подпись.
// Прошивка, пришедшая обновлением, сначала считается непроверенной. Проверенной она становится, когда
// к блоку подключился телефон и гироскоп найден (если был найден до обновления). Не подтвердилась за
// 10 минут или блок перезагрузился раньше — загрузчик возвращает прежнюю.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// На столько байт телефон может уйти вперёд от подтверждённого «принято».
#define UPDATE_WINDOW     4096
// Наибольший кусок данных: MTU 247 минус 3 байта заголовка записи и 4 байта смещения.
#define UPDATE_CHUNK_MAX  240

typedef struct {
    const char *version;     // версия работающей прошивки
    char        build[48];   // дата сборки, 2026-10-08T20:15
    char        slot[20];    // работающий раздел: ota_0 или ota_1
    const char *slot_state;  // VALID, PENDING (ещё не подтверждена) или UNDEFINED
    uint32_t    max_image;   // наибольший размер образа, байт; 0 — второго раздела нет
    bool        can_update;  // прошивка собрана с проверкой подписи и умеет принимать обновление
} update_info_t;

typedef enum {
    UPDATE_IDLE,
    UPDATE_RECV,      // идёт приём
    UPDATE_VERIFY,    // всё принято, проверяются SHA-256 и подпись
    UPDATE_DONE,      // раздел выбран, нужна перезагрузка
    UPDATE_ABORTED,   // отменено командой
    UPDATE_ERROR,     // код ошибки — в err
} update_state_t;

typedef struct {
    update_state_t state;
    uint32_t received;   // сколько байт принято подряд с начала и записано
    uint32_t size;
    char     err[12];    // MOVING, SIZE, WRITE, HASH, SIGNATURE, TIMEOUT, CHIP, LINK …
    bool     resend;     // пришёл кусок дальше ожидаемого места: телефону надо вернуться к received
} update_progress_t;

// NVS, config и events уже запущены.
void update_init(void);
void update_get_info(update_info_t *out);
bool update_supported(void);

// Звать периодически. phone — телефон на связи (HELLO по любому транспорту), imu_ok — гироскоп найден.
// Подтверждает непроверенную прошивку или через 10 минут просит загрузчик вернуть прежнюю.
void update_tick(bool phone, bool imu_ok);

// true, пока прошивка не подтверждена или идёт обновление: на стоянку и в сон уходить нельзя.
bool update_hold_awake(void);

// ---- приём новой прошивки ----

// Начать приём. size и sha256_hex — от файла .bin как он есть, вместе с блоком подписи.
// NULL — принято; иначе код ошибки: BUSY, SIZE, VALUE, PENDING, UNSUPPORTED, WRITE.
const char *update_begin(uint32_t size, const char *sha256_hex, const char *version);

// Кусок из характеристики OTA: смещение (uint32, младший байт первым) и данные. Зовётся из задачи
// стека Bluetooth, только складывает кусок в очередь.
void update_chunk(const uint8_t *msg, size_t len);

// Всё передано: проверить и выбрать раздел. Итог придёт в update_get_progress (DONE или ERROR).
// imu_ok запоминается: новая прошивка должна будет найти гироскоп, если он найден сейчас.
// NULL — проверка началась; иначе код ошибки STATE.
const char *update_end(bool imu_ok);

// Отменить. err == NULL — по команде телефона (ABORTED), иначе ошибка с этим кодом.
void update_abort(const char *err);

// Состояние приёма. Признак resend при чтении сбрасывается.
void update_get_progress(update_progress_t *out);
bool update_active(void);

#ifdef __cplusplus
}
#endif
