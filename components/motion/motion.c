// Неподвижность (6.1), курс (6.2), путь (6.3), автообнуление (7.1), калибровка по команде (7.2, 7.3),
// сдвиг блока (7.5). Состояние ниже без блокировки принадлежит imu_task; наружу — только через s_lock.
#include "motion.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "calib.h"
#include "config.h"
#include "events.h"

#define TAG "motion"

#define RAD2DEG (180.0 / M_PI)
#define DEG2RAD (M_PI / 180.0)

#define BLOCK_US        100000      // окно неподвижности собирается блоками по 0.1 с
#define NBLOCKS         20          // окно 2 с
#define MAX_DT_US       50000       // больший разрыв между отсчётами не интегрируется
#define SPEED_FRESH_US  1000000
#define STILL_RATE_NO_OBD 0.3f      // 6.1 п. 4: без скорости порог поворота строже
#define CONFIRM_DPS     0.3f        // CB-2
#define SHORT_STOP_US   15000000    // CB-3
#define BIAS_TEMP_DIFF  15.0f       // CB-5
#define CMD_US          3000000     // 7.2, 7.3
#define PROGRESS_US     500000
#define ACCEL_MIN       9.3f        // 7.3 п. 3
#define ACCEL_MAX       10.3f
#define MOUNT_DEG       8.0f        // 7.5
#define MOUNT_STOPS     3
#define CMD_STALL_US    1000000     // команда без отсчётов IMU — ERR,IMU
// Калибровка по команде: вибрация допустима, она в среднем ноль. Движение видно по тому, что средние
// за части замера расходятся: ускорение меняет направление (разгон, поворот), гироскоп — среднее.
#define CMD_PARTS       6           // по 0.5 с
#define CMD_ACC_DEG     3.0f
#define CMD_GYRO_DPS    0.5f

typedef struct {
    int    n;
    double a, a2;           // модуль ускорения
    double g[3], g2[3];     // гироскоп
    double acc[3];          // ускорение
} stats_t;

typedef enum { AZ_IDLE, AZ_M1, AZ_M2, AZ_REFINE } az_state_t;

// ---- состояние imu_task ----
static calib_data_t s_cal;
static bool    s_temp_checked;
static stats_t s_block;
static int64_t s_block_t0;
static stats_t s_ring[NBLOCKS];
static int     s_ring_pos, s_ring_n;
static float   s_up[3];             // вертикаль для курса: калибровка или ближайшая ось (MO-5)
static bool    s_up_set;
static bool    s_still;
static int64_t s_stop_start_us;
static az_state_t s_az;
static int     s_az_blocks;
static float   s_m1[3];
static float   s_anchor[3];         // подтверждённое смещение этой стоянки
static bool    s_refined;
static int     s_mount_count;
static bool    s_mount_moved;
static double  s_yaw_deg;
static float   s_prev_rate;
static int64_t s_prev_t;
static double  s_dist_mm;
static double  s_rate_sum;
static int     s_rate_n;
static int64_t s_bias_t_us = -1;
static float   s_shake_gyro, s_shake_acc;

static struct {
    bool         active;
    motion_cmd_t cmd;
    int          seq;
    int64_t      t0, next_progress;
    stats_t      st;
    stats_t      part[CMD_PARTS];
} s_cmd;

// ---- общее, под s_lock ----
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
    bool         pending;
    bool         abort;
    motion_cmd_t cmd;
    int          seq;
} s_req;
static bool    s_busy;              // команда принята и ещё не получила окончательный ответ
static int     s_busy_seq;
static int64_t s_busy_since_us;
static int     s_speed_kmh = -1;
static int64_t s_speed_t_us;
static int64_t s_speed_nonzero_t_us;
static struct {
    double       yaw_deg;
    double       rate_sum;
    int          rate_n;
    double       dist_mm;
    int64_t      t_us;
    uint32_t     flags;
    imu_sample_t last;
    bool         have_last;
    int64_t      bias_t_us;
    float        bias_up_dps;
    float        shake_gyro, shake_acc;
} s_pub = {.bias_t_us = -1};
static calib_data_t s_save_copy;

static QueueHandle_t s_replies;
static TaskHandle_t  s_save_task;

// ---- математика ----

static float dot3(const float *a, const float *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static bool normalize3(float *v)
{
    float n = sqrtf(dot3(v, v));
    if (n < 1e-6f) {
        return false;
    }
    for (int k = 0; k < 3; k++) {
        v[k] /= n;
    }
    return true;
}

static void stats_add(stats_t *s, const imu_sample_t *x)
{
    double a = sqrt((double)x->accel[0] * x->accel[0] + (double)x->accel[1] * x->accel[1] +
                    (double)x->accel[2] * x->accel[2]);
    s->a += a;
    s->a2 += a * a;
    for (int k = 0; k < 3; k++) {
        s->g[k] += x->gyro[k];
        s->g2[k] += (double)x->gyro[k] * x->gyro[k];
        s->acc[k] += x->accel[k];
    }
    s->n++;
}

static void stats_merge(stats_t *d, const stats_t *s)
{
    d->n += s->n;
    d->a += s->a;
    d->a2 += s->a2;
    for (int k = 0; k < 3; k++) {
        d->g[k] += s->g[k];
        d->g2[k] += s->g2[k];
        d->acc[k] += s->acc[k];
    }
}

static double sd(double sum, double sum2, int n)
{
    double m = sum / n;
    return sqrt(fmax(sum2 / n - m * m, 0));
}

static void mean_g(const stats_t *s, float out[3])
{
    for (int k = 0; k < 3; k++) {
        out[k] = s->g[k] / s->n;
    }
}

static void mean_acc(const stats_t *s, float out[3])
{
    for (int k = 0; k < 3; k++) {
        out[k] = s->acc[k] / s->n;
    }
}

// Разброс без поворота: гироскоп по всем осям и модуль ускорения (6.1 п. 1, 2).
static bool quiet(const stats_t *s)
{
    if (s->n < 2 || sd(s->a, s->a2, s->n) >= config_get_float("still_acc")) {
        return false;
    }
    double lim = config_get_float("still_gyro") * DEG2RAD;
    for (int k = 0; k < 3; k++) {
        if (sd(s->g[k], s->g2[k], s->n) >= lim) {
            return false;
        }
    }
    return true;
}

// Проекция на вертикаль в °/с.
static float up_dps(const float *v)
{
    return dot3(v, s_up) * RAD2DEG;
}

// ---- сохранение и ответы ----

// В NVS пишется только то, что сделано командой: калибровка вертикали и обнуление по кнопке.
// Автообнуление на стоянках живёт в памяти до выключения: после перезагрузки блок стартует со
// смещением из последней калибровки и уточняет его на первой же стоянке.
static void request_save(void)
{
    taskENTER_CRITICAL(&s_lock);
    s_save_copy = s_cal;
    taskEXIT_CRITICAL(&s_lock);
    if (s_save_task) {
        xTaskNotifyGive(s_save_task);
    }
}

static void save_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        calib_data_t c;
        taskENTER_CRITICAL(&s_lock);
        c = s_save_copy;
        taskEXIT_CRITICAL(&s_lock);
        calib_save(&c);
    }
}

static void reply(int seq, int progress, const char *err)
{
    motion_reply_t r = {.seq = seq, .progress = progress, .ok = err == NULL};
    if (err) {
        strlcpy(r.err, err, sizeof(r.err));
    }
    if (progress < 0) {
        taskENTER_CRITICAL(&s_lock);
        if (s_busy && s_busy_seq == seq) {
            s_busy = false;
        }
        taskEXIT_CRITICAL(&s_lock);
    }
    xQueueSend(s_replies, &r, 0);
}

// ---- смещение нуля ----

static void set_bias(const float b[3], const imu_sample_t *x, int64_t now)
{
    memcpy(s_cal.bias, b, sizeof(s_cal.bias));
    s_cal.bias_ok = true;
    s_cal.bias_has_temp = x->has_temp;
    s_cal.bias_temp_c = x->temp_c;
    s_bias_t_us = now;
}

static void on_stop_start(const stats_t *w, int64_t now)
{
    s_stop_start_us = now - (int64_t)NBLOCKS * BLOCK_US;
    mean_g(w, s_m1);
    s_az = AZ_M2;
    s_az_blocks = 0;
    s_refined = false;

    // 7.5: блок сдвинули, если вертикаль на трёх стоянках подряд уходит больше чем на 8°.
    if (s_cal.up_ok) {
        float a[3];
        mean_acc(w, a);
        if (normalize3(a)) {
            float c = fminf(fmaxf(dot3(a, s_cal.up), -1), 1);
            float ang = acosf(c) * RAD2DEG;
            if (ang > MOUNT_DEG) {
                if (++s_mount_count >= MOUNT_STOPS && !s_mount_moved) {
                    s_mount_moved = true;
                    events_add('E', "MOUNT_MOVED", "%.1f deg", ang);
                }
            } else {
                s_mount_count = 0;
            }
        }
    }
}

static void on_stop_end(void)
{
    s_az = AZ_IDLE;
}

static void autozero_block(const stats_t *w, const imu_sample_t *x, int64_t now)
{
    float m[3];
    mean_g(w, m);
    switch (s_az) {
    case AZ_IDLE:
        break;

    case AZ_M1:
        if (++s_az_blocks >= NBLOCKS) {
            memcpy(s_m1, m, sizeof(s_m1));
            s_az = AZ_M2;
            s_az_blocks = 0;
        }
        break;

    case AZ_M2: {
        if (++s_az_blocks < NBLOCKS) {
            break;
        }
        s_az_blocks = 0;
        // CB-2: второй замер той же стоянки подтверждает первый.
        float d[3] = {m[0] - s_m1[0], m[1] - s_m1[1], m[2] - s_m1[2]};
        float diff = fabsf(up_dps(d));
        if (diff > CONFIRM_DPS) {
            events_add('W', "CAL_AUTO_REJECTED", "no confirm %.3f dps", diff);
            s_az = AZ_M1;
            break;
        }
        float cand[3];
        for (int k = 0; k < 3; k++) {
            cand[k] = 0.5f * (m[k] + s_m1[k]);
        }
        // CB-3: на короткой стоянке большой скачок — скорее медленный поворот.
        float j[3] = {cand[0] - s_cal.bias[0], cand[1] - s_cal.bias[1], cand[2] - s_cal.bias[2]};
        float jump = fabsf(up_dps(j));
        int64_t stop_us = now - s_stop_start_us;
        if (s_cal.bias_ok && stop_us < SHORT_STOP_US && jump > config_get_float("bias_max_jump")) {
            events_add('W', "CAL_AUTO_REJECTED", "jump %.3f dps stop %d s", jump, (int)(stop_us / 1000000));
            memcpy(s_m1, m, sizeof(s_m1));
            break;
        }
        set_bias(cand, x, now);
        memcpy(s_anchor, cand, sizeof(s_anchor));
        events_add('I', "CAL_AUTO", "%d", (int)lroundf(up_dps(cand) * 1000));
        s_az = AZ_REFINE;
        break;
    }

    case AZ_REFINE: {
        // CB-4: скользящее среднее 2 с, но не дальше 0.3 °/с от подтверждённого — иначе это поворот.
        float d[3] = {m[0] - s_anchor[0], m[1] - s_anchor[1], m[2] - s_anchor[2]};
        if (fabsf(up_dps(d)) < CONFIRM_DPS) {
            set_bias(m, x, now);
            s_refined = true;
        }
        break;
    }
    }
}

// ---- неподвижность ----

static bool eval_still(const stats_t *w, int64_t now, int speed, int64_t speed_t, int64_t nonzero_t)
{
    if (!quiet(w)) {
        return false;
    }
    bool have_speed = speed >= 0 && now - speed_t < SPEED_FRESH_US;
    float lim = config_get_float("still_rate");
    if (!have_speed) {
        lim = fminf(lim, STILL_RATE_NO_OBD);
    }
    // 6.1 п. 3: средний поворот за вычетом смещения. Пока смещение неизвестно — пропускается.
    if (s_cal.bias_ok) {
        float m[3];
        mean_g(w, m);
        const float *ref = s_az == AZ_REFINE ? s_anchor : s_cal.bias;
        float d[3] = {m[0] - ref[0], m[1] - ref[1], m[2] - ref[2]};
        if (fabsf(up_dps(d)) >= lim) {
            return false;
        }
    }
    // 6.1 п. 4: скорость 0 всё окно.
    if (have_speed && (speed != 0 || now - nonzero_t < (int64_t)NBLOCKS * BLOCK_US)) {
        return false;
    }
    return true;
}

static void update_up(const stats_t *w)
{
    if (s_cal.up_ok) {
        memcpy(s_up, s_cal.up, sizeof(s_up));
        s_up_set = true;
        return;
    }
    // MO-5: без калибровки — ось датчика, ближайшая к вертикали.
    float a[3];
    mean_acc(w, a);
    int best = 0;
    for (int k = 1; k < 3; k++) {
        if (fabsf(a[k]) > fabsf(a[best])) {
            best = k;
        }
    }
    s_up[0] = s_up[1] = s_up[2] = 0;
    s_up[best] = a[best] >= 0 ? 1 : -1;
    s_up_set = true;
}

static void block_done(const imu_sample_t *x, int speed, int64_t speed_t, int64_t nonzero_t)
{
    int64_t now = x->t_us;
    s_ring[s_ring_pos] = s_block;
    s_ring_pos = (s_ring_pos + 1) % NBLOCKS;
    if (s_ring_n < NBLOCKS) {
        s_ring_n++;
    }
    stats_t w = {0};
    for (int i = 0; i < s_ring_n; i++) {
        stats_merge(&w, &s_ring[i]);
    }
    update_up(&w);
    if (s_ring_n < NBLOCKS) {
        return;
    }
    s_shake_acc = sd(w.a, w.a2, w.n);
    s_shake_gyro = 0;
    for (int k = 0; k < 3; k++) {
        s_shake_gyro = fmaxf(s_shake_gyro, sd(w.g[k], w.g2[k], w.n) * RAD2DEG);
    }

    bool still = eval_still(&w, now, speed, speed_t, nonzero_t);
    if (still && !s_still) {
        s_still = true;
        on_stop_start(&w, now);
    } else if (!still && s_still) {
        s_still = false;
        on_stop_end();
    } else if (still) {
        autozero_block(&w, x, now);
    }
}

// ---- калибровка по команде ----

static void cmd_start(motion_cmd_t cmd, int seq)
{
    if (cmd == MOTION_CMD_CAL_RESET) {
        s_cal.up_ok = false;
        s_cal.fwd_ok = false;
        s_mount_moved = false;
        s_mount_count = 0;
        request_save();
        reply(seq, -1, NULL);
        return;
    }
    if (cmd == MOTION_CMD_CAL_UP2 && !s_cal.up_ok) {
        reply(seq, -1, "NO_UP");
        return;
    }
    memset(&s_cmd, 0, sizeof(s_cmd));
    s_cmd.active = true;
    s_cmd.cmd = cmd;
    s_cmd.seq = seq;
}

// Блок не двигался за время замера: средние каждой части близки к общему.
static bool cmd_steady(void)
{
    float a[3], g[3];
    mean_acc(&s_cmd.st, a);
    mean_g(&s_cmd.st, g);
    if (!normalize3(a)) {
        return false;
    }
    for (int i = 0; i < CMD_PARTS; i++) {
        const stats_t *p = &s_cmd.part[i];
        if (p->n < 10) {
            return false;
        }
        float pa[3], pg[3];
        mean_acc(p, pa);
        mean_g(p, pg);
        if (!normalize3(pa) || acosf(fminf(dot3(pa, a), 1)) * RAD2DEG > CMD_ACC_DEG) {
            return false;
        }
        for (int k = 0; k < 3; k++) {
            if (fabsf(pg[k] - g[k]) * RAD2DEG > CMD_GYRO_DPS) {
                return false;
            }
        }
    }
    return true;
}

static void cmd_finish(const imu_sample_t *x)
{
    s_cmd.active = false;
    const stats_t *st = &s_cmd.st;
    bool up = s_cmd.cmd != MOTION_CMD_CAL_BIAS;

    if (!cmd_steady()) {
        if (up) {
            events_add('W', "CAL_UP_FAILED", "MOVING");
        }
        reply(s_cmd.seq, -1, "MOVING");
        return;
    }
    float g[3], a[3];
    mean_g(st, g);
    mean_acc(st, a);

    if (up) {
        float an = sqrtf(dot3(a, a));
        if (an < ACCEL_MIN || an > ACCEL_MAX) {
            events_add('W', "CAL_UP_FAILED", "ACCEL_SCALE %.2f", an);
            reply(s_cmd.seq, -1, "ACCEL_SCALE");
            return;
        }
        normalize3(a);
        if (s_cmd.cmd == MOTION_CMD_CAL_UP) {
            memcpy(s_cal.up1, a, sizeof(a));
            memcpy(s_cal.up, a, sizeof(a));
        } else {
            // 7.3: после разворота на 180° наклон дороги меняет знак и вычитается.
            float u[3] = {s_cal.up1[0] + a[0], s_cal.up1[1] + a[1], s_cal.up1[2] + a[2]};
            if (!normalize3(u)) {
                reply(s_cmd.seq, -1, "NO_UP");
                return;
            }
            memcpy(s_cal.up, u, sizeof(u));
        }
        s_cal.up_ok = true;
        s_mount_moved = false;
        s_mount_count = 0;
        memcpy(s_up, s_cal.up, sizeof(s_up));
        events_add('I', "CAL_UP_OK", "%s up %.3f %.3f %.3f",
                   s_cmd.cmd == MOTION_CMD_CAL_UP ? "1" : "2", s_cal.up[0], s_cal.up[1], s_cal.up[2]);
    }
    // Ограничение скачка CB-3 к команде не применяется.
    set_bias(g, x, x->t_us);
    s_az = s_still ? AZ_M1 : AZ_IDLE;
    s_az_blocks = 0;
    request_save();
    reply(s_cmd.seq, -1, NULL);
}

static void cmd_add(const imu_sample_t *x)
{
    if (s_cmd.t0 == 0) {
        s_cmd.t0 = x->t_us;
        s_cmd.next_progress = x->t_us + PROGRESS_US;
    }
    stats_add(&s_cmd.st, x);
    int64_t el = x->t_us - s_cmd.t0;
    int part = (int)(el * CMD_PARTS / CMD_US);
    stats_add(&s_cmd.part[part < CMD_PARTS ? part : CMD_PARTS - 1], x);
    if (el >= CMD_US) {
        cmd_finish(x);
    } else if (x->t_us >= s_cmd.next_progress) {
        s_cmd.next_progress += PROGRESS_US;
        reply(s_cmd.seq, (int)(el * 100 / CMD_US), NULL);
    }
}

// ---- обработка отсчётов ----

void motion_on_samples(const imu_sample_t *s, int n, void *ctx)
{
    if (n <= 0) {
        return;
    }
    bool start = false, abort = false;
    motion_cmd_t cmd = 0;
    int seq = 0, speed;
    int64_t speed_t, nonzero_t;
    taskENTER_CRITICAL(&s_lock);
    if (s_req.pending) {
        start = true;
        cmd = s_req.cmd;
        seq = s_req.seq;
        s_req.pending = false;
    }
    abort = s_req.abort;
    s_req.abort = false;
    speed = s_speed_kmh;
    speed_t = s_speed_t_us;
    nonzero_t = s_speed_nonzero_t_us;
    taskEXIT_CRITICAL(&s_lock);

    if (abort) {
        s_cmd.active = false;
    }
    if (start) {
        cmd_start(cmd, seq);
    }

    // CB-5: смещение из NVS не годится, если температура ушла больше чем на 15 °C.
    if (!s_temp_checked) {
        s_temp_checked = true;
        if (s_cal.bias_ok && s_cal.bias_has_temp && s[0].has_temp &&
            fabsf(s[0].temp_c - s_cal.bias_temp_c) > BIAS_TEMP_DIFF) {
            s_cal.bias_ok = false;
            ESP_LOGW(TAG, "смещение из NVS снято при %.1f °C, сейчас %.1f °C — жду замера",
                     s_cal.bias_temp_c, s[0].temp_c);
        }
    }

    for (int i = 0; i < n; i++) {
        const imu_sample_t *x = &s[i];
        if (!s_up_set) {
            stats_t one = {0};
            stats_add(&one, x);
            update_up(&one);
        }
        if (s_cmd.active) {
            cmd_add(x);
        }

        // MO-1: поворот машины, направо плюс. MO-3: на стоянке не копится.
        float rate = 0;
        if (!s_still) {
            for (int k = 0; k < 3; k++) {
                rate -= (x->gyro[k] - s_cal.bias[k]) * s_up[k];
            }
        }
        int64_t dt = x->t_us - s_prev_t;
        if (s_prev_t && dt > 0 && dt < MAX_DT_US) {
            // MO-2: трапеция.
            s_yaw_deg += 0.5 * (rate + s_prev_rate) * dt * 1e-6 * RAD2DEG;
            // MO-6: путь по последней скорости, если она свежая.
            if (speed > 0 && x->t_us - speed_t < SPEED_FRESH_US) {
                s_dist_mm += speed / 3.6 * dt * 1e-3;
            }
        }
        s_prev_rate = rate;
        s_prev_t = x->t_us;
        s_rate_sum += rate * RAD2DEG;
        s_rate_n++;

        if (s_block.n == 0) {
            s_block_t0 = x->t_us;
        }
        stats_add(&s_block, x);
        if (x->t_us - s_block_t0 >= BLOCK_US) {
            block_done(x, speed, speed_t, nonzero_t);
            memset(&s_block, 0, sizeof(s_block));
        }
    }

    uint32_t flags = 0;
    if (s_cal.bias_ok)  flags |= MOTION_BIAS_OK;
    if (s_cal.up_ok)    flags |= MOTION_UP_OK;
    if (s_cal.fwd_ok)   flags |= MOTION_FWD_OK;
    if (s_still)        flags |= MOTION_STILL;
    if (s_cmd.active)   flags |= MOTION_CALIBRATING;
    if (s_mount_moved)  flags |= MOTION_MOUNT_MOVED;

    taskENTER_CRITICAL(&s_lock);
    s_pub.yaw_deg = s_yaw_deg;
    s_pub.rate_sum += s_rate_sum;
    s_pub.rate_n += s_rate_n;
    s_pub.dist_mm = s_dist_mm;
    s_pub.t_us = s[n - 1].t_us;
    s_pub.flags = flags;
    s_pub.last = s[n - 1];
    s_pub.have_last = true;
    s_pub.bias_t_us = s_bias_t_us;
    s_pub.bias_up_dps = up_dps(s_cal.bias);
    s_pub.shake_gyro = s_shake_gyro;
    s_pub.shake_acc = s_shake_acc;
    taskEXIT_CRITICAL(&s_lock);
    s_rate_sum = 0;
    s_rate_n = 0;
}

// ---- снаружи ----

esp_err_t motion_init(void)
{
    calib_load(&s_cal);
    if (s_cal.up_ok) {
        memcpy(s_up, s_cal.up, sizeof(s_up));
        s_up_set = true;
    }
    ESP_LOGI(TAG, "калибровка из NVS: вертикаль %s, смещение %s",
             s_cal.up_ok ? "есть" : "нет", s_cal.bias_ok ? "есть" : "нет");
    s_replies = xQueueCreate(16, sizeof(motion_reply_t));
    if (!s_replies) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(save_task, "calib_save", 3072, NULL, 3, &s_save_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void motion_set_speed(int kmh, int64_t t_us)
{
    taskENTER_CRITICAL(&s_lock);
    s_speed_kmh = kmh;
    s_speed_t_us = t_us;
    if (kmh != 0) {
        s_speed_nonzero_t_us = t_us;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void motion_take_packet(motion_packet_t *out)
{
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    double rate = s_pub.rate_n ? s_pub.rate_sum / s_pub.rate_n : 0;
    s_pub.rate_sum = 0;
    s_pub.rate_n = 0;
    out->yaw_mdeg = llround(s_pub.yaw_deg * 1000);
    out->rate_mdps = (int32_t)lround(rate * 1000);
    out->dist_mm = (uint64_t)s_pub.dist_mm;
    // Пока IMU молчит, время идёт от часов блока, чтобы приложение видело живой поток.
    int64_t t = s_pub.have_last && now - s_pub.t_us < 200000 ? s_pub.t_us : now;
    out->t_ms = (uint32_t)(t / 1000);
    out->flags = s_pub.flags;
    if (s_speed_kmh >= 0 && s_speed_t_us > 0) {
        int64_t age = now - s_speed_t_us;
        out->speed_kmh = s_speed_kmh;
        out->speed_age_ms = (int)(age / 1000);
        if (age < SPEED_FRESH_US) {
            out->flags |= MOTION_OBD_OK;
        }
    } else {
        out->speed_kmh = -1;
        out->speed_age_ms = -1;
    }
    taskEXIT_CRITICAL(&s_lock);
}

void motion_get_status(motion_status_t *out)
{
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_lock);
    out->bias_age_s = s_pub.bias_t_us < 0 ? -1 : (int)((now - s_pub.bias_t_us) / 1000000);
    out->bias_up_mdps = (int)lroundf(s_pub.bias_up_dps * 1000);
    out->has_temp = s_pub.have_last && s_pub.last.has_temp;
    out->temp_c = s_pub.last.temp_c;
    out->flags = s_pub.flags;
    out->shake_gyro_dps = s_pub.shake_gyro;
    out->shake_acc = s_pub.shake_acc;
    taskEXIT_CRITICAL(&s_lock);
}

const char *motion_command(motion_cmd_t cmd, int seq)
{
    const char *err = NULL;
    taskENTER_CRITICAL(&s_lock);
    if (s_busy) {
        err = "BUSY";
    } else {
        s_busy = true;
        s_busy_seq = seq;
        s_busy_since_us = esp_timer_get_time();
        s_req.pending = true;
        s_req.cmd = cmd;
        s_req.seq = seq;
    }
    taskEXIT_CRITICAL(&s_lock);
    return err;
}

bool motion_get_reply(motion_reply_t *out)
{
    return s_replies && xQueueReceive(s_replies, out, 0) == pdTRUE;
}

void motion_tick(void)
{
    int64_t now = esp_timer_get_time();
    int seq = 0;
    bool stalled = false;
    taskENTER_CRITICAL(&s_lock);
    int64_t last = s_pub.have_last ? s_pub.t_us : 0;
    if (s_busy && now - s_busy_since_us > CMD_STALL_US && now - last > CMD_STALL_US) {
        stalled = true;
        seq = s_busy_seq;
        s_req.pending = false;
        s_req.abort = true;
    }
    taskEXIT_CRITICAL(&s_lock);
    if (stalled) {
        reply(seq, -1, "IMU");
    }
}

bool motion_last_sample(imu_sample_t *out)
{
    taskENTER_CRITICAL(&s_lock);
    bool ok = s_pub.have_last;
    *out = s_pub.last;
    taskEXIT_CRITICAL(&s_lock);
    return ok;
}
