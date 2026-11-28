/**
 * @file    encoder.c
 * @brief   AMT132S-V encoder driver. See encoder.h for the API and README.md for setup.
 * @author  Jordan Rivero (Bionix MCU team)
 * @version 2.0
 *
 * How it works:
 *  - The encoder timer counts every A/B edge in hardware, so its counter is the
 *    position. Encoder_Update() reads it ENC_SAMPLE_HZ times per second and adds
 *    the change to a 64-bit total.
 *  - Speed: window method (counts per sample) or, with ENC_USE_CAPTURE, the M/T
 *    method (exact count of captured edges / exact time between them).
 *  - Acceleration: an alpha-beta-gamma tracking filter. It predicts the next
 *    position from its own position/speed/acceleration estimates, then corrects
 *    all three by the prediction error. Differentiating the speed twice instead
 *    would amplify count jitter into huge acceleration noise.
 *  - Z index: every index pulse must land a whole number of turns (multiple of
 *    8192 counts) from the first one. If it doesn't, counts were missed or added.
 */
#include "encoder.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ========================================================================== */
/*  Constants                                                                 */
/* ========================================================================== */

#define DEG_PER_REV         360.0f
#define SEC_PER_MIN         60.0f
#define MS_PER_S            1000u
#define US_PER_S            1000000.0f
#define DEG_S_PER_RPM       (DEG_PER_REV / SEC_PER_MIN)        /* 1 RPM = 6 deg/s */
#define DEG_PER_COUNT       (DEG_PER_REV / (float)ENC_CPR)     /* 0.0439 deg */
#define DEG_PER_COUNT_D     (360.0 / (double)ENC_CPR)          /* same, double precision */
#define SAMPLE_PERIOD_S     (1.0f / (float)ENC_SAMPLE_HZ)
#define RUNNING_CHECK_MS    100u

/* RPM per count of change in one sample period. */
#define K_WINDOW            (SEC_PER_MIN * (float)ENC_SAMPLE_HZ / (float)ENC_CPR)

/* RPM = n_captures * K_EDGE / ticks. One capture spans ENC_IC_DIV periods of A. */
#define K_EDGE              (SEC_PER_MIN * (float)ENC_TICK_HZ * (float)ENC_IC_DIV / (float)ENC_PPR)

/* Stop timeout in capture-timer ticks. */
#define STOP_TICKS          ((uint32_t)ENC_STOP_TIMEOUT_MS * (ENC_TICK_HZ / MS_PER_S))

/* Values of Encoder.zero_request. */
#define ZERO_NONE           0u
#define ZERO_NOW            1u
#define ZERO_AT_INDEX       2u

/* Index into Encoder.index_ref[]: the Z edge lands on a different count
 * depending on which way the shaft turns, so each direction has its own reference. */
#define DIR_FWD             0u
#define DIR_REV             1u

/* Counter Period (ARR) the encoder timer must have for counter_delta() to work. */
#if ENC_TIMER_IS_32BIT
#define ENC_TIMER_MAX       0xFFFFFFFFu
#else
#define ENC_TIMER_MAX       0xFFFFu
#endif

/* Longest int64 as text: sign + 19 digits + terminator. */
#define I64_STR_LEN         21u
/* Fixed-point number as text, e.g. "-123456789012345.67". */
#define FIX2_STR_LEN        32u

/* ========================================================================== */
/*  Helpers                                                                   */
/* ========================================================================== */

/* Block interrupts while reading or writing values another interrupt may change
 * halfway through (e.g. the 64-bit position takes two 32-bit accesses). */
static inline uint32_t enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void exit_critical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* Signed change of the hardware counter between two reads. Unsigned subtraction
 * wraps exactly like the counter does, so casting to signed gives the right
 * answer across rollover (e.g. 0 -> 65535 on a 16-bit timer reads as -1).
 * Valid while the counter moves less than half its range per sample:
 * 16-bit = 32767 counts, and 8000 RPM for 10 ms is only ~10,900. */
static inline int32_t counter_delta(uint32_t now, uint32_t before)
{
#if ENC_TIMER_IS_32BIT
    return (int32_t)(now - before);
#else
    return (int32_t)(int16_t)(uint16_t)(now - before);
#endif
}

/* Floor division: rounds toward minus infinity, unlike C's '/', which rounds
 * toward zero (-100 / 8192 would give 0 instead of -1). */
static inline int64_t floor_div(int64_t a, int64_t b)
{
    int64_t q = a / b;
    if ((a % b) != 0 && (a < 0)) {
        q -= 1;
    }
    return q;
}

/* int64 to decimal text. printf's %lld is not available in the newlib-nano
 * library CubeIDE uses by default, so we convert by hand. */
static const char *i64_to_str(int64_t v, char buf[I64_STR_LEN])
{
    /* Work with the magnitude as unsigned; written this way so INT64_MIN is safe. */
    uint64_t mag = (v < 0) ? ((uint64_t)(-(v + 1)) + 1u) : (uint64_t)v;
    char *p = &buf[I64_STR_LEN - 1];
    *p = '\0';
    do {
        *--p = (char)('0' + (mag % 10u));
        mag /= 10u;
    } while (mag != 0u);
    if (v < 0) {
        *--p = '-';
    }
    return p;
}

/* Number with two decimals as text, e.g. -12.34. Uses int64 so large values
 * (long runs) never overflow. */
static const char *fix2_to_str(double v, char buf[FIX2_STR_LEN])
{
    char whole_buf[I64_STR_LEN];
    int64_t x100 = (int64_t)(v * 100.0 + ((v >= 0.0) ? 0.5 : -0.5));   /* round */
    int64_t mag  = (x100 < 0) ? -x100 : x100;

    snprintf(buf, FIX2_STR_LEN, "%s%s.%02u",
             (x100 < 0) ? "-" : "",
             i64_to_str(mag / 100, whole_buf),
             (unsigned)(mag % 100));
    return buf;
}

/* snprintf returns how many characters it wanted to write; report only what fit. */
static int clamp_len(int n, size_t len)
{
    if (n < 0) {
        return 0;
    }
    if ((size_t)n >= len) {
        return (int)len - 1;
    }
    return n;
}

/* ========================================================================== */
/*  Init                                                                      */
/* ========================================================================== */

void Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_cap)
{
    /* Interrupts aren't running yet, so clearing the whole struct is safe. */
    memset((void *)enc, 0, sizeof(*enc));
    enc->htim_enc = htim_enc;
    enc->htim_cap = htim_cap;

    /* Tracking filter gains (critically damped "fading memory" alpha-beta-gamma
     * filter). theta in (0,1) sets the smoothing; deriving it from a time
     * constant keeps the response time the same at any sample rate. */
    const float T     = SAMPLE_PERIOD_S;
    const float theta = expf(-T / (ENC_ACCEL_TAU_MS / (float)MS_PER_S));
    const float om    = 1.0f - theta;
    enc->gain_pos = 1.0f - theta * theta * theta;
    enc->gain_vel = 1.5f * om * om * (1.0f + theta) / T;
    enc->gain_acc = 2.0f * (0.5f * om * om * om) / (T * T);

    /* Catch a CubeMX/encoder.h mismatch now, not as wrong positions later. */
    if (__HAL_TIM_GET_AUTORELOAD(htim_enc) != ENC_TIMER_MAX) {
        Error_Handler();
    }

    HAL_Delay(ENC_STARTUP_DELAY_MS);   /* encoder start-up: shaft must be still */

    if (HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(htim_enc, 0);

#if ENC_USE_CAPTURE
    if (htim_cap != NULL) {
        if (HAL_TIM_IC_Start_IT(htim_cap, TIM_CHANNEL_1) != HAL_OK) {
            Error_Handler();
        }
    }
#endif

    /* Last: from here on the interrupts may use this struct. */
    enc->ready = 1;
}

/* ========================================================================== */
/*  Interrupt handlers                                                        */
/* ========================================================================== */

void Encoder_OnCapture(Encoder *enc)
{
#if ENC_USE_CAPTURE
    if (!enc->ready) {
        return;
    }
    /* Tick latched by hardware at the exact edge, so it's exact even if this
     * interrupt runs late. */
    uint32_t t   = HAL_TIM_ReadCapturedValue(enc->htim_cap, TIM_CHANNEL_1);
    int8_t   dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(enc->htim_enc) ? -1 : +1;

    if (enc->cap_have_prev && dir == enc->cap_dir) {
        enc->cap_period = t - enc->cap_time;
    } else {
        /* First capture or a reversal: no valid interval. */
        enc->cap_period = 0;
        if (enc->cap_have_prev) {
            enc->cap_epoch++;   /* tells M/T to restart its measurement */
        }
    }
    enc->cap_index    += dir;
    enc->cap_time      = t;
    enc->cap_dir       = dir;
    enc->cap_have_prev = 1;
    enc->captures++;
#else
    (void)enc;
#endif
}

void Encoder_OnIndex(Encoder *enc)
{
    /* The EXTI interrupt is enabled by MX_GPIO_Init(), before Encoder_Init()
     * runs: ignore pulses until the struct and timer are set up. */
    if (!enc->ready) {
        return;
    }

    const uint32_t d = __HAL_TIM_IS_TIM_COUNTING_DOWN(enc->htim_enc) ? DIR_REV : DIR_FWD;
    const uint32_t other = (d == DIR_FWD) ? DIR_REV : DIR_FWD;

    uint32_t p = enter_critical();

    /* Exact position right now = total at the last sample + change since then. */
    uint32_t cnt = __HAL_TIM_GET_COUNTER(enc->htim_enc);
    int64_t  pos = enc->position + counter_delta(cnt, enc->last_cnt);

    if (!enc->index_ref_ok[d]) {
        enc->index_ref[d]    = pos;
        enc->index_ref_ok[d] = 1;
    } else {
        /* Distance from the nearest whole number of turns from the reference. */
        int64_t err = (pos - enc->index_ref[d]) % (int64_t)ENC_CPR;
        if (err > (int64_t)(ENC_CPR / 2u)) {
            err -= ENC_CPR;
        } else if (err < -(int64_t)(ENC_CPR / 2u)) {
            err += ENC_CPR;
        }

        /* The shaft keeps moving between the Z edge and this read, so allow
         * more slack at speed. filt_vel is a float: read atomically. */
        float tol = (float)ENC_INDEX_TOL_COUNTS
                  + fabsf(enc->filt_vel) * (ENC_INDEX_LATENCY_US / US_PER_S);

        if (fabsf((float)err) > tol) {
            enc->index_errors++;
            enc->index_last_err = (int32_t)err;
            enc->index_err_sum += err;
            /* Re-align both references so one glitch is reported once, not on
             * every following turn. */
            enc->index_ref[d]     += err;
            enc->index_ref[other] += err;
        }
    }
    enc->index_count++;

    if (enc->zero_request == ZERO_AT_INDEX) {
        enc->zero_target  = pos;
        enc->zero_pending = 1;
    }

    exit_critical(p);
}

void Encoder_Update(Encoder *enc)
{
    if (!enc->ready) {
        return;
    }

    /* --- Position (and any requested re-zero) ---------------------------- */
    int64_t off    = 0;
    uint8_t do_off = 0;

    uint32_t p = enter_critical();
    uint32_t now_cnt = __HAL_TIM_GET_COUNTER(enc->htim_enc);
    int32_t  d       = counter_delta(now_cnt, enc->last_cnt);
    enc->last_cnt    = now_cnt;
    enc->position   += d;

    /* Re-zeroing happens only here, so nothing else ever writes position. */
    if (enc->zero_request == ZERO_NOW) {
        off = enc->position;
        do_off = 1;
    } else if (enc->zero_request == ZERO_AT_INDEX && enc->zero_pending) {
        off = enc->zero_target;
        do_off = 1;
    }
    if (do_off) {
        enc->position     -= off;
        enc->index_ref[0] -= off;
        enc->index_ref[1] -= off;
        enc->zero_request  = ZERO_NONE;
        enc->zero_pending  = 0;
    }
    int64_t pos = enc->position;
    exit_critical(p);

    enc->delta      = d;
    enc->rpm_window = (float)d * K_WINDOW;

    /* --- Speed ------------------------------------------------------------ */
#if ENC_USE_CAPTURE
    p = enter_critical();
    uint32_t last_t = enc->cap_time;
    uint32_t period = enc->cap_period;
    int8_t   dir    = enc->cap_dir;
    uint8_t  have   = enc->cap_have_prev;
    int32_t  idx    = enc->cap_index;
    uint32_t epoch  = enc->cap_epoch;
    uint32_t now_t  = __HAL_TIM_GET_COUNTER(enc->htim_cap);
    exit_critical(p);

    uint32_t since = now_t - last_t;   /* ticks since the newest captured edge */

    /* Edge timing (comparison only). If we've waited longer than the last
     * interval, the shaft is slowing: use the elapsed time so it decays. */
    if (!have || period == 0 || since > STOP_TICKS) {
        enc->rpm_edge = 0.0f;
    } else {
        uint32_t eff = (since > period) ? since : period;
        enc->rpm_edge = (float)dir * K_EDGE / (float)eff;
    }

    /* M/T: exactly n captures between two captured edges, exactly dt ticks apart. */
    if (!have || since > STOP_TICKS) {
        enc->rpm = 0.0f;
        enc->mt_base_valid = 0;
    } else if (!enc->mt_base_valid || epoch != enc->mt_base_epoch) {
        /* No start edge yet, or the shaft reversed: restart from the newest edge. */
        enc->mt_base_time     = last_t;
        enc->mt_base_index    = idx;
        enc->mt_base_epoch    = epoch;
        enc->mt_base_valid    = 1;
        enc->mt_ticks_per_cap = 0.0f;
        enc->rpm              = 0.0f;
    } else {
        int32_t n = idx - enc->mt_base_index;
        if (n != 0) {
            uint32_t dt = last_t - enc->mt_base_time;
            enc->rpm = (float)n * K_EDGE / (float)dt;
            enc->mt_ticks_per_cap = (float)dt / (float)((n < 0) ? -n : n);
            enc->mt_base_time  = last_t;
            enc->mt_base_index = idx;
        } else if (enc->mt_ticks_per_cap > 0.0f && (float)since > enc->mt_ticks_per_cap) {
            /* No new edge and it's overdue: let the speed decay. */
            enc->rpm = (float)dir * K_EDGE / (float)since;
        }
    }
#else
    enc->rpm = enc->rpm_window;
#endif

    /* --- Acceleration: alpha-beta-gamma tracking filter ------------------- */
    if (do_off) {
        enc->filt_pos -= (double)off;   /* keep the filter in the new frame */
    }
    if (!enc->filt_ready) {
        enc->filt_pos   = (double)pos;
        enc->filt_vel   = 0.0f;
        enc->filt_acc   = 0.0f;
        enc->filt_ready = 1;
    } else {
        const float T = SAMPLE_PERIOD_S;
        /* Predict with constant-acceleration kinematics... */
        double x_pred = enc->filt_pos + (double)(enc->filt_vel * T + 0.5f * enc->filt_acc * T * T);
        float  v_pred = enc->filt_vel + enc->filt_acc * T;
        /* ...then correct all three estimates by the prediction error. */
        float  r      = (float)((double)pos - x_pred);
        enc->filt_pos = x_pred + (double)(enc->gain_pos * r);
        enc->filt_vel = v_pred + enc->gain_vel * r;
        enc->filt_acc = enc->filt_acc + enc->gain_acc * r;
    }
    enc->accel_cps2 = enc->filt_acc;

    enc->samples++;
}

/* ========================================================================== */
/*  Zeroing                                                                   */
/* ========================================================================== */

void Encoder_Zero(Encoder *enc)
{
    enc->zero_request = ZERO_NOW;
}

void Encoder_ZeroAtNextIndex(Encoder *enc)
{
    uint32_t p = enter_critical();
    enc->zero_pending = 0;
    enc->zero_request = ZERO_AT_INDEX;
    exit_critical(p);
}

/* ========================================================================== */
/*  Results                                                                   */
/* ========================================================================== */

int64_t Encoder_GetPosition(Encoder *enc)
{
    uint32_t p = enter_critical();   /* 64-bit read = two accesses */
    int64_t pos = enc->position;
    exit_critical(p);
    return pos;
}

int32_t Encoder_GetTurns(Encoder *enc)
{
    return (int32_t)floor_div(Encoder_GetPosition(enc), (int64_t)ENC_CPR);
}

float Encoder_GetAngleDeg(Encoder *enc)
{
    int64_t pos    = Encoder_GetPosition(enc);
    int64_t in_rev = pos - floor_div(pos, (int64_t)ENC_CPR) * (int64_t)ENC_CPR;   /* 0..8191 */
    return (float)in_rev * DEG_PER_COUNT;
}

float Encoder_GetJointAngleDeg(Encoder *enc)
{
    /* double: a float can't hold large counts exactly (long runs) */
    return (float)((double)Encoder_GetPosition(enc) * DEG_PER_COUNT_D / (double)ENC_GEAR_RATIO);
}

float Encoder_GetRPM(Encoder *enc)              { return enc->rpm; }
float Encoder_GetJointSpeedDegS(Encoder *enc)   { return enc->rpm * DEG_S_PER_RPM / ENC_GEAR_RATIO; }
float Encoder_GetJointAccelDegS2(Encoder *enc)  { return enc->accel_cps2 * DEG_PER_COUNT / ENC_GEAR_RATIO; }
float Encoder_GetRPMWindow(Encoder *enc)        { return enc->rpm_window; }
float Encoder_GetRPMEdge(Encoder *enc)          { return enc->rpm_edge; }
uint32_t Encoder_GetIndexCount(Encoder *enc)    { return enc->index_count; }
uint32_t Encoder_GetIndexErrors(Encoder *enc)   { return enc->index_errors; }

int64_t Encoder_GetIndexErrorSum(Encoder *enc)
{
    uint32_t p = enter_critical();
    int64_t s = enc->index_err_sum;
    exit_critical(p);
    return s;
}

/* ========================================================================== */
/*  Diagnostics and output                                                    */
/* ========================================================================== */

uint8_t Encoder_IsRunning(Encoder *enc)
{
    /* static: keeps its value between calls (supports one encoder) */
    static uint32_t last_samples = 0;
    static uint32_t last_tick    = 0;
    static uint8_t  running      = 0;

    uint32_t now = HAL_GetTick();
    if (now - last_tick >= RUNNING_CHECK_MS) {
        running      = (enc->samples != last_samples);
        last_samples = enc->samples;
        last_tick    = now;
    }
    return running;
}

int Encoder_FormatStatus(Encoder *enc, char *buf, size_t len)
{
    char s_pos[I64_STR_LEN];
    char s_ang[FIX2_STR_LEN], s_spd[FIX2_STR_LEN], s_acc[FIX2_STR_LEN], s_rpm[FIX2_STR_LEN];

    int64_t pos = Encoder_GetPosition(enc);
    float   rpm = enc->rpm;   /* read once so speed and rpm on one line agree */

    int n = snprintf(buf, len,
        "pos=%s turns=%ld joint=%sdeg speed=%sdeg/s accel=%sdeg/s2 rpm=%s "
        "idx=%lu idx_err=%lu samples=%lu\r\n",
        i64_to_str(pos, s_pos),
        (long)floor_div(pos, (int64_t)ENC_CPR),
        fix2_to_str((double)pos * DEG_PER_COUNT_D / (double)ENC_GEAR_RATIO, s_ang),
        fix2_to_str((double)(rpm * DEG_S_PER_RPM / ENC_GEAR_RATIO), s_spd),
        fix2_to_str((double)Encoder_GetJointAccelDegS2(enc), s_acc),
        fix2_to_str((double)rpm, s_rpm),
        (unsigned long)enc->index_count,
        (unsigned long)enc->index_errors,
        (unsigned long)enc->samples);

    return clamp_len(n, len);
}

int Encoder_FormatCSV(Encoder *enc, char *buf, size_t len)
{
    char s_pos[I64_STR_LEN];
    char s_ang[FIX2_STR_LEN], s_spd[FIX2_STR_LEN], s_acc[FIX2_STR_LEN], s_rpm[FIX2_STR_LEN];

    int64_t pos = Encoder_GetPosition(enc);
    float   rpm = enc->rpm;

    int n = snprintf(buf, len, "%lu,%s,%ld,%s,%s,%s,%s,%lu\r\n",
        (unsigned long)HAL_GetTick(),
        i64_to_str(pos, s_pos),
        (long)floor_div(pos, (int64_t)ENC_CPR),
        fix2_to_str((double)pos * DEG_PER_COUNT_D / (double)ENC_GEAR_RATIO, s_ang),
        fix2_to_str((double)(rpm * DEG_S_PER_RPM / ENC_GEAR_RATIO), s_spd),
        fix2_to_str((double)Encoder_GetJointAccelDegS2(enc), s_acc),
        fix2_to_str((double)rpm, s_rpm),
        (unsigned long)enc->index_errors);

    return clamp_len(n, len);
}
