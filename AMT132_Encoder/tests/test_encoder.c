/**
 * @file    test_encoder.c
 * @brief   PC unit tests for encoder.c (no hardware needed).
 *
 * Simulates the shaft one count at a time: the timer counter follows the
 * position (plus any injected glitch), and a Z pulse fires whenever the shaft
 * enters the index count. Encoder_Update() runs every UPDATE_COUNTS counts.
 *
 * Build and run from this folder (gcc, or MinGW/MSYS2 on Windows):
 *   gcc -std=c11 -Wall -Wextra -Ihal_stub -I.. test_encoder.c ../encoder.c -lm -o test_encoder
 *   ./test_encoder
 */
#include "encoder.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t g_tick;
int      g_error_handler_calls;

#define INDEX_COUNT     1000    /* Z pulse at this count within each turn */
#define UPDATE_COUNTS   400     /* counts of motion between Encoder_Update calls */
#define UPDATE_MS       10u     /* one update = 10 ms at ENC_SAMPLE_HZ = 100 */
#define DEG_PER_COUNT   (360.0 / ENC_CPR)

static TIM_TypeDef       tim;
static TIM_HandleTypeDef htim = { &tim };
static Encoder           enc;
static int64_t           truth;    /* real shaft position, counts */
static int64_t           glitch;   /* counts missed (-) or added (+) by "noise" */
static int               failures;

static void check(int ok, const char *what)
{
    printf("%s %s\n", ok ? "  ok  " : "  FAIL", what);
    if (!ok) {
        failures++;
    }
}

static int64_t mod(int64_t a, int64_t m)
{
    int64_t r = a % m;
    return (r < 0) ? r + m : r;
}

/* Move the shaft one count and fire Z when entering the index count. */
static void step(int dir)
{
    int64_t before = truth;
    truth  += dir;
    tim.down = (dir < 0);
    tim.CNT  = (uint32_t)(truth + glitch) & tim.ARR;
    if (mod(truth, ENC_CPR) == INDEX_COUNT && mod(before, ENC_CPR) != INDEX_COUNT) {
        Encoder_OnIndex(&enc);
    }
}

static void update(void)
{
    g_tick += UPDATE_MS;
    Encoder_Update(&enc);
}

/* Move n counts in one direction, updating every UPDATE_COUNTS counts. */
static void move(int64_t n, int dir)
{
    for (int64_t i = 1; i <= n; i++) {
        step(dir);
        if (i % UPDATE_COUNTS == 0) {
            update();
        }
    }
    update();
}

static void reset(void)
{
    truth = 0;
    glitch = 0;
    tim.ARR = 0xFFFFu;          /* 16-bit timer, as on the fixture */
    tim.CNT = 0;
    g_error_handler_calls = 0;
    Encoder_Init(&enc, &htim, NULL);
}

int main(void)
{
    char line[ENC_STATUS_LEN];

    printf("Safety\n");
    {
        /* enc is a zeroed global: interrupts before Encoder_Init must be ignored. */
        static Encoder fresh;
        Encoder_OnIndex(&fresh);
        Encoder_Update(&fresh);
        Encoder_OnCapture(&fresh);
        check(fresh.index_count == 0 && fresh.samples == 0,
              "interrupts before Encoder_Init are ignored (no crash)");

        tim.ARR = 0xFFFFFFFFu;  /* 32-bit Counter Period, but ENC_TIMER_IS_32BIT = 0 */
        g_error_handler_calls = 0;
        Encoder_Init(&enc, &htim, NULL);
        check(g_error_handler_calls == 1, "Counter Period mismatch calls Error_Handler");
    }

    printf("Position and turns\n");
    reset();
    move(20 * ENC_CPR, -1);
    check(Encoder_GetPosition(&enc) == truth, "16-bit rollover: 20 turns backwards tracked exactly");
    check(Encoder_GetTurns(&enc) == -20, "turns = -20");
    move(ENC_CPR - 100, +1);
    check(Encoder_GetTurns(&enc) == -20, "floor division: -19 turns - 100 counts is turn -20");
    check(fabsf(Encoder_GetAngleDeg(&enc) - (float)((ENC_CPR - 100) * DEG_PER_COUNT)) < 0.01f,
          "angle within turn wraps correctly for negative positions");

    printf("Z index\n");
    reset();
    move(3 * ENC_CPR, +1);
    move(2 * ENC_CPR, -1);
    check(Encoder_GetIndexErrors(&enc) == 0 && Encoder_GetIndexCount(&enc) >= 5,
          "no errors during clean motion in both directions");
    glitch -= 1;
    move(5 * ENC_CPR, +1);
    check(Encoder_GetIndexErrors(&enc) == 1 && enc.index_last_err == -1,
          "one lost count detected exactly once (-1)");
    glitch += 2;
    move(3 * ENC_CPR, +1);
    move(2 * ENC_CPR, -1);
    check(Encoder_GetIndexErrors(&enc) == 2 && enc.index_last_err == 2
          && Encoder_GetIndexErrorSum(&enc) == 1,
          "two extra counts detected once (+2), net sum +1, reverse still clean");

    printf("Long runs\n");
    reset();
    {
        /* Jump ~3e9 counts (beyond 32 bits). A multiple of 65536 keeps the
         * simulated 16-bit counter unchanged, as if the run had really happened. */
        const int64_t big = 45778LL * 65536;
        enc.position += big;
        enc.filt_pos += (double)big;
        truth        += big;
        move(ENC_CPR, +1);
        char want[40];
        snprintf(want, sizeof want, "pos=%lld ", (long long)truth);
        Encoder_FormatStatus(&enc, line, sizeof line);
        check(Encoder_GetPosition(&enc) == truth && strncmp(line, want, strlen(want)) == 0,
              "64-bit position (> 2^31) stored and printed exactly");
        check(Encoder_GetIndexErrors(&enc) == 0, "no false index errors after a long run");
    }

    printf("Zeroing\n");
    reset();
    move(3 * ENC_CPR + 123, +1);
    Encoder_Zero(&enc);
    update();
    check(Encoder_GetPosition(&enc) == 0, "Encoder_Zero gives position 0");
    move(3 * ENC_CPR, +1);
    check(Encoder_GetIndexErrors(&enc) == 0, "no false index errors after zeroing");
    Encoder_ZeroAtNextIndex(&enc);
    move(9000, +1);
    check(Encoder_GetPosition(&enc) == mod(truth - INDEX_COUNT, ENC_CPR),
          "ZeroAtNextIndex makes the Z mark position 0");

    printf("Speed and acceleration\n");
    reset();
    {
        /* Constant 1000 deg/s^2 from rest for 3 s, updated at 100 Hz. */
        const double accel_cps2 = 1000.0 / DEG_PER_COUNT;
        for (int k = 1; k <= 300; k++) {
            double t = k * 0.01;
            int64_t target = (int64_t)floor(0.5 * accel_cps2 * t * t);
            while (truth < target) {
                step(+1);
            }
            update();
        }
        float a = Encoder_GetJointAccelDegS2(&enc);
        float v = Encoder_GetJointSpeedDegS(&enc);
        printf("        accel %.2f deg/s^2 (true 1000), speed %.2f deg/s (true 3000)\n",
               (double)a, (double)v);
        check(fabsf(a - 1000.0f) < 30.0f, "acceleration converges to the true value");
        check(fabsf(v - 3000.0f) < 30.0f, "speed matches the true value");
    }

    printf("Output\n");
    {
        Encoder_FormatCSV(&enc, line, sizeof line);
        int commas = 0;
        for (const char *p = line; *p; p++) {
            commas += (*p == ',');
        }
        int header_commas = 0;
        for (const char *p = ENC_CSV_HEADER; *p; p++) {
            header_commas += (*p == ',');
        }
        printf("        %s", line);
        check(commas == header_commas, "CSV line has the same columns as ENC_CSV_HEADER");
    }

    printf("\n%s (%d failure%s)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED",
           failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
