/**
 * @file    encoder.h
 * @brief   Driver for the Same Sky AMT132S-V incremental encoder (STM32 HAL).
 * @author  Jordan Rivero (Bionix MCU team)
 * @version 2.0
 *
 * Provides position, turn count, angle, speed and acceleration, plus Z-index
 * checking that flags any missed or extra counts.
 *
 * Hardware used (configured in CubeMX, see README.md):
 *  - Encoder timer (TIM3 on the G474 fixture), Encoder Mode TI1 and TI2:
 *    counts every A/B edge in hardware, 8192 counts per revolution.
 *  - Sample timer (TIM6): interrupt at ENC_SAMPLE_HZ, calls Encoder_Update().
 *  - Z index on PD2 (GPIO_EXTI, rising edge): calls Encoder_OnIndex().
 *  - Optional capture timer for M/T speed (ENC_USE_CAPTURE = 1).
 *
 * Encoder connector (AMT132S, mating connector JST ZPDR-18V-S):
 *  - pin 4 GND, pin 6 +5 V, pin 8 A, pin 10 B, pin 12 Z
 *  - pins 1/2 UART RX/TX (configuration only), pin 14 MCLR (never connect)
 *  - all other pins unused
 *  Signals are 5 V: every MCU pin used must be 5 V tolerant (FT).
 *
 * @note The motor must be stopped while Encoder_Init() runs: the encoder
 *       needs a still shaft for its 200 ms start-up (datasheet).
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include <stdint.h>
#include <stddef.h>

/* ========================================================================== */
/*  Configuration: must match the hardware and CubeMX                        */
/* ========================================================================== */

/** Pulses per revolution on channel A (factory default 2048). */
#define ENC_PPR                 2048u

/** Counts per revolution with x4 decoding (both edges of A and B). */
#define ENC_CPR                 (ENC_PPR * 4u)

/** 1 if the encoder timer is 32-bit (TIM2/TIM5), 0 if 16-bit (TIM3 on the fixture). */
#define ENC_TIMER_IS_32BIT      0

/** Rate of the sample timer interrupt that calls Encoder_Update(), in Hz. */
#define ENC_SAMPLE_HZ           100u

/** 1 = M/T speed using a capture timer, 0 = window speed (counts per sample). */
#define ENC_USE_CAPTURE         0

/** Capture timer tick rate in Hz (capture mode only). G474 at 170 MHz with prescaler 16 = 10 MHz. */
#define ENC_TICK_HZ             10000000u

/** Capture timer "Prescaler Division Ratio" (1, 2, 4 or 8), capture mode only. */
#define ENC_IC_DIV              8u

/** No capture for this long means the shaft has stopped (capture mode only). */
#define ENC_STOP_TIMEOUT_MS     200u

/** Motor turns per one joint turn. Negative if the gearbox reverses direction. */
#define ENC_GEAR_RATIO          1.0f

/** Start-up wait in Encoder_Init(); the datasheet requires >= 200 ms with the shaft still. */
#define ENC_STARTUP_DELAY_MS    250u

/**
 * Acceleration filter time constant, in ms.
 * Larger = smoother but slower to respond. 90% response time is about 5.3 x tau.
 * Simulated noise at rest with +/-2 count jitter (motor shaft, gear ratio 1):
 *   tau  25 ms: ~130 ms response, ~27 deg/s^2 noise at 100 Hz, ~8 at 1 kHz
 *   tau  50 ms: ~270 ms response,  ~5 deg/s^2 noise at 100 Hz, ~2 at 1 kHz
 *   tau 100 ms: ~530 ms response,  ~1 deg/s^2 noise at 100 Hz
 */
#define ENC_ACCEL_TAU_MS        25.0f

/**
 * Index check tolerance, in counts, at standstill. Each Z pulse must land within
 * this many counts of a whole number of turns from the first pulse in the same
 * direction. 0 catches a single missed or extra count. If the fixture shows
 * occasional false +/-1 errors at low speed (Z interrupt racing the timer's input
 * filter), set this to 1; then only errors of 2+ counts are reported.
 */
#define ENC_INDEX_TOL_COUNTS    0

/**
 * Worst-case delay between the Z edge and reading the counter in the interrupt,
 * in microseconds. The shaft keeps turning during that delay, so the tolerance
 * grows with speed: tol = ENC_INDEX_TOL_COUNTS + speed x latency.
 * Example: 3 us at 8000 RPM (~1.09 million counts/s) allows ~3 counts, so a
 * single-count error is only guaranteed to be caught below ~2400 RPM. For exact
 * checking at any speed, the G474's hardware encoder-index feature (Z on
 * TIM3_ETR, which is PD2) could replace this interrupt in a later version.
 */
#define ENC_INDEX_LATENCY_US    3.0f

/** Size of the text buffer Encoder_FormatStatus() needs. */
#define ENC_STATUS_LEN          192u

/** Size of the text buffer Encoder_FormatCSV() needs. */
#define ENC_CSV_LEN             192u

/** Column names for Encoder_FormatCSV(). Send once before the data. */
#define ENC_CSV_HEADER \
    "time_ms,pos_counts,turns,joint_deg,joint_speed_degs,joint_accel_degs2,rpm,index_errors\r\n"

/* ========================================================================== */
/*  Encoder state                                                             */
/* ========================================================================== */

/**
 * @brief State of one encoder. Read it through the Encoder_Get...() functions.
 *
 * Fields marked volatile are shared between interrupts and the main loop.
 */
typedef struct {
    TIM_HandleTypeDef *htim_enc;      /**< Encoder-mode timer (&htim3 on the fixture). */
    TIM_HandleTypeDef *htim_cap;      /**< Capture timer, or NULL when ENC_USE_CAPTURE = 0. */
    volatile uint8_t   ready;         /**< Set when Encoder_Init has finished; interrupts are ignored before then. */

    /* Position (owned by Encoder_Update) */
    volatile uint32_t last_cnt;       /**< Hardware counter at the previous sample. */
    volatile int64_t  position;       /**< Counts since start or last zero. 64-bit: never overflows. */
    volatile int32_t  delta;          /**< Counts during the last sample period. */
    volatile float    rpm_window;     /**< Window-method speed, motor RPM. */

    /* Capture data (written by Encoder_OnCapture) */
    volatile uint32_t cap_time;
    volatile uint32_t cap_period;
    volatile int8_t   cap_dir;
    volatile uint8_t  cap_have_prev;
    volatile int32_t  cap_index;
    volatile uint32_t cap_epoch;
    volatile uint32_t captures;       /**< Total captures (debug). */
    volatile float    rpm_edge;       /**< Edge-timing speed (comparison only). */

    /* M/T state (only used inside Encoder_Update) */
    uint32_t          mt_base_time;
    int32_t           mt_base_index;
    uint32_t          mt_base_epoch;
    uint8_t           mt_base_valid;
    float             mt_ticks_per_cap;

    volatile float    rpm;            /**< Best available speed, motor RPM. */

    /* Acceleration tracking filter (only used inside Encoder_Update) */
    double            filt_pos;       /**< Filter position estimate, counts. */
    float             filt_vel;       /**< Filter speed estimate, counts/s. */
    float             filt_acc;       /**< Filter acceleration estimate, counts/s^2. */
    float             gain_pos;       /**< Filter gains, computed once in Encoder_Init. */
    float             gain_vel;
    float             gain_acc;
    uint8_t           filt_ready;
    volatile float    accel_cps2;     /**< Acceleration output, counts/s^2. */

    /* Z index checking (written by Encoder_OnIndex) */
    volatile int64_t  index_ref[2];   /**< Position of the first index pulse, per direction. */
    volatile uint8_t  index_ref_ok[2];
    volatile uint32_t index_count;    /**< Index pulses seen. */
    volatile uint32_t index_errors;   /**< Index pulses that were off by more than the tolerance. */
    volatile int32_t  index_last_err; /**< Counts gained (+) or lost (-) at the last error. */
    volatile int64_t  index_err_sum;  /**< Net counts gained or lost over all errors. */

    /* Zero requests (applied inside Encoder_Update) */
    volatile uint8_t  zero_request;   /**< 0 none, 1 zero now, 2 zero at next index. */
    volatile uint8_t  zero_pending;   /**< Index pulse seen, zero ready to apply. */
    volatile int64_t  zero_target;    /**< Position that becomes the new zero. */

    volatile uint32_t samples;        /**< Encoder_Update() calls; rises by ENC_SAMPLE_HZ per second. */
} Encoder;

/* ========================================================================== */
/*  Functions                                                                 */
/* ========================================================================== */

/**
 * @brief  Initialise the encoder and start its timers.
 * @param  enc       Encoder state to initialise.
 * @param  htim_enc  Timer configured in Encoder Mode TI1 and TI2.
 * @param  htim_cap  Capture timer, or NULL when ENC_USE_CAPTURE = 0.
 * @note   Blocks for ENC_STARTUP_DELAY_MS. The motor must be stopped during this
 *         call. Call before starting the sample timer. Index and capture
 *         interrupts that arrive before this returns are ignored.
 * @note   Calls Error_Handler() if the encoder timer fails to start, or if its
 *         Counter Period doesn't match ENC_TIMER_IS_32BIT (must be 65535 for a
 *         16-bit timer, 0xFFFFFFFF for 32-bit), since the rollover math would
 *         otherwise silently give wrong positions.
 */
void    Encoder_Init(Encoder *enc, TIM_HandleTypeDef *htim_enc, TIM_HandleTypeDef *htim_cap);

/**
 * @brief  Update position, speed and acceleration.
 * @param  enc  Encoder state.
 * @note   Call from the sample timer interrupt, exactly ENC_SAMPLE_HZ times per second.
 */
void    Encoder_Update(Encoder *enc);

/**
 * @brief  Record a Z index pulse and check it lands a whole number of turns
 *         from the first one.
 * @param  enc  Encoder state.
 * @note   Call from HAL_GPIO_EXTI_Callback for the Z pin (PD2, rising edge).
 */
void    Encoder_OnIndex(Encoder *enc);

/**
 * @brief  Record a captured edge of channel A (M/T speed).
 * @param  enc  Encoder state.
 * @note   Call from the capture interrupt. Does nothing when ENC_USE_CAPTURE = 0.
 */
void    Encoder_OnCapture(Encoder *enc);

/**
 * @brief  Make the current position zero.
 * @param  enc  Encoder state.
 * @note   Applied at the next Encoder_Update(), within 1/ENC_SAMPLE_HZ seconds.
 */
void    Encoder_Zero(Encoder *enc);

/**
 * @brief  Make the next Z index pulse the zero position.
 * @param  enc  Encoder state.
 * @note   Gives a repeatable zero within one motor turn: the index is a fixed
 *         mark on the shaft (movable with the encoder's serial '0' command).
 */
void    Encoder_ZeroAtNextIndex(Encoder *enc);

/**
 * @brief  Position since start or last zero.
 * @return Counts (8192 per motor turn), signed.
 */
int64_t Encoder_GetPosition(Encoder *enc);

/**
 * @brief  Complete motor turns since start or last zero.
 * @return Whole turns, rounded down: -1 means between -1 and 0 turns.
 */
int32_t Encoder_GetTurns(Encoder *enc);

/**
 * @brief  Motor shaft angle within the current turn.
 * @return Degrees, 0 to just under 360.
 */
float   Encoder_GetAngleDeg(Encoder *enc);

/**
 * @brief  Joint angle since start or last zero, not wrapped.
 * @return Degrees (uses ENC_GEAR_RATIO).
 */
float   Encoder_GetJointAngleDeg(Encoder *enc);

/**
 * @brief  Motor speed from the best available method (M/T or window).
 * @return Revolutions per minute, signed.
 */
float   Encoder_GetRPM(Encoder *enc);

/**
 * @brief  Joint angular speed.
 * @return Degrees per second, signed.
 */
float   Encoder_GetJointSpeedDegS(Encoder *enc);

/**
 * @brief  Joint angular acceleration from the tracking filter.
 * @return Degrees per second squared, signed. Smoothed with ENC_ACCEL_TAU_MS.
 */
float   Encoder_GetJointAccelDegS2(Encoder *enc);

/** @brief Window-method speed in RPM (for comparison). */
float   Encoder_GetRPMWindow(Encoder *enc);

/** @brief Edge-timing speed in RPM (for comparison, capture mode only). */
float   Encoder_GetRPMEdge(Encoder *enc);

/** @brief Number of Z index pulses seen. */
uint32_t Encoder_GetIndexCount(Encoder *enc);

/**
 * @brief  Number of index pulses that were off by more than the tolerance,
 *         meaning counts were missed or noise added extra ones.
 */
uint32_t Encoder_GetIndexErrors(Encoder *enc);

/** @brief Net counts gained (+) or lost (-) over all detected index errors. */
int64_t Encoder_GetIndexErrorSum(Encoder *enc);

/**
 * @brief  Check the sample timer is running.
 * @return 1 if Encoder_Update() ran in the last 100 ms, else 0.
 */
uint8_t Encoder_IsRunning(Encoder *enc);

/**
 * @brief  Write a human-readable status line.
 * @param  buf  Output buffer, at least ENC_STATUS_LEN bytes.
 * @param  len  Size of buf.
 * @return Number of characters written (ready for HAL_UART_Transmit).
 */
int     Encoder_FormatStatus(Encoder *enc, char *buf, size_t len);

/**
 * @brief  Write one CSV data line, columns as in ENC_CSV_HEADER.
 * @param  buf  Output buffer, at least ENC_CSV_LEN bytes.
 * @param  len  Size of buf.
 * @return Number of characters written.
 */
int     Encoder_FormatCSV(Encoder *enc, char *buf, size_t len);

#endif /* ENCODER_H */
