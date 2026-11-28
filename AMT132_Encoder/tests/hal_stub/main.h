/**
 * @file    main.h (test stub)
 * @brief   Minimal stand-in for the STM32 HAL so encoder.c can be compiled and
 *          tested on a PC. Only what encoder.c uses is provided.
 */
#ifndef MAIN_H_STUB
#define MAIN_H_STUB

#include <stdint.h>

typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;

/** Simulated timer: CNT is the counter, ARR the Counter Period, down = direction bit. */
typedef struct {
    uint32_t CNT;
    uint32_t ARR;
    uint32_t CCR1;
    int      down;
} TIM_TypeDef;

typedef struct {
    TIM_TypeDef *Instance;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_1    0x00u
#define TIM_CHANNEL_ALL  0x3Cu

#define __HAL_TIM_SET_COUNTER(h, v)          ((h)->Instance->CNT = (uint32_t)(v) & (h)->Instance->ARR)
#define __HAL_TIM_GET_COUNTER(h)             ((h)->Instance->CNT & (h)->Instance->ARR)
#define __HAL_TIM_GET_AUTORELOAD(h)          ((h)->Instance->ARR)
#define __HAL_TIM_IS_TIM_COUNTING_DOWN(h)    ((h)->Instance->down)

extern uint32_t g_tick;
extern int      g_error_handler_calls;

static inline uint32_t HAL_GetTick(void) { return g_tick; }
static inline void HAL_Delay(uint32_t ms) { g_tick += ms; }

static inline HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    (void)h; (void)ch; return HAL_OK;
}
static inline HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *h, uint32_t ch)
{
    (void)h; (void)ch; return HAL_OK;
}
static inline uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *h, uint32_t ch)
{
    (void)ch; return h->Instance->CCR1;
}

/* CMSIS interrupt control: no interrupts on a PC, so these do nothing. */
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t p) { (void)p; }

/* The real Error_Handler() halts; the stub only counts calls. */
static inline void Error_Handler(void) { g_error_handler_calls++; }

#endif /* MAIN_H_STUB */
