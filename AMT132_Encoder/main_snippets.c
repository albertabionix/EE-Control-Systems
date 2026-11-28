/**
 * @file    main_snippets.c
 * @brief   Code to add to Core/Src/main.c on the NUCLEO-G474RE fixture
 *          (human-readable status output). Not compiled: copy each block between
 *          the matching USER CODE BEGIN / END markers that CubeMX generated.
 *
 * Assumes CubeMX names: htim3 (encoder), htim6 (100 Hz sample timer),
 * hlpuart1 (USB serial), and PD2 labelled "ENC_Z" (GPIO_EXTI2, rising edge).
 *
 * Keep the USER CODE markers: CubeMX only preserves code between them when it
 * regenerates main.c. Anything outside them is deleted.
 */

/* USER CODE BEGIN Includes */
#include "encoder.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
Encoder enc;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
  /* The motor must stay stopped until Encoder_Init returns: the encoder needs
   * a still shaft during its start-up. Enable the motor driver after this. */
  Encoder_Init(&enc, &htim3, NULL);
  HAL_TIM_Base_Start_IT(&htim6);       /* start sampling after Encoder_Init */

  /* Optional: make the Z index mark the zero position (repeatable homing
   * within one motor turn). Turn the shaft past the index to apply it. */
  /* Encoder_ZeroAtNextIndex(&enc); */
/* USER CODE END 2 */

  /* inside while (1) */
  /* USER CODE BEGIN 3 */
    char line[ENC_STATUS_LEN];
    int n = Encoder_FormatStatus(&enc, line, sizeof line);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)line, (uint16_t)n, 50);
    HAL_Delay(100);                    /* 10 lines per second */
  }
  /* USER CODE END 3 */

/* USER CODE BEGIN 4 */
/** @brief Sample timer tick: update the encoder ENC_SAMPLE_HZ times per second. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Encoder_Update(&enc);
    }
}

/** @brief Z index pulse on PD2: check the encoder hasn't gained or lost counts. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ENC_Z_Pin) {
        Encoder_OnIndex(&enc);
    }
}
/* USER CODE END 4 */

/* If the fixture project already defines either callback (other timers or EXTI
 * pins), don't add a second copy: put the if-block inside the existing one. */
