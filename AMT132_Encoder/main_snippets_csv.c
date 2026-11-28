/**
 * @file    main_snippets_csv.c
 * @brief   CSV-output version of blocks 2 and 3 for graphing and logging with
 *          plot_encoder.py. Blocks Includes, PV and 4 are the same as in
 *          main_snippets.c. Not compiled: copy into Core/Src/main.c.
 *
 * Output: ENC_CSV_HEADER once, then one line every 10 ms:
 *   time_ms,pos_counts,turns,joint_deg,joint_speed_degs,joint_accel_degs2,rpm,index_errors
 * A line takes ~5 ms to send at 115200 baud, so 100 lines/s uses about half the link.
 */

/* USER CODE BEGIN 2 */
  /* Motor must stay stopped until Encoder_Init returns (encoder start-up). */
  Encoder_Init(&enc, &htim3, NULL);
  HAL_TIM_Base_Start_IT(&htim6);

  /* Column names once, so Python/Excel know what each column is. */
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)ENC_CSV_HEADER, sizeof(ENC_CSV_HEADER) - 1u, 50);

  const uint32_t csv_period_ms = 10u;   /* 100 lines per second */
  uint32_t next_csv_ms = HAL_GetTick();
/* USER CODE END 2 */

  /* inside while (1) */
  /* USER CODE BEGIN 3 */
    /* Send on a fixed schedule. HAL_Delay(10) would add the ~5 ms send time on
     * top of the delay and give ~65 lines/s instead of 100. The signed
     * difference stays correct when HAL_GetTick() wraps around. */
    if ((int32_t)(HAL_GetTick() - next_csv_ms) >= 0) {
        next_csv_ms += csv_period_ms;
        char line[ENC_CSV_LEN];
        int n = Encoder_FormatCSV(&enc, line, sizeof line);
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)line, (uint16_t)n, 50);
    }
  }
  /* USER CODE END 3 */
