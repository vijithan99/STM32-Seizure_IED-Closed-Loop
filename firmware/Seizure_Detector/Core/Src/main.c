/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "seizure_detect.h"
#include "input_buffer.h"
#include "uart_recieve.h"
#include <stdio.h>
#include <stdint.h>
#include "main.h"

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN 0 */
#define BLOCK_SIZE 500
#define SAMP_FREQ 5000
#define SAMP_PERIOD_US (1000000U / SAMP_FREQ)
#define BLOCK_PERIOD_US ((BLOCK_SIZE * 1000000U) / SAMP_FREQ)
/* USER CODE END 0 */

void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);

int main(void)
{
    static int32_t buffer_storage[BLOCK_SIZE * 2];
    static input_buffer_t input_buf;
    int32_t *ping_ptr = NULL;
    int32_t *pong_ptr = NULL;

    // Intialization detection structs
    detect_params_t detect_params = {
        .alpha = 0.01f,
        .k = 5.0f,
        .min_std = 1.0f,
        .persist_blocks = 5,
        .warmup_blocks = 100,
        .refractory_us = 500000
    };
    detect_state_t detect_state;
    detect_event_t ev;

    // Count of how many blocks have been processed
    uint32_t block_time_us = 0;

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    /* Initialize leds */
    BSP_LED_Init(LED_GREEN);
    BSP_LED_Init(LED_YELLOW);
    BSP_LED_Init(LED_RED);

    /* Initialize USER push-button */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    /* Initialize COM1 port */
    BspCOMInit.BaudRate   = 921600;
    BspCOMInit.WordLength = COM_WORDLENGTH_8B;
    BspCOMInit.StopBits   = COM_STOPBITS_1;
    BspCOMInit.Parity     = COM_PARITY_NONE;
    BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;

    if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
    {
        Error_Handler();
    }

    /* Initialize buffer and UART RX module */
    buffer_init(&input_buf, buffer_storage, SAMP_FREQ, BLOCK_SIZE);
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    uart_rx_init(&input_buf);
    detect_init(&detect_state, &detect_params);
    uart_rx_start();

    while (1)
    {
        if (BspButtonState == BUTTON_PRESSED){

            BspButtonState = BUTTON_RELEASED;
            BSP_LED_Toggle(LED_GREEN);
            BSP_LED_Toggle(LED_YELLOW);
            BSP_LED_Toggle(LED_RED);
            printf("LEDS ON!\r\n");
            printf("write_index = %lu\r\n", input_buf.write_index);
            printf("ping_ready = %u\r\n", input_buf.ping_ready);
            printf("pong_ready = %u\r\n", input_buf.pong_ready);
            printf("block_samples = %lu\r\n", input_buf.block_samples);
            printf("fs_hz = %u\r\n", input_buf.fs_hz);
            printf("uart_call = %lu\r\n", input_buf.uart_call);
            printf("buffer[0] = %ld\r\n", input_buf.buffer[0]);
            printf("buffer[1] = %ld\r\n", input_buf.buffer[1]);
            printf("buffer[2] = %ld\r\n", input_buf.buffer[2]);
            printf("buffer[3] = %ld\r\n", input_buf.buffer[3]);

        }

        if (input_buf.ping_ready){
            input_buf.ping_ready = 0;
            ping_ptr = buffer_get_ping_ptr(&input_buf);
            ev = detect_process_block(&detect_state, ping_ptr, BLOCK_SIZE, block_time_us);
            block_time_us += BLOCK_PERIOD_US;

            // process ping block here
        }

        if (input_buf.pong_ready){
            input_buf.pong_ready = 0;
            pong_ptr = buffer_get_pong_ptr(&input_buf);
            ev = detect_process_block(&detect_state, pong_ptr, BLOCK_SIZE, block_time_us);
            block_time_us += BLOCK_PERIOD_US;

            // process pong block here
        }

        if (ev == DETECT_SEIZURE_ONSET) {
            BSP_LED_Toggle(LED_RED);
            printf("SEIZURE DETECTED at %lu us\r\n", block_time_us);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BSP_LED_On(LED_YELLOW);
    uart_rx_irq_handler(huart);
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
