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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "seizure_detect.h"
#include "ied_detect.h"
#include "input_buffer.h"
#include "uart_recieve.h"
#include "user_functions.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BLOCK_SIZE 500
#define SAMP_FREQ 5000
#define SAMP_PERIOD_US (1000000U / SAMP_FREQ)
#define BLOCK_PERIOD_US ((BLOCK_SIZE * 1000000U) / SAMP_FREQ)

static void handle_detect_event(detect_event_t ev, const char *block_name, uint64_t block_time_us){
	switch (ev) {
	case DETECT_SEIZURE_ONSET_LL:
		BSP_LED_Toggle(LED_GREEN);
		printf("LL DETECTED (%s), %lu\r\n", block_name, block_time_us);
		break;

    case DETECT_SEIZURE_ONSET_RMS:
        BSP_LED_Toggle(LED_YELLOW);
        printf("RMS DETECTED (%s), %lu\r\n", block_name, block_time_us);
        break;

    case DETECT_SEIZURE_ONSET_BOTH:
        BSP_LED_Toggle(LED_RED);
        printf("BOTH DETECTED (%s), %lu\r\n", block_name, block_time_us);
        break;

    case DETECT_NO_EVENT:
    default:
        break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	static int32_t buffer_storage[BLOCK_SIZE * 2];
	static input_buffer_t input_buf;
	int32_t *ping_ptr = NULL;
	int32_t *pong_ptr = NULL;

	// Intialization seizure detection structs
	detect_params_t detect_params = {
			.alpha = 0.01f,
			.k_ll = 2.25f,
			.k_rms = 2.25f,
			.min_std = 1.0f,
			.persist_blocks = 8,
			.warmup_blocks = 80,
			.refractory_us = 500000};

	detect_state_t detect_state;
	detect_event_t ev;

	// Intialization IED detection structs
	ied_params_t ied_params = {
	    .fs_hz = SAMP_FREQ,

	    .env_k = 5.0f,
	    .amp_min_k = 8.0f,
	    .amp_artifact_k = 20.0f,

	    .baseline_alpha = 0.00002f,
	    .min_std = 1.0f,

	    .warmup_samples = SAMP_FREQ * 5,     // 1 seconds baseline at SAMP_FREQ kHz
	    .refractory_us = 3000000,         // 3 seconds

	    .envelope_window_samples = (SAMP_FREQ * 5U) / 1000U    // Samples of envelope smoothing
	};

	ied_state_t ied_state;

	// Count of how many blocks have been processed
	uint64_t block_time_us = 0;

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 921600;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */
  /* Initialize buffer and UART RX module */
  buffer_init(&input_buf, buffer_storage, SAMP_FREQ, BLOCK_SIZE);
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  uart_rx_init(&input_buf);
  detect_init(&detect_state, &detect_params);
  ied_init(&ied_state, &ied_params);
  uart_rx_start();

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);
  BSP_LED_On(LED_YELLOW);
  BSP_LED_On(LED_RED);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* -- Sample board code for User push-button in interrupt mode ---- */
	  if (BspButtonState == BUTTON_PRESSED)
	  {
	    /* Update button state */
	    BspButtonState = BUTTON_RELEASED;
	    /* -- Sample board code to toggle leds ---- */
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
		printf("overrun_count = %lu\r\n", input_buf.overrun_count);

		printf("buffer[0] = %ld\r\n", input_buf.buffer[0]);
		printf("buffer[1] = %ld\r\n", input_buf.buffer[1]);
		printf("buffer[2] = %ld\r\n", input_buf.buffer[2]);
		printf("buffer[3] = %ld\r\n", input_buf.buffer[3]);
	}


	if (input_buf.ping_ready){
	  input_buf.ping_ready = 0;
	  ping_ptr = buffer_get_ping_ptr(&input_buf);

	  // Sample-by-sample IED detection inside this block
	  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
		  uint64_t sample_time_us = block_time_us + ((uint64_t)i * SAMP_PERIOD_US);
		  uint32_t t_sec = (uint32_t)(sample_time_us / 1000000ULL);
		  uint32_t t_usec = (uint32_t)(sample_time_us % 1000000ULL);

		  detect_ied_event_t ied_ev = ied_process_sample(&ied_state,
												  ping_ptr[i],
												  sample_time_us);

		  if (ied_ev == IED_DETECTED) {
			  BSP_LED_Toggle(LED_RED);
			  //printf("IED DETECTED (ping), %lu\r\n", (unsigned long)sample_time_us);
			  printf("IED DETECTED (ping), %lu.%06lu\r\n",
			         (unsigned long)t_sec,
			         (unsigned long)t_usec);
		  } else if (ied_ev == IED_REJECTED_LOW_AMPLITUDE) {
			  BSP_LED_Toggle(LED_YELLOW);
			  printf("IED REJECTED LOW AMP (ping), %lu env=%ld thresh=%ld amp=%ld amin=%ld\r\n",
					  (unsigned long)sample_time_us,
					 (int32_t)ied_state.last_envelope,
					 (int32_t)ied_state.last_env_thresh,
					 (int32_t)ied_state.last_amp_hp,
					 (int32_t)ied_state.last_amp_min_thresh);
		  } else if (ied_ev == IED_REJECTED_ARTIFACT) {
			  BSP_LED_Toggle(LED_GREEN);
			  printf("IED REJECTED ARTIFACT (ping), %lu\r\n", (unsigned long)sample_time_us);
		  }
	  }
	  /* Seizure Detection component
	   *
	   * ev = detect_process_block(&detect_state, ping_ptr, BLOCK_SIZE, block_time_us);
	   *
	   * handle_detect_event(ev, "ping", block_time_us);
	   */

		// process ping block here
		block_time_us += BLOCK_PERIOD_US;

	  }

	  if (input_buf.pong_ready){
		  input_buf.pong_ready = 0;
		  pong_ptr = buffer_get_pong_ptr(&input_buf);
		  // Sample-by-sample IED detection inside this block
		  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
			  uint64_t sample_time_us = block_time_us + ((uint64_t)i * SAMP_PERIOD_US);
			  uint32_t t_sec = (uint32_t)(sample_time_us / 1000000ULL);
			  uint32_t t_usec = (uint32_t)(sample_time_us % 1000000ULL);


			  detect_ied_event_t ied_ev = ied_process_sample(&ied_state,
													  pong_ptr[i],
													  sample_time_us);

			  if (ied_ev == IED_DETECTED) {
				  BSP_LED_Toggle(LED_RED);
				  printf("IED DETECTED (pong), %lu.%06lu\r\n",
				         (unsigned long)t_sec,
				         (unsigned long)t_usec);
			  } else if (ied_ev == IED_REJECTED_LOW_AMPLITUDE) {
				  BSP_LED_Toggle(LED_YELLOW);
				  printf("IED REJECTED LOW AMP (pong), %lu env=%ld thresh=%ld amp=%ld amin=%ld\r\n",
						  (unsigned long)sample_time_us,
						 (int32_t)ied_state.last_envelope,
						 (int32_t)ied_state.last_env_thresh,
						 (int32_t)ied_state.last_amp_hp,
						 (int32_t)ied_state.last_amp_min_thresh);
			  } else if (ied_ev == IED_REJECTED_ARTIFACT) {
				  BSP_LED_Toggle(LED_GREEN);
				  printf("IED REJECTED ARTIFACT (pong), %lu\r\n", (unsigned long)sample_time_us);
			  }
		  }

		  // Existing block-based seizure detection
		  /* Seizure Detection component
		   *  ev = detect_process_block(&detect_state, pong_ptr, BLOCK_SIZE, block_time_us);
		   *
		   * handle_detect_event(ev, "pong", block_time_us);
		   *
		   */

		  block_time_us += BLOCK_PERIOD_US;

	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_32BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 13750;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Interrupt_Monitor_Pin|ErrorCode_Bit_1_Pin|ErrorCode_Bit_0_Pin|ErrorCode_Bit_2_Pin
                          |Compliance_Monitor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Main_Monitor_GPIO_Port, Main_Monitor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ErrorCode_Bit_3_GPIO_Port, ErrorCode_Bit_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Interrupt_Monitor_Pin ErrorCode_Bit_1_Pin ErrorCode_Bit_0_Pin ErrorCode_Bit_2_Pin
                           Compliance_Monitor_Pin */
  GPIO_InitStruct.Pin = Interrupt_Monitor_Pin|ErrorCode_Bit_1_Pin|ErrorCode_Bit_0_Pin|ErrorCode_Bit_2_Pin
                          |Compliance_Monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : Main_Monitor_Pin */
  GPIO_InitStruct.Pin = Main_Monitor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Main_Monitor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ErrorCode_Bit_3_Pin */
  GPIO_InitStruct.Pin = ErrorCode_Bit_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ErrorCode_Bit_3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uart_rx_irq_handler(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart_rx_start();
    }
}

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
