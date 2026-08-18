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
#include <stdbool.h>
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

static bool rhs_lvds_loopback_test(void){
    uint32_t tx[] = {
        0xA5C33C5AU,
        0xC0FF0000U,
        0x12345678U,
        0x00000000U,
        0xFFFFFFFFU
    };

    uint32_t rx[sizeof(tx) / sizeof(tx[0])] = {0};

    const uint16_t word_count =
        (uint16_t)(sizeof(tx) / sizeof(tx[0]));

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        &hspi3,
        (const uint8_t *)tx,
        (uint8_t *)rx,
        word_count,
        100U
    );

    if (status != HAL_OK) {
        printf(
            "LVDS loopback failed: status=%d error=0x%08lX\r\n",
            (int)status,
            (unsigned long)HAL_SPI_GetError(&hspi3)
        );
        return false;
    }

    bool passed = true;

    printf("\r\nLVDS round-trip loopback\r\n");

    for (uint32_t i = 0U; i < word_count; i++) {
        printf(
            "[%lu] TX=0x%08lX RX=0x%08lX %s\r\n",
            (unsigned long)i,
            (unsigned long)tx[i],
            (unsigned long)rx[i],
            tx[i] == rx[i] ? "PASS" : "FAIL"
        );

        if (tx[i] != rx[i]) {
            passed = false;
        }
    }

    printf(
        "LVDS round-trip loopback: %s\r\n",
        passed ? "PASS" : "FAIL"
    );

    return passed;
}

static bool rhs_spi_rom_test(void){
	uint32_t spi_tx[7] = {
		read_command(255, false, false),
		read_command(254, false, false),
		read_command(253, false, false),
		read_command(252, false, false),
		read_command(251, false, false),

		/* Two extra commands flush the two-command pipeline. */
		read_command(255, false, false),
		read_command(255, false, false)
	};

//	for (uint32_t i = 0; i < 7; i++) {
//		spi_tx[i] = 0xC0FF0000U;
//	}

	uint32_t spi_rx[7] = {0};

	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi3, (const uint8_t *)spi_tx, (uint8_t *)spi_rx, 7U, 100U);

	 if (status != HAL_OK){
		printf("SPI transfer failed: status=%d error=0x%08lX\r\n",
			   (int)status,
			   (unsigned long)HAL_SPI_GetError(&hspi3));

		return false;
	 }

	 printf("\r\nRaw SPI transaction:\r\n");

	 for (uint32_t i = 0U; i < 7U; i++) {
	     printf(
	         "[%" PRIu32 "] TX=0x%08" PRIX32
	         " RX=0x%08" PRIX32 "\r\n",
	         i,
	         spi_tx[i],
	         spi_rx[i]
	     );
	 }

	 /*
	  * The RHS2116 returns each command result two SPI cycles later.
	  *
	  * spi_rx[0] and spi_rx[1] contain previous/undefined pipeline results.
	  * spi_rx[2] is the result of spi_tx[0].
	  * spi_rx[3] is the result of spi_tx[1].
	  * ...
	  * spi_rx[6] is the result of spi_tx[4].
	  *
	  * spi_tx[5] and spi_tx[6] provide the extra clock cycles needed
	  * to receive the final two requested register values.
	  */
	uint16_t chip_id     = (uint16_t)(spi_rx[2] & 0xFFFFU);
	uint16_t revision_ch = (uint16_t)(spi_rx[3] & 0xFFFFU);
	uint16_t intan_nul   = (uint16_t)(spi_rx[4] & 0xFFFFU);
	uint16_t intan_ta    = (uint16_t)(spi_rx[5] & 0xFFFFU);
	uint16_t intan_in    = (uint16_t)(spi_rx[6] & 0xFFFFU);

	printf("\r\nRHS2116 ROM test\r\n");
	printf("Register 255: 0x%04X\r\n", chip_id);
	printf("Register 254: 0x%04X\r\n", revision_ch);
	printf("Register 253: 0x%04X\r\n", intan_nul);
	printf("Register 252: 0x%04X\r\n", intan_ta);
	printf("Register 251: 0x%04X\r\n", intan_in);

	bool passed =
		chip_id == 0x0020U &&
		(revision_ch & 0x00FFU) == 0x0010U &&
		intan_nul == 0x4E00U &&
		intan_ta  == 0x5441U &&
		intan_in  == 0x494EU;

	printf("ROM test: %s\r\n", passed ? "PASS" : "FAIL");

	return passed;
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
	    .amp_min_k = 10.0f,
	    .amp_artifact_k = 30.0f,

	    .baseline_alpha = 0.00002f,
	    .min_std = 1.0f,

	    .warmup_samples = SAMP_FREQ * 5,     // 1 seconds baseline at SAMP_FREQ kHz
	    .refractory_us = 3000000,         	// 3 seconds

		.envelope_window_samples = (SAMP_FREQ * 5U) / 1000U,    // Samples of envelope smoothing

		.candidate_window_samples = (SAMP_FREQ * 10U) / 1000U, // 10 ms
		.raw_abs_artifact_limit = 9000.0f,                     // for GAIN=1000 and ~±10 mV clipping
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

  // Wait for 500 ms after program starts.
  // This is due to a STM32CubeIDE-related bug that runs previously loaded software briefly on the target device
  // before running the new software when downloading and running new code. This doesn't happen when triggering a hardware
  // reset, for example pushing the RESET button on a NUCLEO board, or when downloading/running using
  // STM32CubeProgrammer. For these cases, this delay can be removed.
  // When this bug does occur, we recommend including a delay of ~500 ms so that this brief running of the previously
  // loaded program doesn't have any interaction with any peripherals and this re-run program does nothing important.
  // In practice, 50 ms is likely enough of a pause from our testing, but 500 ms is even safer.
  wait_ms(500);

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();

  /* Stop here so the normal acquisition code does not start. */
//  while (1) {
//      HAL_Delay(1000U);
//  }

  // Write register value to pause all used timers when execution pauses during debug
  SET_BIT(DBGMCU->APB1LFZ1, 0b10); // enable pausing TIM3 during debug

  // Allocate sample_memory array which will be used to store acquired data.
  allocate_sample_memory();

  // Set up SPI DMA configuration for when SPI transfers begin.
  initialize_spi_with_dma();

  initialize_stim_sequences();
  initialize_command_buffer();

  // Initialize Intan chip registers with suitable settings for this application.
  // This not only determines the initial registers, but actually writes them via SPI.
  configure_registers();


  // Populate first CONVERT_COMMANDS_PER_SEQUENCE that will repeatedly
  // convert for each sample interrupt.
  // Note that this doesn't touch the aux commands in command_sequence_MOSI.
  configure_convert_commands();

#ifdef AUTO_STIM_CMD_MODE
  configure_stim_sequences();
  process_stim_sequences();
#else
  configure_aux_commands();
#endif

  copy_next_aux_commands_to_MOSI();


  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (921600, 8 bits (7-bit data + 1 stop bit), no parity */
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

  /* USER CODE BEGIN 2 */
//  printf("\r\nStarting RHS2116 SPI ROM test...\r\n");

    /*
     * Give the headstage and interface electronics time to settle
     * before issuing the first command.
     */
  HAL_Delay(100U);

  bool rhs_detected = false;

  for (uint32_t attempt = 1U; attempt <= 50U; attempt++){
//	  printf("ROM test attempt %" PRIu32 "...\r\n", attempt);

//	  rhs_detected = rhs_spi_rom_test();

	  if (rhs_detected){
//		  printf("PASSED!!");
	  }

  }

  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_YELLOW);
  BSP_LED_Off(LED_RED);

  if (rhs_detected){
	  BSP_LED_On(LED_GREEN);
  }

  else{
	  BSP_LED_On(LED_RED);
  }

  // Turn on LED to indicate acquisition is about to start.
  BSP_LED_On(LED_GREEN);

  // Start timer so that at every period defined by INTERRUPT_TIM, an interrupt occurs, starting an SPI command sequence.
  sample_counter = 0;
  sample_interrupt_occurred = false;
  enable_interrupt_timer(true);
  main_loop_active = true;
  main_pin_status = false;

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  // Break infinite loop when loop_escape() condition has been met.
	  if (loop_escape()) break;

	  // During infinite loop, monitor if GPIO pin needs writing from low to high
	  if (main_loop_active && !main_pin_status) {
		  write_pin(Main_Monitor_GPIO_Port, Main_Monitor_Pin, true);
		  main_pin_status = true;
	  }

	  if (sample_interrupt_occurred) {
		  sample_processing_routine();
	  }

//	  if (input_buf.ping_ready){
//		  input_buf.ping_ready = 0;
//		  ping_ptr = buffer_get_ping_ptr(&input_buf);
//
//		  // Sample-by-sample IED detection inside this block
//		  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
//			  uint64_t sample_time_us = block_time_us + ((uint64_t)i * SAMP_PERIOD_US);
//			  uint32_t t_sec = (uint32_t)(sample_time_us / 1000000ULL);
//			  uint32_t t_usec = (uint32_t)(sample_time_us % 1000000ULL);
//
//			  detect_ied_event_t ied_ev = ied_process_sample(&ied_state,
//													  ping_ptr[i],
//													  sample_time_us);
//
//			  if (ied_ev == IED_DETECTED) {
//				  BSP_LED_Toggle(LED_RED);
//				  //printf("IED DETECTED (ping), %lu\r\n", (unsigned long)sample_time_us);
//				  printf("IED DETECTED (ping), %lu.%06lu\r\n",
//						 (unsigned long)t_sec,
//						 (unsigned long)t_usec);
//			  } else if (ied_ev == IED_REJECTED_LOW_AMPLITUDE) {
//				  BSP_LED_Toggle(LED_YELLOW);
//				  printf("IED REJECTED LOW AMP (ping), %lu env=%ld thresh=%ld amp=%ld amin=%ld\r\n",
//						  (unsigned long)sample_time_us,
//						 (int32_t)ied_state.last_envelope,
//						 (int32_t)ied_state.last_env_thresh,
//						 (int32_t)ied_state.last_amp_hp,
//						 (int32_t)ied_state.last_amp_min_thresh);
//			  } else if (ied_ev == IED_REJECTED_ARTIFACT) {
//				  BSP_LED_Toggle(LED_GREEN);
//				  printf("IED REJECTED ARTIFACT (ping), %lu.%06lu\r\n",
//						 (unsigned long)t_sec,
//						 (unsigned long)t_usec);
//			  }
//		  }
//	  /* Seizure Detection component
//	   *
//	   * ev = detect_process_block(&detect_state, ping_ptr, BLOCK_SIZE, block_time_us);
//	   *
//	   * handle_detect_event(ev, "ping", block_time_us);
//	   */
//
//		// process ping block here
//		block_time_us += BLOCK_PERIOD_US;
//
//	  }
//
//	  if (input_buf.pong_ready){
//		  input_buf.pong_ready = 0;
//		  pong_ptr = buffer_get_pong_ptr(&input_buf);
//		  // Sample-by-sample IED detection inside this block
//		  for (uint32_t i = 0; i < BLOCK_SIZE; i++) {
//			  uint64_t sample_time_us = block_time_us + ((uint64_t)i * SAMP_PERIOD_US);
//			  uint32_t t_sec = (uint32_t)(sample_time_us / 1000000ULL);
//			  uint32_t t_usec = (uint32_t)(sample_time_us % 1000000ULL);
//
//
//			  detect_ied_event_t ied_ev = ied_process_sample(&ied_state,
//													  pong_ptr[i],
//													  sample_time_us);
//
//			  if (ied_ev == IED_DETECTED) {
//				  BSP_LED_Toggle(LED_RED);
//				  printf("IED DETECTED (pong), %lu.%06lu\r\n",
//				         (unsigned long)t_sec,
//				         (unsigned long)t_usec);
//			  } else if (ied_ev == IED_REJECTED_LOW_AMPLITUDE) {
//				  BSP_LED_Toggle(LED_YELLOW);
//				  printf("IED REJECTED LOW AMP (pong), %lu env=%ld thresh=%ld amp=%ld amin=%ld\r\n",
//						  (unsigned long)sample_time_us,
//						 (int32_t)ied_state.last_envelope,
//						 (int32_t)ied_state.last_env_thresh,
//						 (int32_t)ied_state.last_amp_hp,
//						 (int32_t)ied_state.last_amp_min_thresh);
//			  } else if (ied_ev == IED_REJECTED_ARTIFACT) {
//				  BSP_LED_Toggle(LED_GREEN);
//				  printf("IED REJECTED ARTIFACT (pong), %lu.%06lu\r\n",
//				         (unsigned long)t_sec,
//				         (unsigned long)t_usec);
//			  }
//		  }
//
//		  // Existing block-based seizure detection
//		  /* Seizure Detection component
//		   *  ev = detect_process_block(&detect_state, pong_ptr, BLOCK_SIZE, block_time_us);
//		   *
//		   * handle_detect_event(ev, "pong", block_time_us);
//		   *
//		   */
//
//		  block_time_us += BLOCK_PERIOD_US;
//
//	  }
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
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_06CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
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
  htim3.Init.Period = 19199; 					// 96 MHz / 5000 Hz - 1 = 19199
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
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
