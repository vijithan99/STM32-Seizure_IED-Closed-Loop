/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern SPI_HandleTypeDef hspi3;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern TIM_HandleTypeDef htim3;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Interrupt_Monitor_Pin GPIO_PIN_2
#define Interrupt_Monitor_GPIO_Port GPIOG
#define ErrorCode_Bit_1_Pin GPIO_PIN_5
#define ErrorCode_Bit_1_GPIO_Port GPIOG
#define ErrorCode_Bit_0_Pin GPIO_PIN_6
#define ErrorCode_Bit_0_GPIO_Port GPIOG
#define ErrorCode_Bit_2_Pin GPIO_PIN_8
#define ErrorCode_Bit_2_GPIO_Port GPIOG
#define Main_Monitor_Pin GPIO_PIN_8
#define Main_Monitor_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define Compliance_Monitor_Pin GPIO_PIN_13
#define Compliance_Monitor_GPIO_Port GPIOG
#define ErrorCode_Bit_3_Pin GPIO_PIN_0
#define ErrorCode_Bit_3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
