/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//#include "common.h"
//#include "adbms_include.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim2;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TOTAL_IC 2

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define BMS_SCK_Pin GPIO_PIN_5
#define BMS_SCK_GPIO_Port GPIOA
#define BMS_MISO_Pin GPIO_PIN_6
#define BMS_MISO_GPIO_Port GPIOA
#define BMS_MOSI_Pin GPIO_PIN_7
#define BMS_MOSI_GPIO_Port GPIOA
#define BMS_WAKE_Pin GPIO_PIN_4
#define BMS_WAKE_GPIO_Port GPIOC
#define BMS_INT_Pin GPIO_PIN_5
#define BMS_INT_GPIO_Port GPIOC
#define BMS_CS2_Pin GPIO_PIN_7
#define BMS_CS2_GPIO_Port GPIOC
#define BMS_MSTR_Pin GPIO_PIN_10
#define BMS_MSTR_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define BMS_WAKE2_Pin GPIO_PIN_3
#define BMS_WAKE2_GPIO_Port GPIOB
#define BMS_MSTR2_Pin GPIO_PIN_4
#define BMS_MSTR2_GPIO_Port GPIOB
#define BMS_INT2_Pin GPIO_PIN_5
#define BMS_INT2_GPIO_Port GPIOB
#define BMS_CS_Pin GPIO_PIN_6
#define BMS_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
