/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define ARDUINO_GPIO12_MISO_Pin GPIO_PIN_4
#define ARDUINO_GPIO12_MISO_GPIO_Port GPIOB
#define ARDUINO_GPIO13_SCK_Pin GPIO_PIN_3
#define ARDUINO_GPIO13_SCK_GPIO_Port GPIOB
#define UART5_TX_Pin GPIO_PIN_12
#define UART5_TX_GPIO_Port GPIOC
#define ARDUINO_GPIO10_Pin GPIO_PIN_15
#define ARDUINO_GPIO10_GPIO_Port GPIOA
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define SDP_NSS_A_Pin GPIO_PIN_6
#define SDP_NSS_A_GPIO_Port GPIOB
#define ARDUINO_GPIO8_Pin GPIO_PIN_11
#define ARDUINO_GPIO8_GPIO_Port GPIOG
#define SDP_GPIO6_IN_Pin GPIO_PIN_13
#define SDP_GPIO6_IN_GPIO_Port GPIOJ
#define SDP_GPIO5_IN_Pin GPIO_PIN_12
#define SDP_GPIO5_IN_GPIO_Port GPIOJ
#define F469_VIO_CHECK_Pin GPIO_PIN_12
#define F469_VIO_CHECK_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_7
#define LED_RED_GPIO_Port GPIOK
#define LED_ORANGE_Pin GPIO_PIN_6
#define LED_ORANGE_GPIO_Port GPIOK
#define LED_GREEN_Pin GPIO_PIN_5
#define LED_GREEN_GPIO_Port GPIOK
#define ARDUINO_GPIO7_Pin GPIO_PIN_10
#define ARDUINO_GPIO7_GPIO_Port GPIOG
#define SDP_GPIO7_IN_Pin GPIO_PIN_14
#define SDP_GPIO7_IN_GPIO_Port GPIOJ
#define ARDUINO_GPIO5_Pin GPIO_PIN_11
#define ARDUINO_GPIO5_GPIO_Port GPIOA
#define PROC_STATUS_Pin GPIO_PIN_4
#define PROC_STATUS_GPIO_Port GPIOK
#define AWAKE_BLU_Pin GPIO_PIN_3
#define AWAKE_BLU_GPIO_Port GPIOK
#define ARDUINO_GPIO4_Pin GPIO_PIN_9
#define ARDUINO_GPIO4_GPIO_Port GPIOG
#define UART5_RX_Pin GPIO_PIN_2
#define UART5_RX_GPIO_Port GPIOD
#define ARDUINO_GPIO6_Pin GPIO_PIN_10
#define ARDUINO_GPIO6_GPIO_Port GPIOA
#define I2C3_SDA_Pin GPIO_PIN_9
#define I2C3_SDA_GPIO_Port GPIOC
#define SDP_NSS_C_Pin GPIO_PIN_7
#define SDP_NSS_C_GPIO_Port GPIOC
#define SDP_NSS_B_Pin GPIO_PIN_6
#define SDP_NSS_B_GPIO_Port GPIOC
#define ARDUINO_GPIO2_Pin GPIO_PIN_7
#define ARDUINO_GPIO2_GPIO_Port GPIOG
#define ARDUINO_GPIO2_EXTI_IRQn EXTI9_5_IRQn
#define SDP_NSS_Pin GPIO_PIN_6
#define SDP_NSS_GPIO_Port GPIOF
#define SPI5_MOSI_Pin GPIO_PIN_9
#define SPI5_MOSI_GPIO_Port GPIOF
#define SPI5_MISO_Pin GPIO_PIN_8
#define SPI5_MISO_GPIO_Port GPIOF
#define ARDUINO_ADC_IN3_Pin GPIO_PIN_1
#define ARDUINO_ADC_IN3_GPIO_Port GPIOC
#define SDP_GPIO3_ADC_Pin GPIO_PIN_4
#define SDP_GPIO3_ADC_GPIO_Port GPIOJ
#define ARDUINO_GPIO3_Pin GPIO_PIN_12
#define ARDUINO_GPIO3_GPIO_Port GPIOD
#define SDP_GPIO4_OUT_Pin GPIO_PIN_5
#define SDP_GPIO4_OUT_GPIO_Port GPIOJ
#define ARDUINO_GPIO0_UART4_RX_Pin GPIO_PIN_1
#define ARDUINO_GPIO0_UART4_RX_GPIO_Port GPIOA
#define ARDUINO_GPIO1_UART4_TX_Pin GPIO_PIN_0
#define ARDUINO_GPIO1_UART4_TX_GPIO_Port GPIOA
#define ARDUINO_ADC_IN1_Pin GPIO_PIN_4
#define ARDUINO_ADC_IN1_GPIO_Port GPIOA
#define ARDUINO_ADC_IN4_Pin GPIO_PIN_4
#define ARDUINO_ADC_IN4_GPIO_Port GPIOC
#define SDP_GPIO2_OUT_Pin GPIO_PIN_3
#define SDP_GPIO2_OUT_GPIO_Port GPIOJ
#define I2C3_SCL_Pin GPIO_PIN_7
#define I2C3_SCL_GPIO_Port GPIOH
#define ARDUINO_ADC_IN0_Pin GPIO_PIN_2
#define ARDUINO_ADC_IN0_GPIO_Port GPIOA
#define ARDUINO_ADC_IN2_Pin GPIO_PIN_6
#define ARDUINO_ADC_IN2_GPIO_Port GPIOA
#define ARDUINO_ADC_IN5_Pin GPIO_PIN_5
#define ARDUINO_ADC_IN5_GPIO_Port GPIOC
#define SPI5_SCK_Pin GPIO_PIN_6
#define SPI5_SCK_GPIO_Port GPIOH
#define ARDUINO_GPIO11_MOSI_Pin GPIO_PIN_7
#define ARDUINO_GPIO11_MOSI_GPIO_Port GPIOA
#define SDP_GPIO0_OUT_Pin GPIO_PIN_0
#define SDP_GPIO0_OUT_GPIO_Port GPIOJ
#define SDP_GPIO1_OUT_Pin GPIO_PIN_1
#define SDP_GPIO1_OUT_GPIO_Port GPIOJ
#define ARDUINO_GPIO9_Pin GPIO_PIN_15
#define ARDUINO_GPIO9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
