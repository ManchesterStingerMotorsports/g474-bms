/**
********************************************************************************
*
* @file:    pal.c
*
* @brief:   This file contains the pal function implementation.
*
* @details:
*
*******************************************************************************
Copyright(c) 2020 Analog Devices, Inc. All Rights Reserved. This software is
proprietary & confidential to Analog Devices, Inc. and its licensors. By using
this software you agree to the terms of the associated Analog Devices License
Agreement.
*******************************************************************************
*/

/*! \addtogroup Platform_Abstracion_Layer
*  @{
*/
#include "pal.h"
#define WAKEUP_DELAY 1                          /* BMS ic wakeup delay ms*/

#ifdef MBED
extern SPI spi;
extern Timer timer;
extern DigitalOut chip_select;

/**
 *******************************************************************************
 * Function: Delay_ms
 * @brief Delay mili second
 *
 * @details This function insert delay in ms.
 *
 * Parameters:
 * @param [in]  delay   Delay_ms
 *
 * @return None
 *
 *******************************************************************************
*/
void Delay_ms(uint32_t delay)
{
  wait_ms((int)delay);
}

/**
 *******************************************************************************
 * Function: adBmsCsLow
 * @brief Select chip select low
 *
 * @details This function does spi chip select low.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsLow()
{
  spi.lock();
  chip_select = 0;
}

/**
 *******************************************************************************
 * Function: adBmsCsHigh
 * @brief Select chip select High
 *
 * @details This function does spi chip select high.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsHigh()
{
  chip_select = 1;
  spi.unlock();
}

/**
 *******************************************************************************
 * Function: spiWriteBytes
 * @brief Writes an array of bytes out of the SPI port.
 *
 * @details This function wakeup bms ic in IsoSpi mode send dumy byte data in spi line..
 *
 * @param [in]  *tx_Data    Tx data pointer
 *
 * @param [in]   size       Numberof bytes to be send on the SPI line
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteBytes(uint8_t *tx_data, uint16_t size)
{
  uint8_t rx_data[size];
  spi.write((char *)tx_data, size ,(char *)rx_data, size);
}

/**
 *******************************************************************************
 * Function: spiWriteReadBytes
 * @brief Writes and read a set number of bytes using the SPI port.
 *
 * @details This function writes and read a set number of bytes using the SPI port.
 *
 * @param [in]  *tx_data    Tx data pointer
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size            Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteReadBytes
(
uint8_t *tx_data,                   /*array of data to be written on SPI port*/
uint8_t *rx_data,                   /*Input: array that will store the data read by the SPI port*/
uint16_t size                       /*Option: number of bytes*/
)
{
  uint16_t data_size = (4 + size);
  uint8_t cmd[data_size];
  memcpy(&cmd[0], &tx_data[0], 4); /* dst, src, size */
  spi.write((char *)cmd, data_size ,(char *)cmd, data_size);
  memcpy(&rx_data[0], &cmd[4], size); /* dst, src, size */
}

/**
 *******************************************************************************
 * Function: spiReadBytes
 * @brief Read number of bytes using the SPI port.
 *
 * @details This function Read a set number of bytes using the SPI port.
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size        Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiReadBytes(uint8_t *rx_data, uint16_t size)
{
  uint8_t tx_data[size];
  for(uint8_t i=0; i < size; i++)
  {
    tx_data[i] = 0xFF;
  }
  spi.write((char *)tx_data, size ,(char *)rx_data, size);
}

/**
 *******************************************************************************
 * Function: startTimer()
 * @brief Start timer
 *
 * @details This function start the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void startTimer()
{
  timer.start();
}

/**
 *******************************************************************************
 * Function: stopTimer()
 * @brief Stop timer
 *
 * @details This function stop the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void stopTimer()
{
  timer.stop();
}

/**
 *******************************************************************************
 * Function: getTimCount()
 * @brief Get Timer Count Value
 *
 * @details This function return the timer count value.
 *
 * @return tim_count
 *
 *******************************************************************************
*/
uint32_t getTimCount()
{
  uint32_t count = 0;
  count = timer.read_us();
  timer.reset();
  return(count);
}
#else

#include "main.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_it.h"

extern SPI_HandleTypeDef hspi1;                     /* Mcu dependent SPI1 handler */
extern SPI_HandleTypeDef hspi5;                     /* Mcu dependent SPI5 handler */
extern TIM_HandleTypeDef htim2;                     /* Mcu dependent TIM2 handler */
extern TIM_HandleTypeDef htim5;                     /* Mcu dependent TIM5 handler */

#define SPI_TIME_OUT HAL_MAX_DELAY                  /* SPI Time out delay                   */
#define BMS_CS_PIN ARDUINO_GPIO10_Pin               /* Mcu dependent BMS chip select        */
#define BMS_GPIO_PORT ARDUINO_GPIO10_GPIO_Port      /* Mcu dependent BMS chip select port   */

SPI_HandleTypeDef       *spi1   = &hspi1;           /* MUC SPI1 Handler  */
SPI_HandleTypeDef       *spi5   = &hspi5;           /* MUC SPI5 Handler  */
TIM_HandleTypeDef       *tim2   = &htim2;           /* MUC TIM2 Handler  */
TIM_HandleTypeDef       *tim4   = &htim5;           /* MUC TIM5 Handler  */

/**
 *******************************************************************************
 * Function: Delay_ms
 * @brief Delay mili second
 *
 * @details This function insert delay in ms.
 *
 * Parameters:
 * @param [in]  delay   Delay_ms
 *
 * @return None
 *
 *******************************************************************************
*/
void Delay_ms(uint32_t delay)
{
  HAL_Delay(delay);
}

/**
 *******************************************************************************
 * Function: adBmsCsLow
 * @brief Select chip select low
 *
 * @details This function does spi chip select low.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsLow()
{
  HAL_GPIO_WritePin(BMS_GPIO_PORT, BMS_CS_PIN, GPIO_PIN_RESET);
}

/**
 *******************************************************************************
 * Function: adBmsCsHigh
 * @brief Select chip select High
 *
 * @details This function does spi chip select high.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsHigh()
{
  HAL_GPIO_WritePin(BMS_GPIO_PORT, BMS_CS_PIN, GPIO_PIN_SET);
}

/**
 *******************************************************************************
 * Function: spiWriteBytes
 * @brief Writes an array of bytes out of the SPI port.
 *
 * @details This function sends the byte data into spi mosi line.
 *
 * @param [in]  *tx_Data    Tx data pointer
 *
 * @param [in]  size            Numberof bytes to be send on the SPI line
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteBytes(uint8_t *tx_data, uint16_t size)
{
  HAL_SPI_Transmit(spi1, tx_data, size, SPI_TIME_OUT); /* SPI1 , data, size, timeout */
}

/**
 *******************************************************************************
 * Function: spiWriteReadBytes
 * @brief Writes and read a set number of bytes using the SPI port.
 *
 * @details This function writes and read a set number of bytes into spi port.
 *
 * @param [in]  *tx_data    Tx data pointer
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size            Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteReadBytes(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
  HAL_SPI_Transmit(spi1, tx_data, 4, SPI_TIME_OUT);
  HAL_SPI_Receive(spi1, rx_data, size, SPI_TIME_OUT);
}

/**
 *******************************************************************************
 * Function: spiReadBytes
 * @brief Read number of bytes using the SPI port.
 *
 * @details This function Read a set number of bytes spi miso line.
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size            Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiReadBytes(uint8_t *rx_data, uint16_t size)
{
  HAL_SPI_Receive(spi1, rx_data, size, SPI_TIME_OUT);
}

/**
 *******************************************************************************
 * Function: startTimer()
 * @brief Start timer
 *
 * @details This function start the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void startTimer()
{
  HAL_TIM_Base_Start(tim2);
}

/**
 *******************************************************************************
 * Function: stopTimer()
 * @brief Stop timer
 *
 * @details This function stop the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void stopTimer()
{
  HAL_TIM_Base_Stop(tim2);
}

/**
 *******************************************************************************
 * Function: getTimCount()
 * @brief Get Timer Count Value
 *
 * @details This function return the timer count value.
 *
 * @return tim_count
 *
 *******************************************************************************
*/
uint32_t getTimCount()
{
  uint32_t count = 0;
  count = __HAL_TIM_GetCounter(tim2);
  __HAL_TIM_SetCounter(tim2, 0);
  return(count);
}
#endif /* MBED */

/**
 *******************************************************************************
 * Function: adBmsWakeupIc
 * @brief Wakeup bms ic using chip select
 *
 * @details This function wakeup thr bms ic using chip select.
 *
 * @param [in]  total_ic    Total_ic
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsWakeupIc(uint8_t total_ic)
{
  for (uint8_t ic = 0; ic < total_ic; ic++)
  {
    adBmsCsLow();
    Delay_ms(WAKEUP_DELAY);
    adBmsCsHigh();
    Delay_ms(WAKEUP_DELAY);
  }
}

/** @}*/