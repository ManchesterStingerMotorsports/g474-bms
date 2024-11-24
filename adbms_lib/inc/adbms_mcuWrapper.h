

//// pal.c for 2950
//// mcuWrapper.c for 6830


#pragma once

#include "common.h"


void Delay_ms(uint32_t delay);
void adBmsCsLow(void);
void adBmsCsHigh(void);
void spiWriteBytes
( 
  uint8_t *tx_Data,                              /*Array of bytes to be written on the SPI port*/
  uint16_t size                                /*Option: Number of bytes to be written on the SPI port*/
);
void spiWriteReadBytes
(
  uint8_t *tx_data,                             /*array of data to be written on SPI port*/
  uint8_t *rx_data,                             /*Input: array that will store the data read by the SPI port*/
  uint16_t size                             /*Option: number of bytes*/
);
void spiReadBytes(uint8_t *rx_data, uint16_t size);
void startTimer(void);
void stopTimer(void);
uint32_t getTimCount(void);
void adBmsWakeupIc(uint8_t total_ic);

/** @}*/
/** @}*/
