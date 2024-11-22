/**
********************************************************************************
*
* @file:    pal.h
*
* @brief:   This file contains the pal header files & functions.
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

#ifndef __PAL_H
#define __PAL_H

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "common.h"
#include "application.h"

void Delay_ms(uint32_t delay);
void adBmsCsLow(void);
void adBmsCsHigh(void);
void spiWriteBytes(uint8_t *tx_data, uint16_t size);
void spiWriteReadBytes(uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
void spiReadBytes(uint8_t *rx_data, uint16_t size);
void startTimer();
void stopTimer();
uint32_t getTimCount();
void adBmsWakeupIc(uint8_t total_ic);

#endif /*__PAL_H */

/** @}*/