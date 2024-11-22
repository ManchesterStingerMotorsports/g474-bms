/**
********************************************************************************
*
* @file:    adi_bms_utility.h
*
* @brief:   This file contains utility functions.
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
/*! \addtogroup BMS_Driver
*  @{
*/

/*! \addtogroup Driver_Utility
*  @{
*/

#ifndef __UTILITY_H
#define __UTILITY_H

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "adbms2950.h"

uint16_t Pec15_Calc(uint8_t len, uint8_t *data );
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);
void spiSendCmd(uint8_t tx_cmd[2]);
void spiReadData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *rx_data, uint8_t *pec_error, uint8_t *cmd_cntr, uint8_t regData_size);
void spiWriteData(uint8_t tIC, uint8_t tx_cmd[2], uint8_t *data);
void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
uint32_t adBmsPollAdc(uint8_t tx_cmd[2]);

#endif /* __UTILITY_H */
/** @}*/
/** @}*/