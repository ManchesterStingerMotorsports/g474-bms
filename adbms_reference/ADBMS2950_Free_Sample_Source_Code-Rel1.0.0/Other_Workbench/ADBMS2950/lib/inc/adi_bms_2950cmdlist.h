/**
********************************************************************************
*
* @file:    adi_bms_2950cmdlist.h
*
* @brief:   This file contains 2950 command list.
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

/*! \addtogroup Command_List
*  @{
*/

#ifndef __ADBMSCOMMAND_H
#define __ADBMSCOMMAND_H

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "common.h"

/*!< configuration registers commands */
extern uint8_t WRCFGA[2];
extern uint8_t WRCFGB[2];
extern uint8_t RDCFGA[2];
extern uint8_t RDCFGB[2];

/*!< Read cell voltage result registers commands */
extern uint8_t RDI[2];
extern uint8_t RDVBAT[2];
extern uint8_t RDIVBAT[2];
extern uint8_t RDVA[2];
extern uint8_t RDVB[2];
extern uint8_t RDOCR[2];
extern uint8_t RDALLA[2];

/*!< Read Result Registers Commands B */
extern uint8_t RDIAV[2];
extern uint8_t RDVBAV[2];
extern uint8_t RDIVBAV[2];
extern uint8_t RDALLB[2];

/*!< Read Result Registers Commands C */
extern uint8_t RDRVA[2];
extern uint8_t RDRVB[2];
extern uint8_t RDVC[2];
extern uint8_t RDVD[2];
extern uint8_t RDCALL[2];

/*!< Read status registers */
extern uint8_t RDSTATA[2];
extern uint8_t RDSTATB[2];
extern uint8_t RDSTATC[2];
extern uint8_t RDSTATCERR[2];   /*!< With ERR bit*/
extern uint8_t RDSTATD[2];
extern uint8_t RDSTATE[2];

/*!< Read all Status Registers */
extern uint8_t RDASALL[2];

/*!< Pwm registers commands */
extern uint8_t WRPWM[2];
extern uint8_t RDPWM[2];
extern uint8_t WRPWM2[2];
extern uint8_t RDPWM2[2];

/*!< Clear commands */
extern uint8_t CLRAB[2];
extern uint8_t CLRC[2];
extern uint8_t CLRSTAT [2];
extern uint8_t CLRFLAG[2];

/*!< Poll adc command */
extern uint8_t PLADC[2];
extern uint8_t PLI1ADC[2];
extern uint8_t PLI2ADC[2];
extern uint8_t PLVADC[2];
extern uint8_t PLAUX[2];

/*!< GPIOs Comm commands */
extern uint8_t WRCOMM[2];
extern uint8_t RDCOMM[2];
extern uint8_t STCOMM[13];

/*!< Control Commands */
extern uint8_t RDSID[2];
extern uint8_t RSTCC[2];
extern uint8_t SNAP[2];
extern uint8_t UNSNAP[2];
extern uint8_t SRST[2];

/*!< Reserved Read Commands */
extern uint8_t RDAUXC[2];
extern uint8_t RDRAXC[2];
extern uint8_t RDAUXD[2];
extern uint8_t RDRAXD[2];

#endif /* __BMS_COMMAND_H */

/** @}*/
/** @}*/
