/**
********************************************************************************
*
* @file:    adi_bms_2950cmdlist.c
*
* @brief:   This file contains ADBMS2950 command list implementation.
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
/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "adbms2950.h"

/*!< configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/*!< Read cell voltage result registers commands */
uint8_t RDI[2]         = { 0x00, 0x04 };
uint8_t RDVBAT[2]      = { 0x00, 0x06 };
uint8_t RDIVBAT[2]     = { 0x00, 0x08 };
uint8_t RDVA[2]        = { 0x00, 0x0A };
uint8_t RDVB[2]        = { 0x00, 0x09 };
uint8_t RDOCR[2]       = { 0x00, 0x0B };
uint8_t RDALLA[2]      = { 0x00, 0x0C };

/*!< Read Result Registers Commands B */
uint8_t RDIAV[2]       = { 0x00, 0x44 };
uint8_t RDVBAV[2]      = { 0x00, 0x46 };
uint8_t RDIVBAV[2]     = { 0x00, 0x48 };
uint8_t RDALLB[2]      = { 0x00, 0x4C };

/*!< Read Result Registers Commands C */
uint8_t RDRVA[2]       = { 0x00, 0x07 };
uint8_t RDRVB[2]       = { 0x00, 0x0D };
uint8_t RDVC[2]        = { 0x00, 0x03 };
uint8_t RDVD[2]        = { 0x00, 0x05 };
uint8_t RDCALL[2]      = { 0x00, 0x10 };

/*!< Read status registers */
uint8_t RDSTATA[2]       = { 0x00, 0x30 };
uint8_t RDSTATB[2]       = { 0x00, 0x31 };
uint8_t RDSTATC[2]       = { 0x00, 0x32 };
uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };   /* ERR */
uint8_t RDSTATD[2]       = { 0x00, 0x33 };
uint8_t RDSTATE[2]       = { 0x00, 0x34 };

/*!< Read all Status Registers */
uint8_t RDASALL[2]       = { 0x00, 0x35 };

/*!< Pwm registers commands */
uint8_t WRPWMA[2]         = { 0x00, 0x20 };
uint8_t RDPWMA[2]         = { 0x00, 0x22 };
uint8_t WRPWMB[2]         = { 0x00, 0x21 };
uint8_t RDPWMB[2]         = { 0x00, 0x23 };

/*!< Clear commands */
uint8_t CLRAB[2]         = { 0x07, 0x11 };
uint8_t CLRC[2]          = { 0x07, 0x16 };
uint8_t CLRSTAT [2]      = { 0x07, 0x13 };
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };

/*!< Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };
uint8_t PLI1ADC[2]       = { 0x07, 0x1C };
uint8_t PLI2ADC[2]       = { 0x07, 0x1D };
uint8_t PLVADC[2]        = { 0x07, 0x1E };
uint8_t PLAUX[2]         = { 0x07, 0x1F };

/*!< GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
/*!< command + dummy data for 72 clock cycles */
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/*!< Control Commands */
uint8_t RDSID[2]         = { 0x00, 0x2C };
uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };

/*!< Reserved Read Commands */
uint8_t RDAUXC[2]         = { 0x00, 0x1B };
uint8_t RDRAXC[2]         = { 0x00, 0x1E };
uint8_t RDAUXD[2]         = { 0x00, 0x1F };
uint8_t RDRAXD[2]         = { 0x00, 0x25 };

/** @}*/
/** @}*/
