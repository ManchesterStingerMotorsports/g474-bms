/**
********************************************************************************
*
* @file:    application.h
*
* @brief:   This file contains the application header files.
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
/*! \addtogroup Main
*  @{
*/

/*! \addtogroup Application
*  @{
*/

#ifndef __APPLICATION_H
#define __APPLICATION_H

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "adbms2950.h"
#include "pal.h"
#include "adi_2950.h"
#include "print_result.h"

//#define MBED /*!< Switch between IAR Workbench or Mbed Platform*/

#ifdef MBED
#include "mbed.h"
#include "SPI.h"
#include "Timer.h"
#endif /* MBED */

void app_main(void);
void run_command(int cmd);

#endif /* __APPLICATION_H */

/** @}*/
/** @}*/