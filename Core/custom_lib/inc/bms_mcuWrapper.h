/*
 * bms_mcuWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "stdint.h"

// Chip select
void bms_csLow(void);
void bms_csHigh(void);


// Timer Functions
void bms_startTimer(void);
void bms_stopTimer(void);
uint32_t bms_getTimCount(void);  // Get time elapsed in us


// Wake up all the IC in the daisy chain
void bms_wakeupChain(void);

// Microsecond delay
void bms_delayUs(uint32_t us);

// Microsecond delay while preventing isospi sleep
void bms_delayMsActive(uint32_t ms);
