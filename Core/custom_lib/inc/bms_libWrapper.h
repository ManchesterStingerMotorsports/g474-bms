/*
 * bms_libWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "bms_datatypes.h"
#include "main.h"


void bms_init(void);

void bms_readSid(void);

void bms_readConfigA(void);

void bms_readConfigB(void);



void bms68_setGpo45(uint8_t twoBitIndex);


void bms_startAdcvCont(void);

void bms_readAvgCellVoltage(void);

void bms_readSVoltage(void);

void bms_openWireCheck(void);

void bms_getAuxMeasurement(void);


float bms_calculateBalancing(float deltaThreshold);

void bms_startDischarge(float threshold);

void bms_stopDischarge(void);
void bms_softReset(void);


void bms29_setGpo(void);

void bms_readVB(void);

void bms_balancingMeasureVoltage(void);

void bms_startBalancing(float deltaThreshold);

