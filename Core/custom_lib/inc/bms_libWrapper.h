/*
 * bms_libWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "bms_datatypes.h"
#include "main.h"
#include "bms_can.h"


typedef enum
{
    VOLTAGE_C,
    VOLTAGE_C_AVG,
    VOLTAGE_C_FIL,
    VOLTAGE_S,
    VOLTAGE_TEMP,
    TOTAL_VOLTAGE_TYPES,
} VoltageTypes;


void bms_init(void);

void bms_readSid(void);

void bms_readConfigA(void);

void bms_readConfigB(void);



void bms68_setGpo45(uint8_t twoBitIndex);


void bms_startAdcvCont(void);

void bms_readCellVoltage(VoltageTypes voltageType);

void bms_getAuxMeasurement(void);


float bms_calculateBalancing(float deltaThreshold);

void bms_startDischarge(float threshold);

void bms_stopDischarge(void);
void bms_softReset(void);


void bms29_setGpo(void);

void bms29_readVB(void);
void bms29_readCurrent(void);

void bms_balancingMeasureVoltage(void);

void bms_startBalancing(float deltaThreshold);


void BMS_GetCanData(CanTxMsg** buff, uint32_t* len);

uint32_t BMS_LoopActive(void);
uint32_t BMS_LoopCharging(void);
uint32_t BMS_LoopIDLE(void);


