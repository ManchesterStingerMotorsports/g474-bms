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

#include <stdbool.h>


typedef enum
{
    VOLTAGE_C,
    VOLTAGE_C_AVG,
    VOLTAGE_C_FIL,
    VOLTAGE_S,
    VOLTAGE_TEMP,
    TOTAL_VOLTAGE_TYPES,
} VoltageTypes;

typedef enum {
    REG_CONFIG_A,
    REG_CONFIG_B,
    REG_PWM_A,
    REG_PWM_B,
    REG_SID,
    TOTAL_REG_TYPES,
} RegisterTypes;


BMS_StatusTypeDef bms_init(void);

void bms68_setGpo45(uint8_t twoBitIndex);

void bms_startAdcvCont(bool enableRedundant);

BMS_StatusTypeDef bms_readCellVoltage(VoltageTypes voltageType);

BMS_StatusTypeDef bms_readRegister(RegisterTypes regTypes);

BMS_StatusTypeDef bms_getAuxMeasurement(void);

void bms_startDischarge(float threshold);

void bms_stopDischarge(void);

void bms_softReset(void);

BMS_StatusTypeDef bms29_readVB(void);

BMS_StatusTypeDef bms29_readCurrent(void);

BMS_StatusTypeDef bms_balancingMeasureVoltage(void);

void bms_startBalancing(float deltaThreshold);

void BMS_GetCanData(CanTxMsg** buff, uint32_t* len);


BMS_StatusTypeDef BMS_ProgramLoop(void);

void BMS_EnableCharging(bool enabled);
void BMS_ChargingButtonLogic(void);
bool BMS_IsCharging(void);

void BMS_EnableBalancing(bool enabled);
void BMS_ToggleBalancing(void);

bool BMS_CheckNewDataReady(void);



