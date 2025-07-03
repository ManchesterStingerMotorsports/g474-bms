/*
 * bms_can.h
 *
 *  Created on: Jun 8, 2025
 *      Author: amrlxyz
 */

#ifndef INC_BMS_CAN_H_
#define INC_BMS_CAN_H_

#include "main.h"


#define INC_BMS_CAN_H_

#define BASE_CAN_ID 0xB000


typedef struct
{
    uint8_t data[8];
    FDCAN_RxHeaderTypeDef header;
} CanRxMsg;


typedef struct
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header;
} CanTxMsg;



// --- Charger-Specific Definitions ---
// Define the CAN IDs for charger communication.
// Replace these with the actual IDs specified by your charger's manufacturer.
#define CHARGER_CONFIG_CAN_ID 0x1806E5F4
#define CHARGER_STATUS_CAN_ID 0x18FF50E5

// Structure to hold the parsed status data from the charger.
typedef struct {
    float output_voltage; // Volts
    float output_current; // Amps
    uint8_t hardware_fault      : 1;
    uint8_t over_temp_fault     : 1;
    uint8_t input_voltage_fault : 1;
    uint8_t charging_state      : 1;
    uint8_t comms_state         : 1;
    uint8_t reserved            : 3;
} ChargerStatus;

// Structure to hold the configuration we want to send to the charger.
typedef struct {
    float    target_voltage;    // Volts
    float    max_current;       // Amps
    uint8_t  enable_charging;   // 0 = Disable, 1 = Enable
} ChargerConfiguration;

extern ChargerStatus chargerStatus;


void BMS_CAN_Config(void);

void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len);

void BMS_CAN_Test(void);

void BMS_CAN_SendMsg(CanTxMsg msg);

void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config, uint8_t* data);

#endif /* INC_BMS_CAN_H_ */
