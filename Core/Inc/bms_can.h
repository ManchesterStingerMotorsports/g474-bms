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
#define CHARGER_STATUS_CAN_ID 0x18FF5027
#define CHARGER_CONFIG_CAN_ID 0x18FF5127

// Structure to hold the parsed status data from the charger.
typedef struct {
    float    output_voltage; // Volts
    float    output_current; // Amps
    int16_t  temperature;    // Degrees Celsius

    uint8_t hardware_fault      : 1; // Bit 0: 1 if a hardware fault is active
    uint8_t over_temp_fault     : 1; // Bit 1: 1 if temperature is too high
    uint8_t input_voltage_fault : 1; // Bit 2: 1 if input voltage is out of range
    uint8_t charging_state      : 2; // Bits 3-4: 00=Idle, 01=Constant Current, 10=Constant Voltage
    uint8_t reserved            : 3; // Bits 5-7: Unused, for future expansion
} ChargerStatus;

// Structure to hold the configuration we want to send to the charger.
typedef struct {
    float    target_voltage; // Volts
    float    max_current;    // Amps
    uint8_t  enable_charging; // 0 = Disable, 1 = Enable
} ChargerConfiguration;

extern ChargerStatus chargerStatus;


void BMS_CAN_Config(void);

void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len);

void BMS_CAN_Test(void);

void BMS_CAN_SendMsg(CanTxMsg msg);

void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config, uint8_t* data);

#endif /* INC_BMS_CAN_H_ */
