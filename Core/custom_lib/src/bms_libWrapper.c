/*
 * bms_libWrapper.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

/*
 * Compatible commands
 * ADBMS2950 == ADBMS6830
 *
 * RDCFGA
 * RDCFGB
 * ADI1     = ADCV
 * ADI2     = ADSV
 * RDI      = RDFCA or RDCVA
 * RDVB     = RDFCB or RDCVB
 * RDIVB1   = RDFCC or RDCVC
 * RDIACC   = RDACA
 * RDVBACC  = RDACB
 * RDIVB1ACC= RDACC
 *
 */

/*
 * Commands Notes
 *
 * -- 6830 --
 * ADCV : Start ADC
 * ADSV : Start redundancy ADC
 * RDCVA: Read Cell Voltage A
 * RDFCA: Read Filtered Cell A
 * RDACA: Read Averaged Cell A
 *
 * -- 2950 --
 * ADIx: Start IxADC and VBxADC
 * RDI : Read Read I1ADC and I2ADC results
 *
 */


#include "bms_libWrapper.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_cmdlist.h"

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "uartDMA.h"
#include "bms_can.h"


typedef struct
{
    ad29_cfa_t cfa_Tx;
    ad29_cfa_t cfa_Rx;
    ad29_cfb_t cfb_Tx;
    ad29_cfb_t cfb_Rx;

    float current1;
    float current2;
    float vb1;
    float vb2;

} Ic_ad29;


typedef struct
{
    // From/For Config Registers
    ad68_cfa_t cfa_Tx   [TOTAL_AD68];
    ad68_cfa_t cfa_Rx   [TOTAL_AD68];
    ad68_cfb_t cfb_Tx   [TOTAL_AD68];
    ad68_cfb_t cfb_Rx   [TOTAL_AD68];

    ad68_pwma_t pwma    [TOTAL_AD68];
    ad68_pwmb_t pwmb    [TOTAL_AD68];

    // From Read Registers
    float v_cell        [TOTAL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL];        // Average of 8 samples register (C-ADC)
    // Calculated Values
    float v_cell_diff   [TOTAL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL];
    float v_cell_sum    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_avg    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_min    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_max    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];
    float v_cell_delta  [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];

    // From AUX measurement
    float v_segment     [TOTAL_AD68];
    float temp_cell     [TOTAL_AD68][TOTAL_CELL];
    float temp_ic       [TOTAL_AD68];

    // flag stored in bits
    uint16_t isDischarging          [TOTAL_AD68];         // isDischarging Flag
    uint16_t isCellFaultDetected    [TOTAL_AD68];

} Ic_ad68;


typedef struct
{
    bool isFaultDetected    [TOTAL_IC];
    bool isCommsError       [TOTAL_IC];

    // Pack status
    float v_pack_total;
    float v_pack_min;
    float v_pack_max;
} Ic_common;

Ic_common   ic_common;
Ic_ad29     ic_ad29;
Ic_ad68     ic_ad68;

uint8_t  txData[TOTAL_IC][DATA_LEN];
uint8_t  rxData[TOTAL_IC][DATA_LEN];
uint16_t rxPec[TOTAL_IC];
uint8_t  rxCc[TOTAL_IC];

VoltageTypes dischargeVoltageType = VOLTAGE_S;

#define CAN_BUFFER_LEN (7 * 16 + 64)                // TODO: Calculate accurate buffer size
CanTxMsg canTxBuffer[CAN_BUFFER_LEN] = {0};

ChargerConfiguration chargerConfig = {
        .max_current = 1,
        .target_voltage = 450,
        .disable_charging = 1,
};

static const float balancingThreshold = 0.010; // Volts

static const bool DEBUG_SERIAL_VOLTAGE_ENABLED = false;
static const bool DEBUG_SERIAL_AUX_ENABLED = false;


volatile bool enableBalancing = false;
volatile bool newDataReady = false;



void bms_resetConfig(void)
{
    // Obtained from RDCFG after reset
    // Flipped due to Little endian
//    uint64_t const ad29_cfaDefault = 0x00 00 00 3F 3F 11;
    uint64_t const ad29_cfaDefault = 0x113F3F000000;
//    uint64_t const ad29_cfbDefault = 0x00 00 00 00 01 F0;
    uint64_t const ad29_cfbDefault = 0xF00100000000;
//    uint64_t const ad68_cfaDefault = 0x01 00 00 FF 03 00;
    uint64_t const ad68_cfaDefault = 0x0003FF000001;
//    uint64_t const ad68_cfbDefault = 0x00 F8 7F 00 00 00;
    uint64_t const ad68_cfbDefault = 0x0000007FF800;

    // Copy defaults to Tx Buffer
    if (TOTAL_AD29)
    {
        memcpy(&ic_ad29.cfa_Rx, &ad29_cfaDefault, DATA_LEN);
        memcpy(&ic_ad29.cfb_Rx, &ad29_cfbDefault, DATA_LEN);
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(&ic_ad68.cfa_Tx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ic_ad68.cfb_Tx[ic], &ad68_cfbDefault, DATA_LEN);
    }

//    ad68_cfa_t ad68_cfaT;
//    memcpy(&ad68_cfaT, &ad68_cfaDefault, DATA_LEN);
}


void bms_writeRegister(RegisterTypes regType)
{
    uint8_t* command;

    // Use a switch statement to handle the logic for each register type
    switch (regType)
    {
        case REG_CONFIG_A:
            // Prepare data for writing to Configuration Register A
            if (TOTAL_AD29) {
                memcpy(txData[0], &ic_ad29.cfa_Tx, DATA_LEN);
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.cfa_Tx[ic], DATA_LEN);
            }
            command = WRCFGA; // Set the specific command for this operation
            break;

        case REG_CONFIG_B:
            // Prepare data for writing to Configuration Register B
            if (TOTAL_AD29) {
                memcpy(txData[0], &ic_ad29.cfb_Tx, DATA_LEN);
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.cfb_Tx[ic], DATA_LEN);
            }
            command = WRCFGB; // Set the specific command for this operation
            break;

        case REG_PWM_A:
            // Prepare data for writing to PWM Register Group A
            if (TOTAL_AD29) {
                memset(txData[0], 0x00, DATA_LEN); // PWM registers for ad29 are padded with 0
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.pwma[ic], DATA_LEN);
            }
            command = WRPWMA; // Set the specific command for this operation
            break;

        case REG_PWM_B:
            // Prepare data for writing to PWM Register Group B
            if (TOTAL_AD29) {
                memset(txData[0], 0x00, DATA_LEN); // PWM registers for ad29 are padded with 0
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.pwmb[ic], DATA_LEN);
            }
            command = WRPWMB; // Set the specific command for this operation
            break;

        default:
            // In case of an invalid regType
            Error_Handler();
            return;
    }

    // After preparing the buffer, transmit the data with the selected command.
    // This part is common to all cases.
    bms_transmitData(command, txData);
}


BMS_StatusTypeDef bms_init(void)
{
    bms_resetConfig();

    // For 2950 - Enable voltage measurements (Refer Schematic)
    if (TOTAL_AD29)
    {
        ic_ad29.cfa_Tx.gpo1c  = 1;      // State control
        ic_ad29.cfa_Tx.gpo1od = 0;      // 1 = Open drain, 0 = push-pull
        ic_ad29.cfa_Tx.gpo2c  = 1;      // State control
        ic_ad29.cfa_Tx.gpo2od = 0;      // 1 = Open drain, 0 = push-pull
    }

    // For 6830 Configs
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].refon = 0b1;
        ic_ad68.cfa_Tx[ic].fc = 0b001;    // 110 Hz corner freq
    }

    printfDma("\n --- BMS INIT --- \n");

    bms_wakeupChain();                                  // Wakeup needed every 4ms of Inactivity
    if (bms_readRegister(REG_SID) == BMS_ERR_COMMS)     // Make sure the comms is OK
    {
        return BMS_ERR_COMMS;
    }

    bms_writeRegister(REG_CONFIG_A);
    bms_startAdcvCont(false);            // Need to wait 8ms for the average register to fill up
    bms_delayMsActive(12);

    return BMS_OK;
}


void bms68_setGpo45(uint8_t twoBitIndex)
{
    // GPIO Output: 1 = No pulldown (Default), 0 = Pulldown
    // Only for pin 4 and 5
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].gpo1to8 = ((twoBitIndex) << 3) | (0xFF ^ (0x3 << 3));
    }

    bms_writeRegister(REG_CONFIG_A);
}


void bms_printRawData(uint8_t data[TOTAL_IC][DATA_LEN], uint8_t cc[TOTAL_IC])
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("IC%d: ", ic+1);
        for (int j = 0; j < 6; j++)             // For every byte recieved (6 bytes)
        {
            printfDma("0x%02X, ", data[ic][j]);    // Print each of the bytes
        }
        printfDma("CC: %d |   ", cc[ic]);
    }
    printfDma("\n\n");
}


bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    bool faultDetected = false;
    bool* errorIndex = ic_common.isCommsError;

    if (bms_checkRxPec(data, pec, cc, errorIndex))
    {
        printfDma("WARNING! PEC ERROR - IC:");
        for(int ic = 0; ic < TOTAL_IC; ic++)
        {
            if (errorIndex[ic])
            {
                printfDma(" %d,", ic+1);
            }
        }
        printfDma("\n");
        faultDetected = true;
    }

    return faultDetected;

    // TODO: Add command counter fault checker
    // TODO: Add fault handler for PEC fault
}


// used mostly for debugging purposes
BMS_StatusTypeDef bms_readRegister(RegisterTypes regType)
{
    char* title;
    uint8_t* cmd;
    switch (regType)
    {
    case REG_CONFIG_A:
        title = "Config A Register";
        cmd = RDCFGA;
        break;
    case REG_CONFIG_B:
        title = "Config B Register";
        cmd = RDCFGB;
        break;
    case REG_PWM_A:
        title = "PWM A Register";
        cmd = RDPWMA;
        break;
    case REG_PWM_B:
        title = "PWM B Register";
        cmd = RDPWMB;
        break;
    case REG_SID:
        title = "SID Register";
        cmd = RDSID;
        break;
    default:
        Error_Handler(); // Invalid Choice
        break;
    }

    bms_receiveData(cmd, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        return BMS_ERR_COMMS;
    }

    printfDma("%s: \n", title);
    bms_printRawData(rxData, rxCc);

    return BMS_OK;
}


void bms_startAdcvCont(bool enableRedundant)
{
    // 6830
    // For DCP = 0
    // If RD = 0 and CONT = 1, PWM discharge is unaffected
    // If RD = 1 and CONT = 0, (Might be wrong) PWM discharge interrupted temporarily until RD conversion finished (8ms typ)
    // If RD = 1 and CONT = 1, PWM discharge interrupted

    ADCV.CONT = 1;      // Continuous
    ADCV.DCP  = 0;      // Discharge permitted
    ADCV.RSTF = 1;      // Reset filter
    ADCV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    ADCV.RD   = enableRedundant;      // Redundant Measurement

    // Behaviour of 2950 (ADI1 Command)
    //

    bms_transmitCmd((uint8_t *)&ADCV);
}


void bms_parseVoltage(uint8_t rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_IC][TOTAL_CELL], uint8_t register_index)
{
    // Does not take care of 2950
    // TODO: Read master register as well

    uint8_t cell_index = (register_index * 3);

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = cell_index; c < (cell_index + 3); c++)
        {
            vArr[ic][c] = *((int16_t *)(rawData[ic + TOTAL_AD29] + (c - cell_index)*2)) * 0.00015 + 1.5;

            if (register_index == 5) // Skip last Register since the last register only stores 1 cell
            {
                break;
            }
        }
    }
}


void bms_parseAuxVoltage(uint8_t const rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_AD68][TOTAL_CELL], uint8_t cell_index, uint8_t muxIndex)
{
    // Does not take care of 2950

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (cell_index == 4)
        {
            ic_ad68.temp_ic[ic] = (*((int16_t *)(rawData[ic + TOTAL_AD29] + 2)) * 0.00015 + 1.5) / 0.0075 - 273;
            continue;
        }

        uint8_t cellArrIndex = cell_index*3;

        for (int c = cellArrIndex; c < (cellArrIndex + 3); c++)
        {
            if (c == 3 || c == 4) continue; // Skip digital output pins
            int ci = c;                     // compensate for skipped digital pins
            if (c > 4)
            {
                ci -= 2;
            }

            vArr[ic][ci*2 + muxIndex] = *((int16_t *)(rawData[ic + TOTAL_AD29] + (c-cellArrIndex)*2)) * 0.00015 + 1.5;

            if (cell_index == 3)
            {
                ic_ad68.v_segment[ic] = (*((int16_t *)(rawData[ic + TOTAL_AD29] + 4)) * 0.00015 + 1.5) * 25;
                break;
            }
        }
    }
}


void bms_calculateStats(VoltageTypes voltageType)
{
    float total_voltage = 0;
    float pack_min =  999.0;
    float pack_max = -999.0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float min =  999.0;
        float max = -999.0;
        float sum = 0;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            float voltage = ic_ad68.v_cell[voltageType][ic][c];
            sum += voltage;
            if (voltage > max)
            {
                max = voltage;
            }
            if (voltage < min)
            {
                min = voltage;
            }
            if (voltage > pack_max)
            {
                pack_max = voltage;
            }
            if (voltage < pack_min)
            {
                pack_min = voltage;
            }
        }

        total_voltage += sum;
        ic_ad68.v_cell_min  [voltageType][ic] = min;
        ic_ad68.v_cell_max  [voltageType][ic] = max;
        ic_ad68.v_cell_sum  [voltageType][ic] = sum;
        ic_ad68.v_cell_avg  [voltageType][ic] = sum / 16.0;
        ic_ad68.v_cell_delta[voltageType][ic] = max - min;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            ic_ad68.v_cell_diff[voltageType][ic][c] = ic_ad68.v_cell[voltageType][ic][c] - min;
        }
    }

    if (voltageType == dischargeVoltageType)
    {
        ic_common.v_pack_total = total_voltage;
        ic_common.v_pack_min = pack_min;
        ic_common.v_pack_max = pack_max;
    }

    // Calculate voltage diff from the lowest voltage cell
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            ic_ad68.v_cell_diff[voltageType][ic][c] = ic_ad68.v_cell[voltageType][ic][c] - pack_min;
        }
    }
}


void bms_printVoltage(VoltageTypes voltageType)
{
    float (*vArr)[TOTAL_CELL] = ic_ad68.v_cell[voltageType];
    float (*vDev)[TOTAL_CELL] = ic_ad68.v_cell_diff[voltageType];

    char* title;
    switch (voltageType)
    {
    case VOLTAGE_C:
        title = "C Voltage";
        break;
    case VOLTAGE_C_AVG:
        title = "C Average Voltage";
        break;
    case VOLTAGE_C_FIL:
        title = "C Filtered Voltage";
        break;
    case VOLTAGE_S:
        title = "S Voltage";
        break;
    case VOLTAGE_TEMP:
        title = "Temperature Sensors Voltage";
        break;
    default:
        Error_Handler();
        break;
    }
    printfDma("%s: \n", title);

    printfDma("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printfDma("   %2d   |", i+1);
    }
    printfDma("  Sum   |  Delta |\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        uint8_t paddingOffset;
        paddingOffset = printfDma("| %2d |", ic);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%8.5f|", vArr[ic][c]);
        }

        printfDma("%8.5f|", ic_ad68.v_cell_sum  [voltageType][ic]);
        printfDma("%8.5f|", ic_ad68.v_cell_delta[voltageType][ic]);
        printfDma("\n");

        printfDma("%*s", paddingOffset, "");
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%8.5f|", vDev[ic][c]);
        }
        printfDma("\n");
    }
}


void bms_printTemps(void)
{
    float (*tArr)[TOTAL_CELL] = ic_ad68.temp_cell;

    printfDma("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printfDma("  %2d   |", i+1);
    }
    printfDma("  Die Temp / Segment Voltage");
    printfDma("\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printfDma("| %2d |", ic);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%6.1f |", tArr[ic][c]);
        }
        printfDma("  %.2f C  /  %.2f V \n", ic_ad68.temp_ic[ic], ic_ad68.v_segment[ic]);
    }
}


uint8_t* readCellVoltageCmdList[TOTAL_VOLTAGE_TYPES][6] = {
        {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF}, // VOLTAGE_TEMP
        {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF}, // VOLTAGE_C_AVG
        {RDFCA, RDFCB, RDFCC, RDFCD, RDFCE, RDFCF}, // VOLTAGE_FIL
        {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF}, // VOLTAGE_S
        {} // VOLTAGE_TEMP
};

BMS_StatusTypeDef bms_readCellVoltage(VoltageTypes voltageType)
{
    uint8_t** cmdList = readCellVoltageCmdList[voltageType];

    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }
        bms_parseVoltage(rxData, ic_ad68.v_cell[voltageType], i);
    }

    bms_calculateStats(voltageType);

    if (DEBUG_SERIAL_VOLTAGE_ENABLED) bms_printVoltage(voltageType);

    return BMS_OK;
}


uint8_t bms_getAuxVoltage(uint8_t muxIndex)
{
    uint8_t* cmdList[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA};

    for (int i = 0; i < 5; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return -1;
        }
        bms_parseAuxVoltage(rxData, ic_ad68.v_cell[VOLTAGE_TEMP], i, muxIndex);
    }
    return 0;
}


float static convertCellTemp(float cellVoltage)
{
    // From datasheet
    static const float tempValues[]    = { -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5,   10,   15,   20,   25,   30,   35,   40,   45,   50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100,  105,  110,  115,  120};
    static const float voltageValues[] = {2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30};
    static const int   numDataPoints   = sizeof(tempValues) / sizeof(tempValues[0]);

    // Check if in range
    if (cellVoltage > 2.44 || cellVoltage < 1.30)
    {
        // Voltage out of range
        return 888.0;
    }

    int idx;
    for (idx = 0; idx < numDataPoints - 1; idx++)
    {
        if (cellVoltage > voltageValues[idx + 1]) break;
    }

    float x1 = voltageValues[idx];
    float x2 = voltageValues[idx + 1];
    float y1 = tempValues[idx];
    float y2 = tempValues[idx + 1];

    return y1 + (cellVoltage - x1) * (y2 - y1) / (x2 - x1);
}


void bms_parseTemps(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            ic_ad68.temp_cell[ic][c] = convertCellTemp(ic_ad68.v_cell[VOLTAGE_TEMP][ic][c]);
        }
    }
}


BMS_StatusTypeDef bms_getAuxMeasurement(void)
{
    /*
     *  GPO4 = Mux switch
     *  GPO5 = 3V3 Converter Enable
     *
     *  20ms start-up time based on TEC 2-4810WI datasheet
     */

    ADAX.OW   = 0b0;
    ADAX.CH   = 0b0000;
    ADAX.CH4  = 0b0;
    ADAX.PUP  = 0b0;

//    bms_startTimer();
//    bms_wakeupChain();

    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);
    if (bms_getAuxVoltage(1))           // Has to be 1 first due to the mux switching
    {
        return BMS_ERR_COMMS;
    }
    bms68_setGpo45(0b00);           // Switch to the other Mux Channel
    bms_delayMsActive(1);           // Small delay for switching

    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);
    if (bms_getAuxVoltage(0))
    {
        return BMS_ERR_COMMS;
    }
    bms68_setGpo45(0b11);           // Reset to default

    bms_parseTemps();
    bms_calculateStats(VOLTAGE_TEMP);
    if (DEBUG_SERIAL_VOLTAGE_ENABLED)   bms_printVoltage(VOLTAGE_TEMP);
    if (DEBUG_SERIAL_AUX_ENABLED)      bms_printTemps();

//    uint32_t time = bms_getTimCount();
//    bms_stopTimer();
//    printfDma("PT: %ld us\n", time);

    return BMS_OK;
}


void bms_setPwm(uint8_t ic_index, uint8_t cell, uint8_t dutyCycle)
{
    cell++;                                 // Change from 0 indexing to 1 indexing

    switch (cell) {
        case 1:
            ic_ad68.pwma[ic_index].pwm1 = dutyCycle;
            break;
        case 2:
            ic_ad68.pwma[ic_index].pwm2 = dutyCycle;
            break;
        case 3:
            ic_ad68.pwma[ic_index].pwm3 = dutyCycle;
            break;
        case 4:
            ic_ad68.pwma[ic_index].pwm4 = dutyCycle;
            break;
        case 5:
            ic_ad68.pwma[ic_index].pwm5 = dutyCycle;
            break;
        case 6:
            ic_ad68.pwma[ic_index].pwm6 = dutyCycle;
            break;
        case 7:
            ic_ad68.pwma[ic_index].pwm7 = dutyCycle;
            break;
        case 8:
            ic_ad68.pwma[ic_index].pwm8 = dutyCycle;
            break;
        case 9:
            ic_ad68.pwma[ic_index].pwm9 = dutyCycle;
            break;
        case 10:
            ic_ad68.pwma[ic_index].pwm10 = dutyCycle;
            break;
        case 11:
            ic_ad68.pwma[ic_index].pwm11 = dutyCycle;
            break;
        case 12:
            ic_ad68.pwma[ic_index].pwm12 = dutyCycle;
            break;
        case 13:
            ic_ad68.pwmb[ic_index].pwm13 = dutyCycle;
            break;
        case 14:
            ic_ad68.pwmb[ic_index].pwm14 = dutyCycle;
            break;
        case 15:
            ic_ad68.pwmb[ic_index].pwm15 = dutyCycle;
            break;
        case 16:
            ic_ad68.pwmb[ic_index].pwm16 = dutyCycle;
            break;
        default:
            // Handle invalid cases
            break;
    }
}


/*
 * Converts delta threshold (threshold difference between all cell voltages)
 * to
 * discharge threshold (voltage at which cell stop discharging)
 */
float bms_calculateBalancing(float deltaThreshold)
{
    float min = ic_common.v_pack_min;
    float max = ic_common.v_pack_max;

    if (max - min > deltaThreshold)
    {
        return min + deltaThreshold;
    }
    else
    {
        return 1000;  // No balancing needed set the voltage threshold very high
    }
}


void bms_startDischarge(float dischargeThreshold)
{
//    dischargeThreshold = 5;  // Overwrite the discharge aim voltage (for testing)
    uint32_t cellDischargeCount;                    // Keep count of how many cells will be discharged per segment
    uint8_t dutyCycle = 0;                          // 4 bit pwm at 937 ms
    const uint8_t maxDischarge = (0b0111 * 16);    // All cells discharge at half duty

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        cellDischargeCount = 0;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                cellDischargeCount++;
            }
        }

        if (cellDischargeCount == 0) {cellDischargeCount = 1; };

        dutyCycle = maxDischarge / cellDischargeCount;
        if (dutyCycle > 0b1111)
        {
            dutyCycle = 0b1111;
        }

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                printfDma("DISCHARGE: IC %d, CELL %d, DC %d \n", ic+1, c+1, dutyCycle);
                BIT_SET(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, c, dutyCycle);
            }
            else
            {
                BIT_CLEAR(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, c, 0b0000);    // Turn off PWM discharge for that cell
            }
        }

        // The PWM discharge functionality is possible in the standby, REF-UP, extended balancing and in the measure states
        // AND while the discharge timeout has not expired (DCTO ≠ 0)
        ic_ad68.cfb_Tx[ic].dcto = 1;     // DC Timer in minutes (DTRNG = 0)
        ic_ad68.cfb_Tx[ic].dtmen = 0;    // Disables Discharge Timer Monitor (DTM)
        // ic_ad68[0].cfb_Tx.dcc = 0b1; // --- High priority discharge (bypasses PWM)
    }

    // for testing -> enables discharge for cell 1
//    printfDma("DISCHARGE: IC 1, CELL 1 \n");
//    ic_ad68.pwma[0].pwm1 = 0b1111;

    bms_writeRegister(REG_CONFIG_B);             // Send the DCTO Timer config
    bms_writeRegister(REG_PWM_A);                // Send the PWM configs
    bms_writeRegister(REG_PWM_B);                // Send the PWM configs
}


void bms_stopDischarge(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            bms_setPwm(ic, c, 0b0000);    // Turn off PWM discharge for that cell
        }

        // The PWM discharge functionality is possible in the standby, REF-UP, extended balancing and in the measure states
        // AND while the discharge timeout has not expired (DCTO ≠ 0)
        ic_ad68.cfb_Tx[ic].dcto = 0;     // DC Timer in minutes (DTRNG = 0)
        ic_ad68.cfb_Tx[ic].dtmen = 0;    // Disables Discharge Timer Monitor (DTM)
    }
    bms_writeRegister(REG_CONFIG_B);             // Send the DCTO Timer config
    bms_writeRegister(REG_PWM_A);                // Send the PWM configs
    bms_writeRegister(REG_PWM_B);                // Send the PWM configs
}


void bms_softReset(void)
{
    bms_wakeupChain();
    bms_transmitCmd(SRST);      // Put all devices to sleep
    printfDma("\n  ---  SOFT RESET  ---  \n");
}


BMS_StatusTypeDef bms29_readVB(void)
{
    if (TOTAL_AD29)
    {
        bms_receiveData(RDVB, rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }

        ic_ad29.vb1 = *((int16_t *)(rxData[0] + 2)) *  0.000100 * 396.604395604;
        ic_ad29.vb2 = *((int16_t *)(rxData[0] + 4)) * -0.000085 * 751;
        printfDma("Pack Voltage: %fV, %fV  \n", ic_ad29.vb1, ic_ad29.vb2);
    }
    else
    {
        printfDma("Pack Voltage: (AD29 Disabled!) \n");
    }
    return BMS_OK;
}



BMS_StatusTypeDef bms29_readCurrent(void)
{
    if (TOTAL_AD29)
    {
        bms_receiveData(RDI, rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }
    //    bms_printRawData(rxData, rxCc);

        // microvolts
        int32_t i1v = 0;
        int32_t i2v = 0;

        i1v = ((uint32_t)rxData[0][0]) | ((uint32_t)rxData[0][1] << 8) | ((int32_t)rxData[0][2] << 16);
        i2v = ((uint32_t)rxData[0][3]) | ((uint32_t)rxData[0][4] << 8) | ((int32_t)rxData[0][5] << 16);

        if (i1v & (UINT32_C(1) << 23)) { i1v |= 0xFF000000; }; // Check the sign bit (24th bit) and extend the sign
        if (i2v & (UINT32_C(1) << 23)) { i2v |= 0xFF000000; };

        const float SHUNT_RESISTANCE = 0.000050; // 50 microOhms

        ic_ad29.current1 = ((float)i1v / -1000000.0f) / SHUNT_RESISTANCE;
        ic_ad29.current2 = ((float)i2v /  1000000.0f) / SHUNT_RESISTANCE;

        printfDma("Current: %fA, %fA  \n", ic_ad29.current1 , ic_ad29.current2);
    }
    else
    {
        printfDma("Current: (AD29 Disabled!) \n");
    }
    return BMS_OK;
}



BMS_StatusTypeDef bms_balancingMeasureVoltage(void)
{
    // 6830
    // ADSV For triggering single shot S conversion (stops PWM) while C in unaffected
    // So this stops PWM and wait for S to finish
    // Then read the S voltage
    // Potential improvement: S vs C ADC comparison

    ADSV.CONT = 0;      // Continuous
    ADSV.DCP  = 0;      // Discharge permitted
    ADSV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    bms_transmitCmd((uint8_t *)&ADSV);

    bms_transmitPoll(PLSADC);
    if (bms_readCellVoltage(dischargeVoltageType))
    {
        return BMS_ERR_COMMS;
    }

    return BMS_OK;
}


void bms_startBalancing(float deltaThreshold)
{
    float dischargeThreshold = bms_calculateBalancing(deltaThreshold);
    bms_startDischarge(dischargeThreshold);
}


void BMS_GetCanData(CanTxMsg** buff, uint32_t* len)
{
    uint32_t bufferlen = 0;
    uint32_t id = BASE_CAN_ID;
    FDCAN_TxHeaderTypeDef txHeader;

    /* Prepare Tx Header */
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_EXTENDED_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = 8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        int32_t v_segment       = ic_ad68.v_segment[ic] * 1000;
        int16_t temp_ic         = ic_ad68.temp_ic[ic]   * 100;
        uint8_t isCommsError    = ic_common.isCommsError[ic+TOTAL_AD29];
        uint8_t isFaultDetected = ic_common.isFaultDetected[ic+TOTAL_AD29];

        memcpy(&canTxBuffer[bufferlen].data[0], &v_segment, 4);
//        canTxBuffer[bufferlen].data[4] = (uint8_t)(temp_ic & 0xFF);
//        canTxBuffer[bufferlen].data[5] = (uint8_t)((temp_ic >> 8) & 0xFF);
        memcpy(&canTxBuffer[bufferlen].data[4], &temp_ic, 2);
        canTxBuffer[bufferlen].data[6] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));

        txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + ic;
        canTxBuffer[bufferlen].header = txHeader;
        bufferlen++;

        if (isCommsError)
        {
            continue; // Does not send cell voltage and temp as its invalid
        }

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            int16_t cellVoltage = (int16_t)(ic_ad68.v_cell[dischargeVoltageType][ic][c] * 1000);
            int16_t voltageDiff = (int16_t)(ic_ad68.v_cell_diff[dischargeVoltageType][ic][c] * 1000);
            int16_t cellTemp    = (int16_t)(ic_ad68.temp_cell[ic][c] * 100);
            uint8_t isDischarging       = ((ic_ad68.isDischarging[ic]       >> c) & 0x01U);
            uint8_t isCellFaultDetected = ((ic_ad68.isCellFaultDetected[ic] >> c) & 0x01U);

            canTxBuffer[bufferlen].data[0] = (uint8_t)(cellVoltage & 0xFF);
            canTxBuffer[bufferlen].data[1] = (uint8_t)((cellVoltage >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[2] = (uint8_t)(voltageDiff & 0xFF);
            canTxBuffer[bufferlen].data[3] = (uint8_t)((voltageDiff >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[4] = (uint8_t)(cellTemp & 0xFF);
            canTxBuffer[bufferlen].data[5] = (uint8_t)((cellTemp >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[6] = (uint8_t)((isDischarging << 0) | (isCellFaultDetected << 1));

            uint32_t id_cell_offset = ic*TOTAL_CELL + c;

            txHeader.Identifier = id + id_cell_offset;
            canTxBuffer[bufferlen].header = txHeader;
            bufferlen++;
        }
    }

    if (TOTAL_AD29)
    {
        uint8_t isCommsError    = ic_common.isCommsError[0];
        uint8_t isFaultDetected = ic_common.isFaultDetected[0];

        canTxBuffer[bufferlen].data[0] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));
        txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + 7 + 1;
        canTxBuffer[bufferlen].header = txHeader;
        bufferlen++;

        if (!isCommsError)
        {
            int16_t packVoltage     = (ic_ad29.vb1 + ic_ad29.vb2) * 10 / 2;
            int16_t packCurrent     = (int16_t)(((ic_ad29.current1 + ic_ad29.current2) * 100.0f) / 2.0f);

            packVoltage = ic_common.v_pack_total * 10; // overwrite the packvoltage measurement from master

            canTxBuffer[bufferlen].data[0] = (packVoltage >> 0)  & 0xFF;
            canTxBuffer[bufferlen].data[1] = (packVoltage >> 8)  & 0xFF;

            canTxBuffer[bufferlen].data[2] = (packCurrent >> 0)  & 0xFF;
            canTxBuffer[bufferlen].data[3] = (packCurrent >> 8)  & 0xFF;

            txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + 7;
            canTxBuffer[bufferlen].header = txHeader;
            bufferlen++;
        }
    }

    // --- CHARGER CONFIG CAN MESSAGE --- //
    BMS_CAN_GetChargerMsg(&chargerConfig, canTxBuffer[bufferlen].data);
    txHeader.Identifier = CHARGER_CONFIG_CAN_ID;
    canTxBuffer[bufferlen].header = txHeader;
    bufferlen++;

    *len = bufferlen;
    *buff = canTxBuffer;

    newDataReady = false;
}


BMS_StatusTypeDef bms_checkStatus(void)
{
    // Check voltage
    // Check temp

    const float MAX_PACK_VOLTAGE = 4.2 * 16 * 7;
    const float MIN_PACK_VOLTAGE = 3.0 * 16 * 7;

    const float MAX_CURRENT = 10.0;
    const float MIN_CURRENT = -MAX_CURRENT;

    const float MAX_VOLTAGE = 4.2;
    const float MIN_VOLTAGE = 3.3;

    const float MAX_IC_VOLTAGE = 4.2 * 16;
    const float MIN_IC_VOLTAGE = 3.0 * 16;

    const float MAX_TEMP = 60;
    const float MIN_TEMP = 0;

    const float MAX_IC_TEMP = 70;
    const float MIN_IC_TEMP = 0;

    BMS_StatusTypeDef status = BMS_OK;
    BMS_StatusTypeDef returnStatus = BMS_OK;

    if (TOTAL_AD29)
    {
//        float packVoltage = ic_ad29.vb1;        // TODO: Figure out how to combine 2 values
        float packVoltage = ic_common.v_pack_total;
        float packCurrent = ic_ad29.current1;

        if (packVoltage > MAX_PACK_VOLTAGE)
        {
            printfDma("Pack Overvoltage Detected: %f V \n", packVoltage);
            ic_common.isFaultDetected[0] = true;
            status = BMS_ERR_FAULT;
        }

        if (packVoltage < MIN_PACK_VOLTAGE)
        {
            printfDma("Pack Undervoltage Detected: %f V \n", packVoltage);
            ic_common.isFaultDetected[0] = true;
            status = BMS_ERR_FAULT;
        }

        if (packCurrent > MAX_CURRENT)
        {
            printfDma("Pack OverTemp Detected: %f C \n", packCurrent);
            ic_common.isFaultDetected[0] = true;
            status = BMS_ERR_FAULT;
        }

        if (packCurrent < MIN_CURRENT)
        {
            printfDma("Pack UnderTemp Detected: %f C \n", packCurrent);
            ic_common.isFaultDetected[0] = true;
            status = BMS_ERR_FAULT;
        }

        if (status == BMS_OK)
        {
            ic_common.isFaultDetected[0] = false;
        }

        returnStatus |= status;
        status = BMS_OK;
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            float cellVoltage = ic_ad68.v_cell[dischargeVoltageType][ic][c];
            float cellTemp = ic_ad68.temp_cell[ic][c];

            if (cellVoltage > MAX_VOLTAGE)
            {
                printfDma("Overvoltage Detected: SEG %d, CELL %d, %f \n", ic+1, c+1, cellVoltage);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status = BMS_ERR_FAULT;
            }

            if (cellVoltage < MIN_VOLTAGE)
            {
                printfDma("Undervoltage Detected: SEG %d, CELL %d, %f \n", ic+1, c+1, cellVoltage);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status = BMS_ERR_FAULT;
            }

            if (cellTemp > MAX_TEMP)
            {
                printfDma("OverTemp Detected: SEG %d, CELL %d, %f \n", ic+1, c+1, cellTemp);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status = BMS_ERR_FAULT;
            }

            if (cellTemp < MIN_TEMP)
            {
                printfDma("UnderTemp Detected: SEG %d, CELL %d, %f \n", ic+1, c+1, cellTemp);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status = BMS_ERR_FAULT;
            }

            if (status == BMS_OK)
            {
                BIT_CLEAR(ic_ad68.isCellFaultDetected[ic], c);
            }

            returnStatus |= status;
            status = BMS_OK;
        }

        float icVoltage = ic_ad68.v_segment[ic];
        float icTemp = ic_ad68.temp_ic[ic];

        if (icVoltage > MAX_IC_VOLTAGE)
        {
            printfDma("IC Overvoltage Detected: SEG %d, %f \n", ic+1, icVoltage);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status = BMS_ERR_FAULT;
        }

        if (icVoltage < MIN_IC_VOLTAGE)
        {
            printfDma("IC Undervoltage Detected: SEG %d, %f \n", ic+1, icVoltage);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status = BMS_ERR_FAULT;
        }

        if (icTemp > MAX_IC_TEMP)
        {
            printfDma("IC OverTemp Detected: SEG %d, %f \n", ic+1, icTemp);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status = BMS_ERR_FAULT;
        }

        if (icTemp < MIN_IC_TEMP)
        {
            printfDma("IC UnderTemp Detected: SEG %d, %f \n", ic+1, icTemp);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status = BMS_ERR_FAULT;
        }

        if (status == BMS_OK)
        {
            ic_common.isFaultDetected[ic + TOTAL_AD29] = false;
        }

        returnStatus |= status;
        status = BMS_OK;
    }
    return returnStatus;
}



BMS_StatusTypeDef BMS_ProgramLoop(void)
{
    BMS_StatusTypeDef status;
//    bms_wakeupChain();
//    if ((status = bms_readCellVoltage(VOLTAGE_C_FIL)))  return status;
    bms_wakeupChain();
    if ((status = bms_getAuxMeasurement())) return status; // around 40 ms

    bms_wakeupChain();
    if ((status = bms29_readVB()))      return status;
    bms_wakeupChain();
    if ((status = bms29_readCurrent())) return status;
    bms_wakeupChain();
    if ((status = bms_balancingMeasureVoltage()))       return status;

    // Only balancing/charging if status is OK
    status = bms_checkStatus();
//    status = BMS_OK;              // Uncomment to bypass status check

    if (enableBalancing && (status == BMS_OK))
    {
        bms_wakeupChain();
        bms_startBalancing(balancingThreshold);
    }

    bms_wakeupChain();
    newDataReady = true;
    return status;
}


void BMS_EnableBalancing(bool enabled)
{
    enableBalancing = enabled;
}

void BMS_EnableCharging(bool enabled)
{
    chargerConfig.disable_charging = !enabled;   // Inverted logic because config is disable = 1

    char *state = (chargerConfig.disable_charging)? "Disabled" : "Enabled";
    printfDma("Charger Status: %s\n", state);
}

void BMS_ToggleBalancing(void)
{
    enableBalancing = !enableBalancing;
}


void BMS_ChargingButtonLogic(void)
{
    bool chargerEnabled = !chargerConfig.disable_charging;

    if (chargerEnabled)
    {
        BMS_EnableCharging(false);
        return;
    }

    bool statusOK = true;
    if (chargerStatus.hardware_fault != false)      statusOK = false;
    if (chargerStatus.over_temp_fault != false)     statusOK = false;
    if (chargerStatus.input_voltage_fault != false) statusOK = false;
    if (chargerStatus.output_voltage < 300.0f)      statusOK = false;

    if (statusOK)
    {
        BMS_EnableCharging(true);
    }
    else
    {
        printfDma("Charger NOT OK to start Charging \n");
    }
}

bool BMS_CheckNewDataReady(void)
{
    return newDataReady;
}



