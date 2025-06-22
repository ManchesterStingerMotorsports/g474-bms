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


uint8_t  txData[TOTAL_IC][DATA_LEN];
uint8_t  rxData[TOTAL_IC][DATA_LEN];
uint16_t rxPec[TOTAL_IC];
uint8_t  rxCc[TOTAL_IC];

uint32_t errorCount = 0;
uint32_t errorCount_Alltime = 0;

VoltageTypes dischargeVoltageType = VOLTAGE_S;

#define CAN_BUFFER_LEN (7 * 16 + 32)       // TODO: Accurate buffer size
CanTxMsg canTxBuffer[CAN_BUFFER_LEN];


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
    float v_cell_diff[TOTAL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL];
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
    bool isCommsError               [TOTAL_AD68];

} Ic_ad68;


Ic_ad29 ic_ad29;
Ic_ad68 ic_ad68;



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
    memcpy(&ic_ad29.cfa_Rx, &ad29_cfaDefault, DATA_LEN);
    memcpy(&ic_ad29.cfb_Rx, &ad29_cfbDefault, DATA_LEN);

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(&ic_ad68.cfa_Tx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ic_ad68.cfb_Tx[ic], &ad68_cfbDefault, DATA_LEN);
    }

    ad68_cfa_t ad68_cfaT;
    memcpy(&ad68_cfaT, &ad68_cfaDefault, DATA_LEN);
}


void bms_writeConfigA(void)
{
    // Fill buffer for ad2950 first
    memcpy(txData[0], &ic_ad29.cfa_Tx, DATA_LEN);

    // Fill buffer with the other ad6830 data
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic+1], &ic_ad68.cfa_Tx[ic], DATA_LEN);
    }

    // write config A
    bms_transmitData(WRCFGA, txData);
}


void bms_writeConfigB(void)
{
    // Fill buffer for ad2950 first
    memcpy(txData[0], &ic_ad29.cfb_Tx, DATA_LEN);

    // Fill buffer with the other ad6830 data
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic+1], &ic_ad68.cfb_Tx[ic], DATA_LEN);
    }

    // write config B
    bms_transmitData(WRCFGB, txData);
}


void bms_writePwmA(void)
{
    // Fill padding bytes for ad29
    memset(txData[0], 0x00, DATA_LEN);

    // Fill buffer with the other ad6830 data
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic+1], &ic_ad68.pwma[ic], DATA_LEN);
    }

    // write config A
    bms_transmitData(WRPWM1, txData);
}


void bms_writePwmB(void)
{
    // Fill padding bytes for ad29
    memset(txData[0], 0x00, DATA_LEN);

    // Fill buffer with the other ad6830 data
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(txData[ic+1], &ic_ad68.pwmb[ic], DATA_LEN);
    }

    // write config B
    bms_transmitData(WRPWM2, txData);
}


void bms_init(void)
{
    bms_resetConfig();

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].refon = 0b1;
        ic_ad68.cfa_Tx[ic].fc = 0b001;    // 110 Hz corner freq
    }

    bms_writeConfigA();
}


void bms68_setGpo45(uint8_t twoBitIndex)
{
    // GPIO Output: 1 = No pulldown (Default), 0 = Pulldown
    // Only for pin 4 and 5
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].gpo1to8 = ((twoBitIndex) << 3) | (0xFF ^ (0x3 << 3));
    }

    bms_writeConfigA();
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


//void bms_pecErrorHandler(uint8_t err)
//{
//    if (err == 0)
//    {
//       return;
//    }
//    printfDma("WARNING! PEC ERROR - IC: %d", err);
//    errorCount++;
//    errorCount_Alltime++;
//
//    if (errorCount >= 3)
//    {
//        bms_resetSequnce();
//    }
//    else
//    {
//        bms_retryComms();
//    }
//}
//
//uint8_t bms_checkComms(void)
//{
//    bms_softReset();
//    bms_readSid();
//}

bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    bool faultDetected = false;
    bool errorIndex[TOTAL_IC];

    if (!bms_checkRxPec(data, pec, cc, errorIndex))
    {
        printfDma("WARNING! PEC ERROR - IC:");
        for(int ic = 0; ic < TOTAL_IC; ic++)
        {
            if (!errorIndex[ic])
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
void bms_readSid(void)
{
    bms_receiveData(RDSID, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        printfDma("SID: \n");
        bms_printRawData(rxData, rxCc);
    }
}


void bms_readConfigA(void)
{
    bms_receiveData(RDCFGA, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        printfDma("CFGA: \n");
        bms_printRawData(rxData, rxCc);
    }
}


void bms_readConfigB(void)
{
    bms_receiveData(RDCFGB, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        printfDma("CFGB: \n");
        bms_printRawData(rxData, rxCc);
    }
}


void bms_startAdcvCont(void)
{
    // 6830
    // For DCP = 0
    // If RD = 0 and CONT = 1, PWM discharge is permitted
    // If RD = 1 and CONT = 0, PWM discharge interrupted temporarily until RD conversion finished (8ms typ)
    // If RD = 1 and CONT = 1, PWM discharge stopped

    ADCV.CONT = 1;      // Continuous
    ADCV.RD   = 0;      // Redundant Measurement
    ADCV.DCP  = 0;      // Discharge permitted
    ADCV.RSTF = 1;      // Reset filter
    ADCV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    // Behaviour of 2950 (ADI1 Command)
    //

    bms_transmitCmd((uint8_t *)&ADCV);
}


void bms_parseVoltage(uint8_t rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_IC][TOTAL_CELL], uint8_t register_index)
{
    // Does not take care of 2950
    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        uint8_t cell_index = (register_index * 3);

        for (int c = cell_index; c < (cell_index + 3); c++)
        {
            vArr[ic-1][c] = *((int16_t *)(rawData[ic] + (c - cell_index)*2)) * 0.00015 + 1.5;

            if (cell_index == 5)
            {
                break;
            }
        }
    }
}


void bms_parseAuxVoltage(uint8_t const rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_AD68][TOTAL_CELL], uint8_t cell_index, uint8_t muxIndex)
{
    // Does not take care of 2950
    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        if (cell_index == 4)
        {
            ic_ad68.temp_ic[ic-1] = (*((int16_t *)(rawData[ic] + 2)) * 0.00015 + 1.5) / 0.0075 - 273;
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

            vArr[ic-1][ci + 8*muxIndex] = *((int16_t *)(rawData[ic] + (c-cellArrIndex)*2)) * 0.00015 + 1.5;

            if (cell_index == 3)
            {
                ic_ad68.v_segment[ic-1] = (*((int16_t *)(rawData[ic] + 4)) * 0.00015 + 1.5) * 25;
                break;
            }
        }
    }
}


void bms_calculateStats(VoltageTypes voltageType)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float min = 999.0;
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
        }

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

void bms_readCellVoltage(VoltageTypes voltageType)
{
    uint8_t** cmdList = readCellVoltageCmdList[voltageType];

    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return;
        }
        bms_parseVoltage(rxData, ic_ad68.v_cell[voltageType], i);
    }

    bms_calculateStats(voltageType);
    bms_printVoltage(voltageType);
}


void bms_getAuxVoltage(uint8_t muxIndex)
{
    uint8_t* cmdList[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA};

    for (int i = 0; i < 5; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return;
        }
        bms_parseAuxVoltage(rxData, ic_ad68.v_cell[VOLTAGE_TEMP], i, muxIndex);
    }
}


float convertCellTemp(float cellVoltage)
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


void bms_getAuxMeasurement(void)
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

    bms_wakeupChain();

    // TODO: CHANGE THE NAME ITS 54 SHOULD
//    bms68_setGpo45(0b11);           // Enable 3V3 Converter
//    bms_delayMsActive(25);          // 20ms start-up time based on TEC 2-4810WI datasheet

    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);
    bms_getAuxVoltage(1);

    bms68_setGpo45(0b00);           // Switch to the other Mux Channel
    bms_delayMsActive(1);           // Small delay for switching

    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);
    bms_getAuxVoltage(0);

    bms68_setGpo45(0b11);           // Reset to default

    bms_parseTemps();
    bms_calculateStats(VOLTAGE_TEMP);
    bms_printVoltage(VOLTAGE_TEMP);
    bms_printTemps();

//    uint32_t time = bms_getTimCount();
//    bms_stopTimer();
//    printfDma("PT: %ld us\n", time);
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
    float min = 999.0;
    float max = -999.0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float segment_min = ic_ad68.v_cell_min[dischargeVoltageType][ic];
        float segment_max = ic_ad68.v_cell_max[dischargeVoltageType][ic];

        if (segment_min < min)
        {
            min = segment_min;
        }
        if (segment_max > max)
        {
            max = segment_max;
        }
    }

    if (max - min > deltaThreshold)
    {
        return min + deltaThreshold;
    }
    else
    {
        return -1;  // No balancing needed
    }
}


void bms_startDischarge(float dischargeThreshold)
{
    dischargeThreshold = 5;  // Overwrite the discharge aim voltage (for testing)
    const uint8_t dutyCycle = 0b1111;   // 4 bit pwm at 937 ms

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                printfDma("DISCHARGE: IC %d, CELL %d \n", ic+1, c+1);
                BIT_SET(ic_ad68.isDischarging[ic], c);
                bms_setPwm(ic, c, dutyCycle);
            }
            else
            {
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
    printfDma("DISCHARGE: IC 1, CELL 1 \n");
    ic_ad68.pwma[0].pwm1 = 0b1111;

    bms_writeConfigB();             // Send the DCTO Timer config
    bms_writePwmA();                // Send the PWM configs
    bms_writePwmB();                // Send the PWM configs
}


void bms_stopDischarge(void)
{
    // TODO: Stop discharege without resetting
    bms_softReset();
}


void bms_softReset(void)
{
    bms_wakeupChain();
    bms_transmitCmd(SRST);      // Put all devices to sleep
    printfDma("\n  ---  SOFT RESET  ---  \n");
}


void bms29_setGpo(void)
{
    ic_ad29.cfa_Tx.gpo1c  = 1;      // State control
    ic_ad29.cfa_Tx.gpo1od = 0;      // 1 = Open drain, 0 = push-pull
    ic_ad29.cfa_Tx.gpo2c  = 1;      // State control
    ic_ad29.cfa_Tx.gpo2od = 0;      // 1 = Open drain, 0 = push-pull

    bms_writeConfigA();
}


void bms29_readVB(void)
{
    bms_receiveData(RDVB, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
//    bms_printRawData(rxData, rxCc);
    ic_ad29.vb1 = *((int16_t *)(rxData[0] + 2)) *  0.000100 * 396.604395604;
    ic_ad29.vb2 = *((int16_t *)(rxData[0] + 4)) * -0.000085 * 751;
    printfDma("VB: %fV, %fV  \n\n", ic_ad29.vb1, ic_ad29.vb2);
}



void bms29_readCurrent(void)
{
    bms_receiveData(RDI, rxData, rxPec, rxCc);
    bms_checkRxFault(rxData, rxPec, rxCc);
//    bms_printRawData(rxData, rxCc);

    // microvolts
    int32_t i1v = 0;
    int32_t i2v = 0;

    i1v = ((uint32_t)rxData[0][0]) | ((uint32_t)rxData[0][1] << 8) | ((int32_t)rxData[0][2] << 16);
    i2v = ((uint32_t)rxData[0][3]) | ((uint32_t)rxData[0][4] << 8) | ((int32_t)rxData[0][5] << 16);

    if (i1v & (UINT32_C(1) << 23)) { i1v |= 0xFF000000; }; // Check the sign bit (24th bit) and extend the sign
    if (i2v & (UINT32_C(1) << 23)) { i2v |= 0xFF000000; };

    const float SHUNT_RESISTANCE = 0.000050; // 50 microOhms

    ic_ad29.current1 = ((float)i1v / 1000000.0f) / SHUNT_RESISTANCE;
    ic_ad29.current2 = ((float)i2v / 1000000.0f) / SHUNT_RESISTANCE;

    printfDma("Current: %fA, %fA  \n\n", ic_ad29.current1 , ic_ad29.current2);
}



void bms_balancingMeasureVoltage(void)
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
    bms_readCellVoltage(dischargeVoltageType);
}


void bms_startBalancing(float deltaThreshold)
{
    float dischargeThreshold = bms_calculateBalancing(deltaThreshold);

    if (dischargeThreshold > 0)
    {
        bms_startDischarge(dischargeThreshold);
    }
}



void BMS_GetCanData(CanTxMsg** buff, uint32_t* len)
{
    *len = 0;
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

    int16_t cellVoltage = 0;
    int16_t cellTemp = 0;
    uint8_t txData[8] = {0};
    uint8_t isDischarging = false;
    uint8_t isFaultDetected = false;
    int32_t packVoltage = 0;
    uint32_t packCurrent = (ic_ad29.current1 + ic_ad29.current2) * 1000 / 2;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            cellVoltage = (int16_t)(ic_ad68.v_cell[VOLTAGE_S][ic][c] * 1000);
            cellTemp    = (int16_t)(ic_ad68.temp_cell[ic][c] * 100);
            isDischarging = ((ic_ad68.isDischarging[ic] >> 1U) & 0x01);

            packVoltage += (cellVoltage * 1000);

            txData[0] = (uint8_t)(cellVoltage & 0xFF);
            txData[1] = (uint8_t)((cellVoltage >> 8) & 0xFF);
            txData[2] = (uint8_t)(cellTemp & 0xFF);
            txData[3] = (uint8_t)((cellTemp >> 8) & 0xFF);
            txData[4] = (uint8_t)((isDischarging << 0) | (isFaultDetected << 1));

            uint32_t id_offset = ic*TOTAL_CELL + c;

            memcpy(canTxBuffer[id_offset].data, txData, 8);
            txHeader.Identifier = id + id_offset;
            canTxBuffer[id_offset].header = txHeader;

            *len += 1;
        }
    }

    uint32_t packOffset = TOTAL_AD68*TOTAL_CELL;

    canTxBuffer[packOffset].data[0] = (packVoltage >> 0)  & 0xFF;
    canTxBuffer[packOffset].data[1] = (packVoltage >> 8)  & 0xFF;
    canTxBuffer[packOffset].data[2] = (packVoltage >> 16) & 0xFF;
    canTxBuffer[packOffset].data[3] = (packVoltage >> 24) & 0xFF;

    canTxBuffer[packOffset].data[4] = (packCurrent >> 0)  & 0xFF;
    canTxBuffer[packOffset].data[5] = (packCurrent >> 8)  & 0xFF;
    canTxBuffer[packOffset].data[6] = (packCurrent >> 16) & 0xFF;
    canTxBuffer[packOffset].data[7] = (packCurrent >> 24) & 0xFF;

    txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL;
    canTxBuffer[packOffset].header = txHeader;

    *len += 1;
    *buff = canTxBuffer;
}

uint32_t BMS_LoopActive(void)
{
    return 0;
}

uint32_t BMS_LoopCharging(void)
{
    return 0;
}

uint32_t BMS_LoopIDLE(void)
{
    return 0;
}



