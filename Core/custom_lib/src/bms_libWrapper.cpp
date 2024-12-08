/*
 * bms_libWrapper.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#include "bms_libWrapper.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_cmdlist.h"

#include <string.h>
#include <stdio.h>
#include "main.h"


ad29_cfa_t ad29_cfaTx;
ad29_cfb_t ad29_cfbTx;

ad29_cfa_t ad29_cfaRx;
ad29_cfb_t ad29_cfbRx;

ad68_cfa_t ad68_cfaTx[TOTAL_IC-1];
ad68_cfb_t ad68_cfbTx[TOTAL_IC-1];

ad68_cfa_t ad68_cfaRx[TOTAL_IC-1];
ad68_cfb_t ad68_cfbRx[TOTAL_IC-1];

uint8_t  txData[TOTAL_IC * DATA_LEN];
uint8_t  rxData[TOTAL_IC * DATA_LEN];
uint16_t rxPec[TOTAL_IC];
uint8_t  rxCc[TOTAL_IC];

float avgCellV[TOTAL_IC-1][16];
float avgCellV_sum[TOTAL_IC-1];
float avgCellV_avg[TOTAL_IC-1];
float avgCellV_delta[TOTAL_IC-1];

float sVoltage[TOTAL_IC-1][16];

float tempSensV[TOTAL_IC-1][16];
float cellTemp[TOTAL_IC-1][16];
float segmentVoltage[TOTAL_IC-1];
float dieTemp[TOTAL_IC-1];



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
    memcpy(&ad29_cfaTx, &ad29_cfaDefault, DATA_LEN);
    memcpy(&ad29_cfbTx, &ad29_cfbDefault, DATA_LEN);

    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        memcpy(&ad68_cfaTx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ad68_cfbTx[ic], &ad68_cfbDefault, DATA_LEN);
    }

    ad68_cfa_t ad68_cfaT;
    memcpy(&ad68_cfaT, &ad68_cfaDefault, DATA_LEN);
}


void bms_init(void)
{
    bms_resetConfig();
}


// Fill in the tx buffer for the 2950 and 6830 daisy chain
void bms_setupTxBuffer(uint8_t ad29[DATA_LEN], uint8_t ad68[DATA_LEN])
{
    // Fill buffer for ad2950 first
    memcpy(txData, ad29, DATA_LEN);

    // Fill buffer with the other ad6830 data
    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        memcpy(txData + ic*DATA_LEN, ad68, DATA_LEN);
    }
}


void bms_writeConfigA(void)
{
    // write config A
    bms_setupTxBuffer((uint8_t *)&ad29_cfaTx, (uint8_t *)&ad68_cfaTx);
    bms_transmitData(WRCFGA, txData);
}


void bms_writeConfigB(void)
{
    // write config B
    bms_setupTxBuffer((uint8_t *)&ad29_cfbTx, (uint8_t *)&ad68_cfbTx);
    bms_transmitData(WRCFGB, txData);
}


void bms68_setGpo45(uint8_t twoBitIndex)
{
    // GPIO Output: 1 = No pulldown (Default), 0 = Pulldown
    // Only for pin 4 and 5
    ad68_cfaTx[0].gpo1to8  = ((twoBitIndex) << 3) | (0xFF ^ (0x3 << 3));
    bms_writeConfigA();
}



void bms_printRawData(uint8_t data[DATA_LEN * TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printf("IC%d: ", ic+1);
        for (int j = 0; j < 6; j++)              // For every byte recieved (6 bytes)
        {
            printf("0x%02X, ", data[j + ic*6]);  // Print each of the bytes
        }
        printf("CC: %d |   ", cc[ic]);
    }
    printf("\n\n");
}


bool bms_checkRxFault(uint8_t data[DATA_LEN * TOTAL_IC], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    bool faultDetected = false;
    bool errorIndex[TOTAL_IC];

    if (!bms_checkRxPec(data, pec, cc, errorIndex))
    {
        printf("WARNING! PEC ERROR - IC:");
        for(int ic = 0; ic < TOTAL_IC; ic++)
        {
            if (!errorIndex[ic])
            {
                printf(" %d,", ic+1);
            }
        }
        printf("\n");
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
    printf("SID: \n");
    bms_checkRxFault(rxData, rxPec, rxCc);
    bms_printRawData(rxData, rxCc);
}


void bms_readConfigA(void)
{
    bms_receiveData(RDCFGA, rxData, rxPec, rxCc);
    printf("CFGA: \n");
    bms_checkRxFault(rxData, rxPec, rxCc);
    bms_printRawData(rxData, rxCc);
}


void bms_readConfigB(void)
{
    bms_receiveData(RDCFGB, rxData, rxPec, rxCc);
    printf("CFGB: \n");
    bms_checkRxFault(rxData, rxPec, rxCc);
    bms_printRawData(rxData, rxCc);
}



void bms_startAdcvCont(void)
{
    ADCV.CONT = 1;      // Continuous
    ADCV.RD   = 1;      // Redundant Measurement
    ADCV.DCP  = 0;      // Discharge permitted
    ADCV.RSTF = 0;      // Reset filter
    ADCV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    bms_transmitCmd((uint8_t *)&ADCV);
}


void bms_parseVoltage(uint8_t rawData[TOTAL_IC * DATA_LEN], float vArr[TOTAL_IC-1][TOTAL_CELL], uint8_t cell_index)
{
    // Does not take care of 2950

    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        for (int c = cell_index*3; c < (cell_index*3 + 3); c++)
        {
            vArr[ic-1][c] = *((int16_t *)(rawData + ic*6 + (c-cell_index*3)*2)) * 0.00015 + 1.5;
            if (cell_index == 5)
            {
                break;
            }
        }
    }
}


void bms_parseAuxVoltage(uint8_t rawData[TOTAL_IC * DATA_LEN], float vArr[TOTAL_IC-1][TOTAL_CELL], uint8_t cell_index, uint8_t muxIndex)
{
    // Does not take care of 2950

    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        if (cell_index == 4)
        {
            dieTemp[ic-1] = (*((int16_t *)(rawData + ic*6 + 2)) * 0.00015 + 1.5) / 0.0075 - 273;
            continue;
        }

        uint8_t cellArrIndex = cell_index*3;

        for (int c = cellArrIndex; c < (cellArrIndex + 3); c++)
        {
            if (c == 3 || c == 4) continue; // Skip digital output pins
            int ci = c;
            if (c > 4)
            {
                ci -= 2;
            }

            vArr[ic-1][ci + 8*muxIndex] = *((int16_t *)(rawData + ic*6 + (c-cellArrIndex)*2)) * 0.00015 + 1.5;

            if (cell_index == 3)
            {
                segmentVoltage[ic-1] = (*((int16_t *)(rawData+10)) * 0.00015 + 1.5) * 25;
                break;
            }
        }
    }
}


void bms_calculateStats(void)
{
    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        float min = 999.0;
        float max = -999.0;
        float sum = 0;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            float voltage = avgCellV[ic][c];
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
        avgCellV_sum[ic]   = sum;
        avgCellV_avg[ic]   = sum / 16.0;
        avgCellV_delta[ic] = max - min;
    }
}


void bms_printVoltage(float vArr[TOTAL_IC-1][16])
{
    printf("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printf("   %2d   |", i+1);
    }
    printf("  Sum   |  Delta |\n");

    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        printf("| %2d |", ic);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printf("%8.5f|", vArr[ic][c]);
        }
        printf("%8.5f|",avgCellV_sum[ic]);
        printf("%8.5f|",avgCellV_delta[ic]);
        printf("\n");
    }
}


void bms_printTemps(float tArr[TOTAL_IC-1][16])
{
    printf("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printf("  %2d   |", i+1);
    }
    printf("\n");

    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        printf("| %2d |", ic);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printf("%6.1f |", tArr[ic][c]);
        }
        printf("\n");
    }
}


void bms_readAvgCellVoltage(void)
{
    uint8_t* cmdList[] = {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF};

    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return;
        }
        bms_parseVoltage(rxData, avgCellV, i);
    }
}


void bms_readSVoltage(void)
{
    uint8_t* cmdList[] = {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF};

    for (int i = 0; i < 6; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return;
        }
        bms_parseVoltage(rxData, sVoltage, i);
    }
}


void bms_getAuxVoltage(uint8_t muxIndex)
{
    uint8_t* cmdList[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA};

    for (int i = 0; i < 5; i++)
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
//        if (bms_checkRxFault(rxData, rxPec, rxCc))
//        {
//            return;
//        }
        bms_parseAuxVoltage(rxData, tempSensV, i, muxIndex);
    }

    if (muxIndex == 1)
    {
        muxIndex = 0;
    }
    else
    {
        muxIndex = 1;
    }
}


void bms_openWireCheck(void)
{
    ADSV.CONT = 0;      // Continuous
    ADSV.DCP  = 0;      // Discharge permitted
    ADSV.OW   = 0b00;   // Open wire on C-ADCS and S-ADCs

    bms_transmitPoll((uint8_t *)&ADSV);
    bms_readSVoltage();
    bms_printVoltage(sVoltage);
    bms_wakeupChain();

    bms_startTimer();

    ADSV.OW   = 0b10;   // Open wire on C-ADCS and S-ADCs
    bms_transmitPoll((uint8_t *)&ADSV);

    uint32_t time = bms_getTimCount();
    bms_stopTimer();

    printf("PT: %ld us\n", time);


    bms_delayMsActive(50);

    bms_readSVoltage();
    bms_printVoltage(sVoltage);
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
        return 777.7;
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
    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            cellTemp[ic][c] = convertCellTemp(tempSensV[ic][c]);
        }
    }
}


void bms_getAuxMeasurement(void)
{
    ADAX.OW   = 0b0;
    ADAX.CH   = 0b0000;
    ADAX.CH4  = 0b0;
    ADAX.PUP  = 0b0;

//    bms_startTimer();

    bms68_setGpo45(0b10);
    bms_delayMsActive(5);

    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);

    bms_getAuxVoltage(0);
//    bms_printVoltage(tempSensV);
//    bms_wakeupChain();

    bms68_setGpo45(0b01);
    bms_delayMsActive(5);


    bms_transmitCmd((uint8_t *)&ADAX);
    bms_transmitPoll(PLAUX1);

    bms_getAuxVoltage(1);
//    bms_printVoltage(tempSensV);
//    bms_wakeupChain();

    bms68_setGpo45(0b11);
    bms_delayMsActive(5);

    bms_parseTemps();
    bms_printTemps(cellTemp);

    printf("dieTemp: %f\n", dieTemp[0]);
    printf("SegVoltage: %f\n", segmentVoltage[0]);

//    uint32_t time = bms_getTimCount();
//    bms_stopTimer();
//    printf("PT: %ld us\n", time);
}




