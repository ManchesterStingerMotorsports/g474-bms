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
float sVoltage[TOTAL_IC-1][16];


void bms_resetConfig(void)
{
    // Obtained from RDCFG after reset
    // 0x00 added at LSB due to Little endian
    uint64_t const ad29_cfaDefault = 0x0000003F3F1100;
    uint64_t const ad29_cfbDefault = 0x0000000001F000;
    uint64_t const ad68_cfaDefault = 0x010000FF030000;
    uint64_t const ad68_cfbDefault = 0x00F87F00000000;

    // Copy defaults to Tx Buffer
    memcpy(&ad29_cfaTx, &ad29_cfaDefault, DATA_LEN);
    memcpy(&ad29_cfbTx, &ad29_cfbDefault, DATA_LEN);

    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        memcpy(&ad68_cfaTx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ad68_cfbTx[ic], &ad68_cfbDefault, DATA_LEN);
    }
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


void bms68_toggleGpo4(void)
{
    ad68_cfaTx[0].gpo1to8  ^= (1u << (4-1));
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
    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        for (int c = cell_index*3; c < (cell_index*3 + 3); c++)
        {
            vArr[ic-1][c] = *((int16_t *)(rawData + ic*6)) * 0.00015 + 1.5;
            if (cell_index == 5)
            {
                break;
            }
        }
    }
}


void bms_printVoltage(float vArr[TOTAL_IC-1][16])
{
    printf("| IC |");
    for (int i = 0; i < 16; i++)
    {
        printf("    %2d    |", i+1);
    }
    printf("\n");

    for (int ic = 0; ic < TOTAL_IC-1; ic++)
    {
        printf("| %2d |", ic);
        for (int c = 0; c < 16; c++)
        {
            printf(" %8.5f |", vArr[ic][c]);
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
//    bms_printVoltage(avgCellV);
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
        printf("CC: %d\n", rxCc[1]);
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




