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

    bms_startTimer();

    bms_transmitCmd((uint8_t *)&ADCV);

    uint32_t time = bms_getTimCount();
    bms_stopTimer();

    printf("Polling Time: %ld us\n", time);
}


void bms_parseVoltage(uint8_t rawData[TOTAL_IC * DATA_LEN], uint8_t cell_index)
{
    for (int ic = 1; ic < TOTAL_IC; ic++)
    {
        for (int c = cell_index*3; c < (cell_index*3 + 3); c++)
        {
            avgCellV[ic-1][c] = *((int16_t *)(rawData + ic*6)) * 0.00015 + 1.5;
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
        bms_parseVoltage(rxData, i);
    }
    bms_printVoltage(avgCellV);
}




//void adi2950_init_config(uint8_t tIC, cell_asic *ic)
//{
//  for(int cic=0; cic<tIC; cic++)
//  {
//    //CFGA
//    ic[cic].tx_cfga.gpo1c = PULLED_UP_TRISTATED;
//    ic[cic].tx_cfga.gpo2c = PULLED_UP_TRISTATED;
//    ic[cic].tx_cfga.gpo3c = PULLED_UP_TRISTATED;
//    ic[cic].tx_cfga.gpo4c = PULLED_UP_TRISTATED;
//    ic[cic].tx_cfga.gpo5c = PULLED_UP_TRISTATED;
//    ic[cic].tx_cfga.gpo6c = PULLED_UP_TRISTATED;
//
//    ic[cic].tx_cfga.gpo1od = OPEN_DRAIN;
//    ic[cic].tx_cfga.gpo2od = OPEN_DRAIN;
//    ic[cic].tx_cfga.gpo3od = OPEN_DRAIN;
//    ic[cic].tx_cfga.gpo4od = OPEN_DRAIN;
//    ic[cic].tx_cfga.gpo5od = OPEN_DRAIN;
//    ic[cic].tx_cfga.gpo6od = OPEN_DRAIN;
//
//    ic[cic].tx_cfga.vs1  = VSM_SGND;
//    ic[cic].tx_cfga.vs2  = VSM_SGND;
//    ic[cic].tx_cfga.vs3  = VSMV_SGND;
//    ic[cic].tx_cfga.vs4  = VSMV_SGND;
//    ic[cic].tx_cfga.vs5  = VSMV_SGND;
//    ic[cic].tx_cfga.vs6  = VSMV_SGND;
//    ic[cic].tx_cfga.vs7  = VSMV_SGND;
//    ic[cic].tx_cfga.vs8  = VSMV_SGND;
//    ic[cic].tx_cfga.vs9  = VSMV_SGND;
//    ic[cic].tx_cfga.vs10 = VSMV_SGND;
//
//    ic[cic].tx_cfga.injosc = INJOSC0_NORMAL;
//    ic[cic].tx_cfga.injmon = INJMON0_NORMAL;
//    ic[cic].tx_cfga.injts  = NO_THSD;
//    ic[cic].tx_cfga.injecc = NO_ECC;
//    ic[cic].tx_cfga.injtm  = NO_TMODE;
//
//    ic[cic].tx_cfga.soak    = SOAK_DISABLE;
//    ic[cic].tx_cfga.ocen    = OC_DISABLE;
//    ic[cic].tx_cfga.gpio1fe = FAULT_STATUS_DISABLE;
//    ic[cic].tx_cfga.spi3w   = FOUR_WIRE;
//
//    ic[cic].tx_cfga.acci    = ACCI_8;
//    ic[cic].tx_cfga.commbk  = COMMBK_OFF;
//    ic[cic].tx_cfga.vb1mux  = SINGLE_ENDED_SGND;
//    ic[cic].tx_cfga.vb2mux  = SINGLE_ENDED_SGND;
//
//    //CFGB
//    ic[cic].tx_cfgb.gpio1c = PULL_DOWN_OFF;
//    ic[cic].tx_cfgb.gpio2c = PULL_DOWN_OFF;
//    ic[cic].tx_cfgb.gpio3c = PULL_DOWN_OFF;
//    ic[cic].tx_cfgb.gpio4c = PULL_DOWN_OFF;
//
//    ic[cic].tx_cfgb.oc1th = 0x0;
//    ic[cic].tx_cfgb.oc2th = 0x0;
//    ic[cic].tx_cfgb.oc3th = 0x0;
//
//    ic[cic].tx_cfgb.oc1ten = NORMAL_INPUT;
//    ic[cic].tx_cfgb.oc2ten = NORMAL_INPUT;
//    ic[cic].tx_cfgb.oc3ten = NORMAL_INPUT;
//
//    ic[cic].tx_cfgb.ocdgt  = OCDGT0_1oo1;
//    ic[cic].tx_cfgb.ocdp   = OCDP0_NORMAL;
//    ic[cic].tx_cfgb.reften = NORMAL_INPUT;
//    ic[cic].tx_cfgb.octsel = OCTSEL0_OCxADC_P140_REFADC_M20;
//
//    ic[cic].tx_cfgb.ocod   = PUSH_PULL;
//    ic[cic].tx_cfgb.oc1gc  = GAIN_1;
//    ic[cic].tx_cfgb.oc2gc  = GAIN_1;
//    ic[cic].tx_cfgb.oc3gc  = GAIN_1;
//    ic[cic].tx_cfgb.ocmode = OCMODE0_DISABLED;
//    ic[cic].tx_cfgb.ocax   = OCABX_ACTIVE_HIGH;
//    ic[cic].tx_cfgb.ocbx   = OCABX_ACTIVE_HIGH;
//
//    ic[cic].tx_cfgb.diagsel   = DIAGSEL0_IAB_VBAT;
//    ic[cic].tx_cfgb.gpio2eoc  = EOC_DISABLED;
//  }
//}
