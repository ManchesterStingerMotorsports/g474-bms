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

#include "string.h"
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


void bms68_toggleGpo(void)
{
    static bool gpo_on = false;

    if (gpo_on)
    {
        ad68_cfaTx[0].gpo1to8  = 0x00;
        ad68_cfaTx[0].gpo9to10 = 0b00;
        gpo_on = false;
    }
    else
    {
        ad68_cfaTx[0].gpo1to8  = 0xFF;
        ad68_cfaTx[0].gpo9to10 = 0b11;
        gpo_on = true;
    }

    bms_writeConfigA();
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
