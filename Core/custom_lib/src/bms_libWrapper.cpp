/*
 * bms_libWrapper.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#include "bms_libWrapper.h"




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
