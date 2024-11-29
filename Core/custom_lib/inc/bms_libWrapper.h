/*
 * bms_libWrapper.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "bms_datatypes.h"
#include "main.h"

extern ad29_cfa_t ad29_cfaTx;
extern ad29_cfb_t ad29_cfbTx;

extern ad29_cfa_t ad29_cfaRx;
extern ad29_cfb_t ad29_cfbRx;

extern ad68_cfa_t ad68_cfaTx[TOTAL_IC-1];
extern ad68_cfb_t ad68_cfbTx[TOTAL_IC-1];

extern ad68_cfa_t ad68_cfaRx[TOTAL_IC-1];
extern ad68_cfb_t ad68_cfbRx[TOTAL_IC-1];

void bms_init(void);





void bms_readSid(void);

void bms_readConfigA(void);

void bms_readConfigB(void);




void bms68_toggleGpo4(void);






