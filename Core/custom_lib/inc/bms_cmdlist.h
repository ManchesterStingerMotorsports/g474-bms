/*
 * bms_cmdlist.h
 *
 *  Created on: Nov 24, 2024
 *      Author: amrlxyz
 */

#pragma once

#include "stdint.h"

// ------------------------------------- 2950 ------------------------------------- //

/* configuration registers commands */
extern uint8_t WRCFGA[2];
extern uint8_t WRCFGB[2];
extern uint8_t RDCFGA[2];
extern uint8_t RDCFGB[2];

/* Read VBxADC and IxADC result registers commands */
extern uint8_t RDI[2];
extern uint8_t RDVB[2];
extern uint8_t RDIVB1[2];

/* Read VxADC result registers commands */
extern uint8_t RDV1A[2];
extern uint8_t RDV1B[2];
extern uint8_t RDV1C[2];
extern uint8_t RDV1D[2];
extern uint8_t RDV2A[2];//RDRVA
extern uint8_t RDV2B[2];
extern uint8_t RDV2C[2];
extern uint8_t RDV2D[2];
extern uint8_t RDV2E[2];

/* Read OCxADC result registers commands */
extern uint8_t RDOC[2];

/* Read all commands */
extern uint8_t RDALLI[2];
extern uint8_t RDALLA[2];
extern uint8_t RDALLC[2];
extern uint8_t RDALLV[2];
extern uint8_t RDALLR[2];
extern uint8_t RDALLX[2];

/* Read Result Registers Commands B */
extern uint8_t RDIACC[2];
extern uint8_t RDVBACC[2];
extern uint8_t RDIVB1ACC[2];
extern uint8_t RDALLB[2];

/* Read Result Registers Commands C */
extern uint8_t RDCALL[2];

extern uint8_t RDSTAT[2];
extern uint8_t RDFLAG[2];
extern uint8_t RDFLAGERR[2];   /* ERR */

/* Read all Status Registers */
extern uint8_t RDASALL[2];

/* Clear commands */
extern uint8_t CLRI[2];
extern uint8_t CLRA[2];
extern uint8_t CLRO[2];
extern uint8_t CLRC[2];
extern uint8_t CLRVX [2];
extern uint8_t CLRFLAG[2];

/* Poll adc command */
extern uint8_t PLADC[2];
extern uint8_t PLI1[2];
extern uint8_t PLI2[2];
extern uint8_t PLV[2];
extern uint8_t PLX[2];

/* GPIOs Comm commands */
extern uint8_t WRCOMM[2];
extern uint8_t RDCOMM[2];
extern uint8_t STCOMM[13];

/* Control Commands */
extern uint8_t RDSID[2];
extern uint8_t RSTCC[2];
extern uint8_t SNAP[2];
extern uint8_t UNSNAP[2];
extern uint8_t SRST[2];

//Command + PEC
extern uint8_t RSTATD[4];
extern uint8_t RFLAG[4];
extern uint8_t sRDI[4];
extern uint8_t sRSTATA [4];
extern uint8_t sRDVA [4];
extern uint8_t sRDVB [4];
extern uint8_t sRDVC [4];
extern uint8_t sRDVD [4];
extern uint8_t sRDIAV[4];
extern uint8_t sRDVBAT[4];
extern uint8_t sADI1[2];
extern uint8_t sADX[2];

/* Testmode and debugging commands */
extern uint8_t TM_48[2];

/* Read Aux ADC result registers commands */
extern uint8_t RDXA[2];
extern uint8_t RDXB[2];
extern uint8_t RDXC[2];


// ------------------------------------- 6830 ------------------------------------- //

/*!< configuration registers commands */
extern uint8_t WRCFGA[2];
extern uint8_t WRCFGB[2];
extern uint8_t RDCFGA[2];
extern uint8_t RDCFGB[2];

/*!< Read status registers */
extern uint8_t RDSTATA[2];
extern uint8_t RDSTATB[2];
extern uint8_t RDSTATC[2];
extern uint8_t RDSTATCERR[2];   /* ERR */
extern uint8_t RDSTATD[2];
extern uint8_t RDSTATE[2];

/* Read all AUX and all Status Registers */
extern uint8_t RDASALL[2];

/*!< Reserved Read Commands - 2950 */
/* Read aux results */
extern uint8_t RDAUXC[2];
extern uint8_t RDAUXD[2];

/*!< Reserved Read Commands - 2950 */
/* Read redundant aux results */
extern uint8_t RDRAXC[2];
extern uint8_t RDRAXD[2];

/* Clear commands */
extern uint8_t CLRFLAG[2];

/* Poll adc command */
extern uint8_t PLADC[2];

/*!< GPIOs Comm commands */
extern uint8_t WRCOMM[2];
extern uint8_t RDCOMM[2];
/*!< command + dummy data for 72 clock cycles */
extern uint8_t STCOMM[13];

/*!< Control Commands */
extern uint8_t RDSID[2];  /* Read SID command */
extern uint8_t RSTCC[2];
extern uint8_t SNAP[2];
extern uint8_t UNSNAP[2];
extern uint8_t SRST[2];

extern uint8_t RDCVA[2];
extern uint8_t RDCVB[2];
extern uint8_t RDCVC[2];
extern uint8_t RDCVD[2];
extern uint8_t RDCVE[2];
extern uint8_t RDCVF[2];
extern uint8_t RDCVALL[2];

/* Read average cell voltage result registers commands commands */
extern uint8_t RDACA[2];
extern uint8_t RDACB[2];
extern uint8_t RDACC[2];
extern uint8_t RDACD[2];
extern uint8_t RDACE[2];
extern uint8_t RDACF[2];
extern uint8_t RDACALL[2];

/* Read s voltage result registers commands */
extern uint8_t RDSVA[2];
extern uint8_t RDSVB[2];
extern uint8_t RDSVC[2];
extern uint8_t RDSVD[2];
extern uint8_t RDSVE[2];
extern uint8_t RDSVF[2];
extern uint8_t RDSALL[2];

/* Read c and s results */
extern uint8_t RDCSALL[2];
extern uint8_t RDACSALL[2];

/* Read filtered cell voltage result registers*/
extern uint8_t RDFCA[2];
extern uint8_t RDFCB[2];
extern uint8_t RDFCC[2];
extern uint8_t RDFCD[2];
extern uint8_t RDFCE[2];
extern uint8_t RDFCF[2];
extern uint8_t RDFCALL[2];

/* Read aux results */
extern uint8_t RDAUXA[2];
extern uint8_t RDAUXB[2];

/* Read redundant aux results */
extern uint8_t RDRAXA[2];
extern uint8_t RDRAXB[2];

/* Pwm registers commands */
extern uint8_t WRPWMA[2];
extern uint8_t RDPWMA[2];
extern uint8_t WRPWMB[2];
extern uint8_t RDPWMB[2];

/* Clear commands */
extern uint8_t CLRCELL[2];
extern uint8_t CLRAUX [2];
extern uint8_t CLRSPIN[2];
extern uint8_t CLRFC[2];
extern uint8_t CLOVUV[2];

/* Poll adc command */
extern uint8_t PLAUT[2];
extern uint8_t PLCADC[2];
extern uint8_t PLSADC[2];
extern uint8_t PLAUX1[2];
extern uint8_t PLAUX2[2];

/* Diagn command */
extern uint8_t DIAGN[2];

/* Mute and Unmute commands */
extern uint8_t MUTE[2];
extern uint8_t UNMUTE[2];


//-----------------------------------------------------------//

// AD68 Cell Measurement

typedef struct
{
    uint8_t       RD  : 1;
    uint8_t       RES2: 2;
    uint8_t       X   : 5;

    uint8_t       OW  : 2;
    uint8_t       RSTF: 1;
    uint8_t       RES0: 1;
    uint8_t       DCP : 1;
    uint8_t       RES1: 2;
    uint8_t       CONT: 1;
} ADCV_t;

extern const ADCV_t ADCV_default;
extern ADCV_t ADCV;

typedef struct
{
    uint8_t       RES2: 3;
    uint8_t       X   : 5;

    uint8_t       OW  : 2;
    uint8_t       RES0: 2;
    uint8_t       DCP : 1;
    uint8_t       RES1: 2;
    uint8_t       CONT: 1;
} ADSV_t;

extern const ADSV_t ADSV_default;
extern ADSV_t ADSV;


// AD68 AUX ADC Measurement

typedef struct
{
    uint16_t      RES : 12;
    uint16_t      CH  :  4;
} ADAX2_t;

extern const ADAX2_t ADAX2_default;
extern ADAX2_t ADAX2;

typedef struct
{
    uint8_t       OW  : 1;
    uint8_t       RES0: 2;
    uint8_t       X   : 5;

    uint8_t       CH  : 4;
    uint8_t       RES1: 2;
    uint8_t       CH4 : 1;
    uint8_t       PUP : 1;
} ADAX_t;

extern const ADAX_t ADAX_default;
extern ADAX_t ADAX;














