

#pragma once

#include "common.h"

/// ----------------------- BOTH 2950 AND 6830 ------------------------------- ///

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



/// ----------------------- 2950 ONLY ------------------------------- ///

/*!< Read cell voltage result registers commands */
extern uint8_t RDI[2];
extern uint8_t RDVBAT[2];
extern uint8_t RDIVBAT[2];
extern uint8_t RDVA[2];
extern uint8_t RDVB[2];
extern uint8_t RDOCR[2];
extern uint8_t RDALLA[2];

/*!< Read Result Registers Commands B */
extern uint8_t RDIAV[2];
extern uint8_t RDVBAV[2];
extern uint8_t RDIVBAV[2];
extern uint8_t RDALLB[2];

/*!< Read Result Registers Commands C */
extern uint8_t RDRVA[2];
extern uint8_t RDRVB[2];
extern uint8_t RDVC[2];
extern uint8_t RDVD[2];
extern uint8_t RDCALL[2];

/*!< Pwm registers commands */
extern uint8_t WRPWMA[2];
extern uint8_t RDPWMA[2];
extern uint8_t WRPWMB[2];
extern uint8_t RDPWMB[2];

/*!< Clear commands */
extern uint8_t CLRAB[2];
extern uint8_t CLRC[2];
extern uint8_t CLRSTAT [2];

/*!< Poll adc command */
extern uint8_t PLI1ADC[2];
extern uint8_t PLI2ADC[2];
extern uint8_t PLVADC[2];
extern uint8_t PLAUX[2];



/// ----------------------- 6830 ONLY ------------------------------- ///

/* Read cell voltage result registers commands */
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
extern uint8_t WRPWM1[2];
extern uint8_t RDPWM1[2];
extern uint8_t WRPWM2[2];
extern uint8_t RDPWM2[2];

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


