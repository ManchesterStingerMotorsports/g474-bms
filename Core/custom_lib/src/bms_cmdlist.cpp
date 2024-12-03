

#include "bms_cmdlist.h"
#include "stdint.h"

/// ----------------------- BOTH 2950 AND 6830 ------------------------------- ///

/*!< configuration registers commands */
uint8_t WRCFGA[2]        = { 0x00, 0x01 };
uint8_t WRCFGB[2]        = { 0x00, 0x24 };
uint8_t RDCFGA[2]        = { 0x00, 0x02 };
uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/*!< Read status registers */
uint8_t RDSTATA[2]       = { 0x00, 0x30 };
uint8_t RDSTATB[2]       = { 0x00, 0x31 };
uint8_t RDSTATC[2]       = { 0x00, 0x32 };
uint8_t RDSTATCERR[2]    = { 0x00, 0x72 };   /* ERR */
uint8_t RDSTATD[2]       = { 0x00, 0x33 };
uint8_t RDSTATE[2]       = { 0x00, 0x34 };

/* Read all AUX and all Status Registers */
uint8_t RDASALL[2]       = { 0x00, 0x35 };

/*!< Reserved Read Commands - 2950 */
/* Read aux results */
uint8_t RDAUXC[2]        = { 0x00, 0x1B };
uint8_t RDAUXD[2]        = { 0x00, 0x1F };

/*!< Reserved Read Commands - 2950 */
/* Read redundant aux results */
uint8_t RDRAXC[2]        = { 0x00, 0x1E };
uint8_t RDRAXD[2]        = { 0x00, 0x25 };

/* Clear commands */
uint8_t CLRFLAG[2]       = { 0x07, 0x17 };

/* Poll adc command */
uint8_t PLADC[2]         = { 0x07, 0x18 };

/*!< GPIOs Comm commands */
uint8_t WRCOMM[2]        = { 0x07, 0x21 };
uint8_t RDCOMM[2]        = { 0x07, 0x22 };
/*!< command + dummy data for 72 clock cycles */
uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/*!< Control Commands */
uint8_t RDSID[2]         = { 0x00, 0x2C };  /* Read SID command */
uint8_t RSTCC[2]         = { 0x00, 0x2E };
uint8_t SNAP[2]          = { 0x00, 0x2D };
uint8_t UNSNAP[2]        = { 0x00, 0x2F };
uint8_t SRST[2]          = { 0x00, 0x27 };


/// ----------------------- 2950 ONLY ------------------------------- ///


/* configuration registers commands */
//uint8_t WRCFGA[2]        = { 0x00, 0x01 };
//uint8_t WRCFGB[2]        = { 0x00, 0x24 };
//uint8_t RDCFGA[2]        = { 0x00, 0x02 };
//uint8_t RDCFGB[2]        = { 0x00, 0x26 };

/* Read VBxADC and IxADC result registers commands */
uint8_t RDI[2]           = { 0x00, 0x04 };
uint8_t RDVB[2]          = { 0x00, 0x06 };
uint8_t RDIVB1[2]        = { 0x00, 0x08 };
uint8_t RDIACC[2]        = { 0x00, 0x44 };
uint8_t RDVBACC[2]       = { 0x00, 0x46 };
uint8_t RDIVB1ACC[2]      = { 0x00, 0x48 };

/* Read OCxADC result registers commands */
uint8_t RDOC[2]         = { 0x00, 0x0B };

/* Read VxADC result registers commands */
uint8_t RDV1A[2]         = { 0x00, 0x0A };
uint8_t RDV1B[2]         = { 0x00, 0x09 };
uint8_t RDV1C[2]         = { 0x00, 0x03 };
uint8_t RDV1D[2]         = { 0x00, 0x1B };
uint8_t RDV2A[2]         = { 0x00, 0x07 };//RDRVA
uint8_t RDV2B[2]         = { 0x00, 0x0D };
uint8_t RDV2C[2]         = { 0x00, 0x05 };
uint8_t RDV2D[2]         = { 0x00, 0x1F };
uint8_t RDV2E[2]         = { 0x00, 0x25 };

/* Read Status register */
uint8_t RDSTAT[2]       = { 0x00, 0x34 };

/* Read Flag register */
uint8_t RDFLAG[2]       = { 0x00, 0x32 };
uint8_t RDFLAGERR[2]    = { 0x00, 0x72 };   /* ERR */

/* Read AUX ADC result registers */
uint8_t RDXA[2]       = { 0x00, 0x30 };
uint8_t RDXB[2]       = { 0x00, 0x31 };
uint8_t RDXC[2]       = { 0x00, 0x33 };

/* Read all commands */
//------Read All IxADC and VBxADC results+Status+Flag-------
uint8_t RDALLI[2]        = { 0x00, 0x0C };

//------Read All IxACC and VBxACC results+Status+Flag-------
uint8_t RDALLA[2]        = { 0x00, 0x4C };

//------Read All configuration registers+Status+Flag-------
uint8_t RDALLC[2]        = { 0x00, 0x10 };

//------Read All Voltages-------
uint8_t RDALLV[2]        = { 0x00, 0x35 };

//------Read All Redundant Voltages-------
uint8_t RDALLR[2]        = { 0x00, 0x11 };

//------Read All Aux Voltages-------
uint8_t RDALLX[2]        = { 0x00, 0x51 };

/* Pwm registers commands */
uint8_t WRPWMA[2]         = { 0x00, 0x20 };
uint8_t RDPWMA[2]         = { 0x00, 0x22 };
uint8_t WRPWMB[2]         = { 0x00, 0x21 };
uint8_t RDPWMB[2]         = { 0x00, 0x23 };

/* Clear commands */
//uint8_t CLRAB[2]         = { 0x07, 0x11 };
uint8_t CLRI[2]         = { 0x07, 0x11 };
uint8_t CLRA[2]         = { 0x07, 0x14 };
uint8_t CLRO[2]         = { 0x07, 0x13 };
uint8_t CLRC[2]         = { 0x07, 0x16 };//Ask about CLRC//Sayani
uint8_t CLRVX [2]       = { 0x07, 0x12 };
//uint8_t CLRSTAT [2]      = { 0x07, 0x13 };
//uint8_t CLRFLAG[2]       = { 0x07, 0x17 };

/* Poll adc command */
//uint8_t PLADC[2]        = { 0x07, 0x18 };
uint8_t PLI1[2]         = { 0x07, 0x1C };
uint8_t PLI2[2]         = { 0x07, 0x1D };
uint8_t PLV[2]          = { 0x07, 0x1E };
uint8_t PLX[2]          = { 0x07, 0x1F };

/* GPIOs Comm commands */
//uint8_t WRCOMM[2]        = { 0x07, 0x21 };
//uint8_t RDCOMM[2]        = { 0x07, 0x22 };
/* command + dummy data for 72 clock cycles */
//uint8_t STCOMM[13]       = { 0x07, 0x23, 0xB9, 0xE4 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00 ,0x00};

/* Control Commands */
//uint8_t RDSID[2]         = { 0x00, 0x2C };
//uint8_t RSTCC[2]         = { 0x00, 0x2E };
//uint8_t SNAP[2]          = { 0x00, 0x2D };
//uint8_t UNSNAP[2]        = { 0x00, 0x2F };
//uint8_t SRST[2]          = { 0x00, 0x27 };
uint8_t sADI1[2]         = { 0x02, 0x60 };
uint8_t sADX[2]          = { 0x05, 0x30 };

//Command + pec
uint8_t RSTATD[4]        = {0x00, 0x33, 0x4D, 0x4A};  //Command +Pec
//uint8_t RSTATC[4]        = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Status C
uint8_t RFLAG[4]         = {0x00, 0x32, 0xc6, 0x78}; // Tiger CC is in Flag//Check PEC code//Sayani//Put breakpoint at Pec15_Calc for RDFLAG to check
uint8_t sRDI[4]          = {0x00, 0x04, 0x07, 0xC2};
uint8_t sCLRAB[4]        = {0x07,0x11,0xC9,0xC0};
//uint8_t sRSTATA [4]      = { 0x00, 0x30, 0x5B, 0x2E };
uint8_t sRDVA [4]        = { 0x00, 0x0A ,  0xC3 , 0x04};
uint8_t sRDVB [4]        = { 0x00, 0x09 , 0xD5 , 0x60};
uint8_t sRDVC [4]        = { 0x00, 0x03 , 0xA0, 0x38};
uint8_t sRDVD [4]        = { 0x00, 0x05 , 0x8C, 0xF0};
uint8_t sRDIAV[4]        = { 0x00, 0x44 , 0xE0, 0x48};
uint8_t sRDVBAT[4]       = { 0x00, 0x06 , 0x9A, 0x94};
/* Testmode and debugging commands */
uint8_t TM_48[2]       = { 0x00, 0x0E };        // LION: RDSVE



/// ----------------------- 6830 ONLY ------------------------------- ///

/* Read cell voltage result registers commands */
uint8_t RDCVA[2]         = { 0x00, 0x04 };
uint8_t RDCVB[2]         = { 0x00, 0x06 };
uint8_t RDCVC[2]         = { 0x00, 0x08 };
uint8_t RDCVD[2]         = { 0x00, 0x0A };
uint8_t RDCVE[2]         = { 0x00, 0x09 };
uint8_t RDCVF[2]         = { 0x00, 0x0B };
uint8_t RDCVALL[2]       = { 0x00, 0x0C };

/* Read average cell voltage result registers commands commands */
uint8_t RDACA[2]         = { 0x00, 0x44 };
uint8_t RDACB[2]         = { 0x00, 0x46 };
uint8_t RDACC[2]         = { 0x00, 0x48 };
uint8_t RDACD[2]         = { 0x00, 0x4A };
uint8_t RDACE[2]         = { 0x00, 0x49 };
uint8_t RDACF[2]         = { 0x00, 0x4B };
uint8_t RDACALL[2]       = { 0x00, 0x4C };

/* Read s voltage result registers commands */
uint8_t RDSVA[2]         = { 0x00, 0x03 };
uint8_t RDSVB[2]         = { 0x00, 0x05 };
uint8_t RDSVC[2]         = { 0x00, 0x07 };
uint8_t RDSVD[2]         = { 0x00, 0x0D };
uint8_t RDSVE[2]         = { 0x00, 0x0E };
uint8_t RDSVF[2]         = { 0x00, 0x0F };
uint8_t RDSALL[2]        = { 0x00, 0x10 };

/* Read c and s results */
uint8_t RDCSALL[2]       = { 0x00, 0x11 };
uint8_t RDACSALL[2]      = { 0x00, 0x51 };

/* Read filtered cell voltage result registers*/
uint8_t RDFCA[2]         = { 0x00, 0x12 };
uint8_t RDFCB[2]         = { 0x00, 0x13 };
uint8_t RDFCC[2]         = { 0x00, 0x14 };
uint8_t RDFCD[2]         = { 0x00, 0x15 };
uint8_t RDFCE[2]         = { 0x00, 0x16 };
uint8_t RDFCF[2]         = { 0x00, 0x17 };
uint8_t RDFCALL[2]       = { 0x00, 0x18 };

/* Read aux results */
uint8_t RDAUXA[2]        = { 0x00, 0x19 };
uint8_t RDAUXB[2]        = { 0x00, 0x1A };

/* Read redundant aux results */
uint8_t RDRAXA[2]        = { 0x00, 0x1C };
uint8_t RDRAXB[2]        = { 0x00, 0x1D };

/* Pwm registers commands */
uint8_t WRPWM1[2]        = { 0x00, 0x20 };
uint8_t RDPWM1[2]        = { 0x00, 0x22 };
uint8_t WRPWM2[2]        = { 0x00, 0x21 };
uint8_t RDPWM2[2]        = { 0x00, 0x23 };

/* Clear commands */
uint8_t CLRCELL[2]       = { 0x07, 0x11 };
uint8_t CLRAUX [2]       = { 0x07, 0x12 };
uint8_t CLRSPIN[2]       = { 0x07, 0x16 };
uint8_t CLRFC[2]         = { 0x07, 0x14 };
uint8_t CLOVUV[2]        = { 0x07, 0x15 };

/* Poll adc command */
uint8_t PLAUT[2]         = { 0x07, 0x19 };
uint8_t PLCADC[2]        = { 0x07, 0x1C };
uint8_t PLSADC[2]        = { 0x07, 0x1D };
uint8_t PLAUX1[2]        = { 0x07, 0x1E };
uint8_t PLAUX2[2]        = { 0x07, 0x1F };

/* Diagn command */
uint8_t DIAGN[2]         = {0x07 , 0x15};

/* Mute and Unmute commands */
uint8_t MUTE[2] 	     = { 0x00, 0x28 };
uint8_t UNMUTE[2]        = { 0x00, 0x29 };




//---------------------------------------------------------------//



ADCV_t const ADCV_default =
{
    .RD   = 0,
    .RES2 = 0b01,
    .X    = 0b00000,

    .OW   = 0,
    .RSTF = 0,
    .RES0 = 0b0,
    .DCP  = 0,
    .RES1 = 0b11,
    .CONT = 0,

};

ADCV_t ADCV = ADCV_default;




//.OW   = 0,
//.RSTF = 0,
//.RES0 = 0b0,
//.DCP  = 0,
//.RES1 = 0b11,
//.CONT = 0,
//.RD   = 0,
//.RES2 = 0b01,
//.X    = 0b00000



















