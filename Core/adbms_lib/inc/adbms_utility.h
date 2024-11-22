

#pragma once

#include "common.h"
#include "adbms6830_data.h"
#include "adbms2950_data.h"


/* Calculates and returns the CRC15Table */
uint16_t Pec15_Calc
( 
  uint8_t len, /* Number of bytes that will be used to calculate a PEC */
  uint8_t *data /* Array of data that will be used to calculate  a PEC */								 
);									 
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data);
void spiSendCmd(uint8_t tx_cmd[2]);
void spiReadData
( 
uint8_t tIC, 
uint8_t tx_cmd[2], 
uint8_t *rx_data,
uint8_t *pec_error,
uint8_t *cmd_cntr,
uint8_t regData_size
);
void spiWriteData
(
  uint8_t tIC, 
  uint8_t tx_cmd[2], 
  uint8_t *data
);

uint32_t adBmsPollAdc(uint8_t tx_cmd[2]);

namespace AD68_NS
{
  void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
  void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);

  void adBms6830_Adcv
  (
    RD rd,
    CONT cont,
    DCP dcp,
    RSTF rstf,
    OW_C_S owcs
  );

  void adBms6830_Adsv
  (
    CONT cont,
    DCP dcp,
    OW_C_S owcs
  );

  void adBms6830_Adax
  (
  OW_AUX owaux, 							
  PUP pup,
  CH ch
  );

  void adBms6830_Adax2
  (
    CH ch
  );
}

namespace AD29_NS
{
  void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
  void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group);
}
