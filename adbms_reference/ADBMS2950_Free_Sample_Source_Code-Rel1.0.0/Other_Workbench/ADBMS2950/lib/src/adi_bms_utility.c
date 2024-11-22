/**
********************************************************************************
*
* @file:    adi_bms_utility.c
*
* @brief:   This file contains utility functions implementation.
*
* @details:
*
*******************************************************************************
Copyright(c) 2020 Analog Devices, Inc. All Rights Reserved. This software is
proprietary & confidential to Analog Devices, Inc. and its licensors. By using
this software you agree to the terms of the associated Analog Devices License
Agreement.
*******************************************************************************
*/
/*! \addtogroup BMS_Driver
*  @{
*/

/*! \addtogroup Driver_Utility
*  @{
*/

/*!<
This documentation provides details about BMS driver APIs and their usage.
Using the BMS Driver Application can:
- Read/Write the configuration registers of the BMS devices stacked in daisy chaining.
- Send commands and Read the Cell Voltages, Aux Voltages and Status registers (Sum of cells, Internal Die temperature etc.)
*/

/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "adbms2950.h"
#include "pal.h"
/*!<**************************************** BMS Driver APIs definitions ********************************************/

/*!<
* @brief Precomputed CRC15 Table
*/
const uint16_t Crc15Table[256] =
{
  0x0000,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,
  0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
  0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
  0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
  0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
  0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
  0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
  0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
  0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
  0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
  0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
  0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
  0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
  0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
  0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
  0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
  0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
  0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
  0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
  0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
  0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
  0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
  0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
};

/**
*******************************************************************************
* Function: Pec15_Calc
* @brief CRC15 Pec Calculation Function
*
* @details This function calculates and return the CRC15 value
*
* Parameters:
* @param [in]	Len	Data length
*
* @param [in] *data    Data pointer
*
* @return CRC15_Value
*
*******************************************************************************
*/
uint16_t Pec15_Calc
(
uint8_t len, /*!< Number of bytes that will be used to calculate a PEC */
uint8_t *data /*!< Array of data that will be used to calculate  a PEC */
)
{
  uint16_t remainder,addr;
  remainder = 16; /*!< initialize the PEC */
  for (uint8_t i = 0; i<len; i++) /*!< loops for each byte in data array */
  {
    addr = (((remainder>>7)^data[i])&0xff);/*!< calculate PEC table address */
    remainder = ((remainder<<8)^Crc15Table[addr]);
  }
  return(remainder*2);/*!< The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2 */
}

/**
*******************************************************************************
* Function: Pec10_Calc
* @brief CRC10 Pec Calculation Function
*
* @details This function calculates and return the CRC15 value
*
* Parameters:
* @param [in]	Len	Data length
*
* @param [in] *data    Data pointer
*
* @return CRC15_Value
*
*******************************************************************************
*/
uint16_t pec10_calc(bool rx_cmd, int len, uint8_t *data)
{
  uint16_t remainder = 16; /*!< PEC_SEED;   0000010000 */
  uint16_t polynom = 0x8F; /*!< x10 + x7 + x3 + x2 + x + 1 <- the CRC15 polynomial         100 1000 1111   48F */

  /*!< Perform modulo-2 division, a byte at a time. */
  for (uint8_t pbyte = 0; pbyte < len; ++pbyte)
  {
    /*!< Bring the next byte into the remainder. */
    remainder ^= (uint16_t)(data[pbyte] << 2);
    /*!< Perform modulo-2 division, a bit at a time.*/
    for (uint8_t bit_ = 8; bit_ > 0; --bit_)
    {
      /*!< Try to divide the current data bit. */
      if ((remainder & 0x200) > 0)/*!<equivalent to remainder & 2^14 simply check for MSB */
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)(remainder << 1);
      }
    }
  }
  if (rx_cmd == true)
  {
    remainder ^= (uint16_t)((data[len] & 0xFC) << 2);
    /*!< Perform modulo-2 division, a bit at a time */
    for (uint8_t bit_ = 6; bit_ > 0; --bit_)
    {
      /*!< Try to divide the current data bit */
      if ((remainder & 0x200) > 0)/*!<equivalent to remainder & 2^14 simply check for MSB*/
      {
        remainder = (uint16_t)((remainder << 1));
        remainder = (uint16_t)(remainder ^ polynom);
      }
      else
      {
        remainder = (uint16_t)((remainder << 1));
      }
    }
  }
  return ((uint16_t)(remainder & 0x3FF));
}

/**
*******************************************************************************
* Function: spiSendCmd
* @brief Send command over SPI
*
* @details This function send bms command in spi line
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic structure pointer
*
* @param [in]  tx_cmd   Tx command byte array
*
* @return None
*
*******************************************************************************
*/
void spiSendCmd(uint8_t tx_cmd[2])
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  cmd[0] = tx_cmd[0];
  cmd[1] =  tx_cmd[1];
  cmd_pec = Pec15_Calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  adBmsCsLow();
  spiWriteBytes(&cmd[0], 4);
  adBmsCsHigh();
}

/**
*******************************************************************************
* Function: spiReadData
* @brief Spi Read Data, Lower Level function for adBmsReadData
*
* @details This function send command to DUT and read command corrospond data byte.
*
* Parameters:
* @param [in]	tIC     Total IC
*
* @param [in]  tx_cmd   Tx command bytes
*
* @param [in]  *rx_data Rx data pointer
*
* @param [in]  *pec_error Pec error pointer
*
* @param [in]  *cmd_cntr command counter pointer
*
* @return None
*
*******************************************************************************
*/
void spiReadData
(
uint8_t tIC,
uint8_t tx_cmd[2],
uint8_t *rx_data,
uint8_t *pec_error,
uint8_t *cmd_cntr,
uint8_t regData_size
)
{
  uint8_t *data, *copyArray, src_address = 0;
  uint16_t cmd_pec, received_pec, calculated_pec;
  uint8_t BYTES_IN_REG = regData_size;
  uint8_t RX_BUFFER = (regData_size * tIC);

  data = (uint8_t *)calloc(RX_BUFFER, sizeof(uint8_t));
  copyArray = (uint8_t *)calloc(BYTES_IN_REG, sizeof(uint8_t));
  if((data == NULL) || (copyArray == NULL))
  {
    printMsg("Failed to allocate spi read data memory");
    exit(0);
  }
  else
  {
    uint8_t cmd[4];
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    cmd_pec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    adBmsWakeupIc(tIC);
    adBmsCsLow();
    spiWriteReadBytes(&cmd[0], &data[0], RX_BUFFER);                 /*!< Read the configuration data of all ICs on the daisy chain into readdata array */
    adBmsCsHigh();
    for (uint8_t current_ic = 0; current_ic < tIC; current_ic++)     /*!< executes for each ic in the daisy chain and packs the data */
    {																																      /*!< Into the r_comm array as well as check the received data for any bit errors */
      for (uint8_t current_byte = 0; current_byte < (BYTES_IN_REG-2); current_byte++)
      {
        rx_data[(current_ic*BYTES_IN_REG)+current_byte] = data[current_byte + (current_ic*BYTES_IN_REG)];
      }
      /*!< Get command counter value */
      cmd_cntr[current_ic] = (data[(current_ic * BYTES_IN_REG) + (BYTES_IN_REG - 2)] >> 2);
      /*!< Get received pec value from ic*/
      received_pec = (uint16_t)(((data[(current_ic * BYTES_IN_REG) + (BYTES_IN_REG - 2)] & 0x03) << 8) | data[(current_ic * BYTES_IN_REG) + (BYTES_IN_REG - 1)]);
      /*!< Copy each ic correspond data + pec value for calculate data pec */
      memcpy(&copyArray[0], &data[src_address], BYTES_IN_REG);
      src_address = ((current_ic+1) * (regData_size));
      /*!< Calculate data pec */
      calculated_pec = (uint16_t)pec10_calc(true, (BYTES_IN_REG-2), &copyArray[0]);
      /*!< Match received pec with calculated pec */
      if (received_pec == calculated_pec){ pec_error[current_ic] = 0; }/*!< If no error is there value set to 0 */
      else{ pec_error[current_ic] = 1; }                               /*!< If error is there value set to 1 */
    }
  }
  free(data);
  free(copyArray);
}

/**
*******************************************************************************
* Function: spiWriteData
* @brief write data over SPI, Lower Level function for adBmsWriteData
*
* @details This function write the data into bms ic
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  tx_cmd   Tx command bytes
*
* @param [in]  *data   Data pointer
*
* @return None
*
*******************************************************************************
*/
void spiWriteData
(
uint8_t tIC,
uint8_t tx_cmd[2],
uint8_t *data
)
{
  uint8_t BYTES_IN_REG = TX_DATA;
  uint8_t CMD_LEN = 4 + (RX_DATA * tIC);
  uint16_t data_pec, cmd_pec;
  uint8_t *cmd, copyArray[TX_DATA], src_address = 0;
  uint8_t cmd_index;
  cmd = (uint8_t *)calloc(CMD_LEN, sizeof(uint8_t));
  if(cmd == NULL)
  {
    printMsg("Failed to allocate cmd array memory");
    exit(0);
  }
  else
  {
    cmd[0] = tx_cmd[0];
    cmd[1] = tx_cmd[1];
    cmd_pec = Pec15_Calc(2, cmd);
    cmd[2] = (uint8_t)(cmd_pec >> 8);
    cmd[3] = (uint8_t)(cmd_pec);
    cmd_index = 4;
    /*!< executes for each LTC68xx, this loops starts with the last IC on the stack */
    for (uint8_t current_ic = tIC; current_ic > 0; current_ic--)
    {
      src_address = ((current_ic-1) * TX_DATA);
      /*!< The first configuration written is received by the last IC in the daisy chain */
      for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
      {
        cmd[cmd_index] = data[((current_ic-1)*6)+current_byte];
        cmd_index = cmd_index + 1;
      }
      /*!< Copy each ic correspond data + pec value for calculate data pec */
      memcpy(&copyArray[0], &data[src_address], TX_DATA); /*!< dst, src, size */
      /*!< calculating the PEC for each Ics configuration register data */
      data_pec = (uint16_t)pec10_calc(true,BYTES_IN_REG, &copyArray[0]);
      cmd[cmd_index] = (uint8_t)(data_pec >> 8);
      cmd_index = cmd_index + 1;
      cmd[cmd_index] = (uint8_t)data_pec;
      cmd_index = cmd_index + 1;
    }
    adBmsCsLow();
    spiWriteBytes(&cmd[0], CMD_LEN);
    adBmsCsHigh();
  }
  free(cmd);
}

/**
*******************************************************************************
* Function: adBmsReadData
* @brief Adbms Read Data From Bms ic.
*
* @details This function sends the command read payload, parse data put it in structure and check pec error.(Ex: RDCFGA)
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @param [in]  cmd_arg   command bytes
*
* @param [in]  TYPE   Enum type of resistor
*
* @param [in]  GRP   Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void adBmsReadData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group)
{
  uint16_t rBuff_size;
  uint8_t regData_size;
  if(group == ALL_GRP)
  {
    if(type == Rdalla){rBuff_size = RDALLA_SIZE; regData_size = RDALLA_SIZE;}
    else if(type == Rdallb){rBuff_size = RDALLB_SIZE; regData_size = RDALLB_SIZE;}
    else if(type == Rdcall){rBuff_size = RDCALL_SIZE; regData_size = RDCALL_SIZE;}
    else if(type == Rdasall){rBuff_size = RDASALL_SIZE; regData_size = RDASALL_SIZE;}
    else
    {
      printMsg("Read All cmd wrong type select");
    }
  }
  else{rBuff_size = (tIC * RX_DATA); regData_size = RX_DATA;}
  uint8_t *read_buffer, *pec_error, *cmd_count;
  read_buffer = (uint8_t *)calloc(rBuff_size, sizeof(uint8_t));
  pec_error = (uint8_t *)calloc(tIC, sizeof(uint8_t));
  cmd_count = (uint8_t *)calloc(tIC, sizeof(uint8_t));
  if((pec_error == NULL) || (cmd_count == NULL) || (read_buffer == NULL))
  {
    printMsg("Failed to allocate memory");
    exit(0);
  }
  else
  {
    spiReadData(tIC, &cmd_arg[0], &read_buffer[0], &pec_error[0], &cmd_count[0], regData_size);
    switch (type)
    {
    case Config:
      adBms2950ParseConfig(tIC, ic, group, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.cfgr_pec = pec_error[cic];
      }
      break;

    case Vr:
      adBms2950VrParseData(tIC, ic, group, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.vr_pec = pec_error[cic];
      }
      break;

    case Rvr:
      adBms2950RedVrParseData(tIC, ic, group, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.rvr_pec = pec_error[cic];
      }
      break;

    case Cr:
      adBms2950ParseCurrentRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.cr_pec = pec_error[cic];
      }
      break;

    case Vbat:
      adBms2950ParseVbatRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.vbat_pec = pec_error[cic];
      }
      break;

    case Ivbat:
      adBms2950ParseIVbatRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.ivbat_pec = pec_error[cic];
      }
      break;

    case Oc:
      adBms2950ParseOcRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.oc_pec = pec_error[cic];
      }
      break;

    case AvgCr:
      adBms2950ParseAvgCurrentRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.avgcr_pec = pec_error[cic];
      }
      break;

    case AvgVbat:
      adBms2950ParseAvgVbatRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.avgvbat_pec = pec_error[cic];
      }
      break;

    case AvgIvbat:
      adBms2950ParseAvgIVbatRegData(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.avgivbat_pec = pec_error[cic];
      }
      break;

    case Status:
      adBms2950ParseStatus(tIC, ic, group, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.stat_pec = pec_error[cic];
      }
      break;

    case Comm:
      adBms2950ParseComm(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.comm_pec = pec_error[cic];
      }
      break;

    case SID:
      adBms2950ParseSID(tIC, ic, &read_buffer[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].cccrc.sid_pec = pec_error[cic];
      }
      break;

    case Rdalla:
      // TBD
      break;

    case Rdallb:
      // TBD
      break;

    case Rdcall:
      // TBD
      break;

    case Rdasall:
      // TBD
      break;

    default:
      break;
    }
  }
  free(read_buffer);
  free(pec_error);
  free(cmd_count);
}
/**
*******************************************************************************
* Function: adBmsWriteData
* @brief Adbms Write Data into Bms ic.
*
* @details This function write the data to DUT. (EX: WRCFGA)
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @param [in]  cmd_arg   command bytes
*
* @param [in]  TYPE   Enum type of resistor
*
* @param [in]  GRP   Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void adBmsWriteData(uint8_t tIC, cell_asic *ic, uint8_t cmd_arg[2], TYPE type, GRP group)
{
  uint8_t data_len = TX_DATA, write_size = (TX_DATA * tIC);
  uint8_t *write_buffer = (uint8_t *)calloc(write_size, sizeof(uint8_t));
  if(write_buffer == NULL)
  {
    printMsg("Failed to allocate write_buffer array memory");
    exit(0);
  }
  else
  {
    switch (type)
    {
    case Config:
      switch (group)
      {
      case A:
        adBms2950CreateConfiga(tIC, &ic[0]);
        for (uint8_t cic = 0; cic < tIC; cic++)
        {
          for (uint8_t data = 0; data < data_len; data++)
          {
            write_buffer[(cic * data_len) + data] = ic[cic].configa.tx_data[data];
          }
        }
        break;
      case B:
        adBms2950CreateConfigb(tIC, &ic[0]);
        for (uint8_t cic = 0; cic < tIC; cic++)
        {
          for (uint8_t data = 0; data < data_len; data++)
          {
            write_buffer[(cic * data_len) + data] = ic[cic].configb.tx_data[data];
          }
        }
        break;
      }
      break;

    case Comm:
      adBms2950CreateComm(tIC, &ic[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        for (uint8_t data = 0; data < data_len; data++)
        {
          write_buffer[(cic * data_len) + data] = ic[cic].com.tx_data[data];
        }
      }
      break;

    case Clrflag:
      adBms2950CreateClrflagData(tIC, &ic[0]);
      for (uint8_t cic = 0; cic < tIC; cic++)
      {
        for (uint8_t data = 0; data < data_len; data++)
        {
          write_buffer[(cic * data_len) + data] = ic[cic].clrflag.tx_data[data];
        }
      }
      break;

    default:
      break;
    }
  }
  adBmsWakeupIc(tIC);
  spiWriteData(tIC, cmd_arg, &write_buffer[0]);
  free(write_buffer);
}

/**
*******************************************************************************
* Function: adBmsPollAdc
* @brief PLADC Command.
*
* @details Send poll adc command and retun adc conversion count.
*
* Parameters:
*
* @param [in]  tIC      Total IC
*
* @param [in]  *ic      cell_asic structure pointer
*
* @param [in]  tx_cmd   Tx command byte
*
* @return Pladc_count(uint32_t)
*
*******************************************************************************
*/
uint32_t adBmsPollAdc(uint8_t tx_cmd[2])
{
  uint32_t pladc_count = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t read_data = 0x00;
  uint8_t SDO_Line = 0xFF;
  cmd[0] = tx_cmd[0];
  cmd[1] = tx_cmd[1];
  cmd_pec = Pec15_Calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);
  startTimer();
  adBmsCsLow();
  spiWriteBytes(&cmd[0], 4);
  do{
    spiReadBytes(&read_data, 1);
  }while(!(read_data == SDO_Line));
  adBmsCsHigh();
  pladc_count = getTimCount();
  stopTimer();
  return(pladc_count);
}

/** @}*/
/** @}*/