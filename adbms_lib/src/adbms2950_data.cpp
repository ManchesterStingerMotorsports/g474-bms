/**
********************************************************************************
*
* @file:    adi_bms_2950data.c
*
* @brief:   This file contains 2950 data function implementation.
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

/*! \addtogroup Data_Management
*  @{
*/

#include "common.h"
#include "adbms2950_data.h"
#include "adbms_mcuWrapper.h"
#include "adbms_utility.h"
#include "adbms_cmdlist.h"


namespace AD29_NS 
{
  /**
  *******************************************************************************
  * Function: adBms2950_Adi1
  * @brief ADI1 Command.
  *
  * @details Send ADI1 command to start Current1 ADC Conversion.
  *
  * Parameters:
  * @param [in]  RD      Enum type Read bit
  *
  * @param [in]  CONT    Enum type continuous measurement bit
  *
  * @param [in]  ow      Enum type open wire bit
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950_Adi1(RD rd, CONT cont, OW ow)
  {
    uint8_t cmd[2];
    cmd[0] = 0x02 + rd;
    cmd[1] = (cont<<7) + (ow & 0x03) + 0x60;
    spiSendCmd(cmd);
  }

  /**
  *******************************************************************************
  * Function: adBms2950_Adi2
  * @brief ADI2 Command.
  *
  * @details Send ADI2 command to start Current2 ADC Conversion.
  *
  * Parameters:
  * @param [in]  cont     Enum type continuous measurement bit
  *
  * @param [in]  ow       Enum type open wire
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950_Adi2(CONT cont, OW ow)
  {
    uint8_t cmd[2];
    cmd[0] = 0x01;
    cmd[1] = (cont<<7) + (ow &0x03) + 0x68;
    spiSendCmd(cmd);
  }

  /**
  *******************************************************************************
  * Function: adBms2950_Adv
  * @brief ADV Command.
  *
  * @details Send ADV command to start Voltage ADC Conversion.
  *
  * Parameters:
  * @param [in]  ow      Enum type open wire bits
  *
  * @param [in]  vch     Enum type open VCH channel
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950_Adv(OW ow, VCH vch)
  {
    uint8_t cmd[2];
    cmd[0] = 0x04;
    cmd[1] = (ow << 7) + 0x30 + (vch & 0x0F);
    spiSendCmd(cmd);
  }

  /**
  *******************************************************************************
  * Function: adBms2950_Adaux
  * @brief ADAUX Command.
  *
  * @details Send ADAUX command to start aux ADC Conversion.
  *
  * Parameters:
  * @param [in]  ach      Enum type ACH channel
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950_Adaux(ACH ach)
  {
    uint8_t cmd[2];
    cmd[0] = 0x05;
    cmd[1] = 0x30 + (ach & 0x03);
    spiSendCmd(cmd);
  }

  /**
  *******************************************************************************
  * Function: adBms2950_Stcomm
  * @brief Send command to Start I2C/SPI Communication.
  *
  * @details Send command to start I2C/SPI communication.
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950_Stcomm(void)
  {
    adBmsCsLow();
    spiWriteBytes(&STCOMM[0], 13);
    adBmsCsHigh();
  }

  /**
  *******************************************************************************
  * Function: adBms2950ConfigA_Flag
  * @brief Config A Flag Bits.
  *
  * @details This function Set configuration A flag bits.
  *
  * Parameters:
  *
  * @param [in]  flag_d      Enum type flag bit.
  *
  * @param [in]  flag       Enum type set or clr flag.
  *
  * @return Flag_value
  *
  *******************************************************************************
  */
  uint8_t adBms2950ConfigA_Flag(FLAG_D flag_d, CFGA_FLAG flag)
  {
    uint8_t flag_value;
    if(flag == FLAG_SET)
    {
      flag_value = (1 << flag_d);
    }
    else
    {
      flag_value = (0 << flag_d);
    }
    return(flag_value);
  }

  /**
  *******************************************************************************
  * Function: adBms2950ConfigA_Gpio
  * @brief Config Gpio Pull High/Low.
  *
  * @details This function Set configuration gpio as pull high/Low.
  *
  * Parameters:
  *
  * @param [in]  gpio        Enum type GPIO Pin.
  *
  * @param [in]  set_clr     Enum type gpio set (Low or High).
  *
  * @return Gpio_value
  *
  *******************************************************************************
  */
  uint8_t adBms2950ConfigA_Gpio(GPIO gpio, CFGA_GPIO set_clr)
  {
    uint8_t gpoivalue;
    if(set_clr == GPIO_SET)
    {
      gpoivalue = (1 << gpio);
    }
    else
    {
      gpoivalue = (0 << gpio);
    }
    return(gpoivalue);
  }

  /**
  *******************************************************************************
  * Function: adBms2950ConfigA_Gpo
  * @brief Set config GPO Pull High/Low.
  *
  * @details This function Set configuration gpo as pull high/Low.
  *
  * Parameters:
  *
  * @param [in]  gpo      Enum type GPO Pin.
  *
  * @param [in]  set_clr     Enum type gpo set (Low or High).
  *
  * @return Gpo_value
  *
  *******************************************************************************
  */
  uint8_t adBms2950ConfigA_Gpo(GPO gpo, CFGA_GPO set_clr)
  {
    uint8_t gpovalue;
    if(set_clr == GPO_SET)
    {
      gpovalue = (1 << gpo);
    }
    else
    {
      gpovalue = (0 << gpo);
    }
    return(gpovalue);
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseConfiga
  * @brief Parse the recived Configuration register A data
  *
  * @details This function Parse the recived Configuration register A data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                      cell_asic ic structure pointer
  *
  * @param [in]  *data                    Data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseConfiga(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].configa.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));

      ic[curr_ic].rx_cfga.vs10 = (ic[curr_ic].configa.rx_data[0] & 0x01);
      ic[curr_ic].rx_cfga.vs1 = (ic[curr_ic].configa.rx_data[0] & 0x06) >> 1;
      ic[curr_ic].rx_cfga.refon   = (ic[curr_ic].configa.rx_data[0] & 0x80) >> 7;

      ic[curr_ic].rx_cfga.flag_d  = (ic[curr_ic].configa.rx_data[1] & 0xFF);

      ic[curr_ic].rx_cfga.soak   = (ic[curr_ic].configa.rx_data[2] & 0x38) >> 3;
      ic[curr_ic].rx_cfga.vs6    = (((ic[curr_ic].configa.rx_data[2] & 0x40) >> 6));
      ic[curr_ic].rx_cfga.vs7    = (((ic[curr_ic].configa.rx_data[2] & 0x80) >> 7));

      ic[curr_ic].rx_cfga.gpio   = (ic[curr_ic].configa.rx_data[3] & 0x1E);
      ic[curr_ic].rx_cfga.gpo    = ((ic[curr_ic].configa.rx_data[3] & 0xE0) >> 4) | ((ic[curr_ic].configa.rx_data[4] & 0x03) << 4) | (ic[curr_ic].configa.rx_data[3] & 0x01) ;

      ic[curr_ic].rx_cfga.snap_st   = ((ic[curr_ic].configa.rx_data[5] & 0x20) >> 5);
      ic[curr_ic].rx_cfga.comm_bk   = ((ic[curr_ic].configa.rx_data[5] & 0x08) >> 3);
      ic[curr_ic].rx_cfga.vs5   = ((ic[curr_ic].configa.rx_data[5] & 0x04) >> 2);
      ic[curr_ic].rx_cfga.vs4   = ((ic[curr_ic].configa.rx_data[5] & 0x02) >> 1);
      ic[curr_ic].rx_cfga.vs3   = (ic[curr_ic].configa.rx_data[5] & 0x01);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseConfigb
  * @brief Parse the recived Configuration register B data
  *
  * @details This function Parse the recived Configuration register B data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   Data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseConfigb(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].configb.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));

      ic[curr_ic].rx_cfgb.oc1th = (ic[curr_ic].configb.rx_data[0] & 0x7F);
      ic[curr_ic].rx_cfgb.vs2 =  (((ic[curr_ic].configb.rx_data[0] & 0x80) >> 7) << 1); /* VS2[1]  MSB bit */

      ic[curr_ic].rx_cfgb.oc2th = (ic[curr_ic].configb.rx_data[1] & 0x7F);
      ic[curr_ic].rx_cfgb.vs2 =  (ic[curr_ic].configb.rx_data[1] & 0x80) >> 7; /* VS2[0] LSB bit */

      ic[curr_ic].rx_cfgb.dg1th = (ic[curr_ic].configb.rx_data[2] & 0x07);
      ic[curr_ic].rx_cfgb.dg2th = (ic[curr_ic].configb.rx_data[2] & 0x38) >> 3;
      ic[curr_ic].rx_cfgb.vs8 =  (ic[curr_ic].configb.rx_data[2] & 0xC0) >> 6;
      ic[curr_ic].rx_cfgb.vs9 =  (ic[curr_ic].configb.rx_data[2] & 0xC0) >> 7;
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseConfig
  * @brief Parse the recived Configuration register A & B data
  *
  * @details This function Parse the recived Configuration register A & B data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  grp                     Enum type register group
  *
  * @param [in]  *data                   Data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseConfig(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *data)
  {
    switch (grp)
    {
    case A:
      adBms2950ParseConfiga(tIC, &ic[0], &data[0]);
      break;

    case B:
      adBms2950ParseConfigb(tIC, &ic[0], &data[0]);
      break;

    default:
      break;
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseCurrentRegData
  * @brief Parse Current Register data
  *
  * @details This function Parse the received current register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *i_data                Current data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseCurrentRegData(uint8_t tIC, cell_asic *ic, uint8_t *i_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &i_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].i.i1 = ic[curr_ic].reg.rx_data[0] + (ic[curr_ic].reg.rx_data[1] << 8) + (ic[curr_ic].reg.rx_data[2] << 16);
      ic[curr_ic].i.i2 = ic[curr_ic].reg.rx_data[3] + (ic[curr_ic].reg.rx_data[4] << 8) + (ic[curr_ic].reg.rx_data[5] << 16);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseVbatRegData
  * @brief Parse Battery Voltage Register Data
  *
  * @details This function Parse the received battery voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *vbat_data             Vbat data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *vbat_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &vbat_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].vbat.vbat1 = ic[curr_ic].reg.rx_data[2] + (ic[curr_ic].reg.rx_data[3] << 8);
      ic[curr_ic].vbat.vbat2 = ic[curr_ic].reg.rx_data[4] + (ic[curr_ic].reg.rx_data[5] << 8);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseIVbatRegData
  * @brief Parse Current and Battery Voltage Register Data.
  *
  * @details This function Parse the received current and battery voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *ivbat_data            ivbat data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseIVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *ivbat_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &ivbat_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].ivbat.i1 = ic[curr_ic].reg.rx_data[0] + (ic[curr_ic].reg.rx_data[1] << 8) + (ic[curr_ic].reg.rx_data[2] << 16);
      ic[curr_ic].ivbat.vbat1 = ic[curr_ic].reg.rx_data[4] + (ic[curr_ic].reg.rx_data[5] << 8);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseOcRegData
  * @brief Parse Overcurrent Register Data.
  *
  * @details This function Parse the received overcurrent adc register.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *oc_data               Oc data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseOcRegData(uint8_t tIC, cell_asic *ic, uint8_t *oc_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &oc_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].oc.oc1r = (ic[curr_ic].reg.rx_data[0]);
      ic[curr_ic].oc.oc2r = (ic[curr_ic].reg.rx_data[1]);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseAvgCurrentRegData
  * @brief Parse Average Current Register Data.
  *
  * @details This function Parse the received average current register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *iavg_data             Average current data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseAvgCurrentRegData(uint8_t tIC, cell_asic *ic, uint8_t *iavg_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &iavg_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].iavg.i1avg = ic[curr_ic].reg.rx_data[0] + (ic[curr_ic].reg.rx_data[1] << 8) + ((ic[curr_ic].reg.rx_data[2]) << 16);
      ic[curr_ic].iavg.i2avg = ic[curr_ic].reg.rx_data[3] + (ic[curr_ic].reg.rx_data[4] << 8) + ((ic[curr_ic].reg.rx_data[5]) << 16);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseAvgVbatRegData
  * @brief Parse Average Batter Voltage Register Data.
  *
  * @details This function Parse the received average batter voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *avgvbat_data          avgvbat data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseAvgVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *avgvbat_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &avgvbat_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].vbavg.vb1avg = ic[curr_ic].reg.rx_data[0] + (ic[curr_ic].reg.rx_data[1] << 8) + (ic[curr_ic].reg.rx_data[2] << 16);
      ic[curr_ic].vbavg.vb2avg = ic[curr_ic].reg.rx_data[3] + (ic[curr_ic].reg.rx_data[4] << 8) + (ic[curr_ic].reg.rx_data[5] << 16);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseAvgIVbatRegData
  * @brief Parse Average Current and Battery Voltage Register Data.
  *
  * @details This function Parse the received average current and battery voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  *avgivbat_data         avgivbat data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseAvgIVbatRegData(uint8_t tIC, cell_asic *ic, uint8_t *avgivbat_data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].reg.rx_data[0], &avgivbat_data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].i_vbavg.i1avg = ic[curr_ic].reg.rx_data[0] + (ic[curr_ic].reg.rx_data[1] << 8) + (ic[curr_ic].reg.rx_data[2] << 16);
      ic[curr_ic].i_vbavg.vb1avg = ic[curr_ic].reg.rx_data[3] + (ic[curr_ic].reg.rx_data[4] << 8) + (ic[curr_ic].reg.rx_data[5] << 16);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950VrParseData
  * @brief Parse Voltage Register Data.
  *
  * @details This function Parse the received voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  grp                    Enum type register group
  *
  * @param [in]  *vr_data               Volatge reg. data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950VrParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *vr_data)
  {
    uint8_t *data, data_size, address = 0;
    if(grp == ALL_GRP){data_size = ALLVR_SIZE;}
    else {data_size = RX_DATA;}
    data = (uint8_t *)calloc(data_size, sizeof(uint8_t));
    if(data == NULL)
    {
      printf("Failed to allocate parse vr memory");
      exit(0);
    }
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&data[0], &vr_data[address], data_size); /* dst , src , size */
      address = ((curr_ic+1) * (data_size));
      switch (grp)
      {
      case A: /* VR Register group A */
        ic[curr_ic].vr.v_codes[0] = (data[0] + (data[1] << 8));
        ic[curr_ic].vr.v_codes[1] = (data[2] + (data[3] << 8));
        ic[curr_ic].vr.v_codes[2] = (data[4] + (data[5] << 8));
        break;

      case B: /* VR Register group B */
        ic[curr_ic].vr.v_codes[3] = (data[0] + (data[1] << 8));
        ic[curr_ic].vr.v_codes[4] = (data[2] + (data[3] << 8));
        ic[curr_ic].vr.v_codes[5] = (data[4] + (data[5] << 8));
        break;

      case C: /* VR Register group C */
        ic[curr_ic].vr.v_codes[6] = (data[0] + (data[1] << 8));
        ic[curr_ic].vr.v_codes[7] = (data[2] + (data[3] << 8));
        ic[curr_ic].vr.v_codes[8] = (data[4] + (data[5] << 8));
        break;

      case D: /* VR Register group D */
        ic[curr_ic].vr.v_codes[9] =  (data[0] + (data[1] << 8));
        ic[curr_ic].vr.v_codes[10] =  (data[2] + (data[3] << 8));
        ic[curr_ic].vr.v_codes[11] =  (data[4] + (data[5] << 8));
        break;

      case ALL_GRP: /* VR Register group ALL */
        ic[curr_ic].vr.v_codes[0]  = (data[0] + (data[1] << 8));
        ic[curr_ic].vr.v_codes[1]  = (data[2] + (data[3] << 8));
        ic[curr_ic].vr.v_codes[2]  = (data[4] + (data[5] << 8));
        ic[curr_ic].vr.v_codes[3]  = (data[6] + (data[7] << 8));
        ic[curr_ic].vr.v_codes[4]  = (data[8] + (data[9] << 8));
        ic[curr_ic].vr.v_codes[5]  = (data[10] + (data[11] << 8));
        ic[curr_ic].vr.v_codes[6]  = (data[12] + (data[13] << 8));
        ic[curr_ic].vr.v_codes[7]  = (data[14] + (data[15] << 8));
        ic[curr_ic].vr.v_codes[8]  = (data[16] + (data[17] << 8));
        ic[curr_ic].vr.v_codes[9]  = (data[18] + (data[19] << 8));
        ic[curr_ic].vr.v_codes[10] = (data[20] + (data[21] << 8));
        ic[curr_ic].vr.v_codes[11] = (data[22] + (data[23] << 8));
        break;

      default:
        break;
      }
    }
    free(data);
  }

  /**
  *******************************************************************************
  * Function: adBms2950VrxParseData
  * @brief Parse Voltage Register X Data.
  *
  * @details This function Parse the received voltage register x data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  grp                    Enum type register group
  *
  * @param [in]  *vrx_data               Volatge reg. data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950VrxParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *vrx_data)
  {
    uint8_t *data, data_size, address = 0;
    if(grp == ALL_GRP){}
    else {data_size = RX_DATA;}
    data = (uint8_t *)calloc(data_size, sizeof(uint8_t));
    if(data == NULL)
    {
      printf("Failed to allocate parse vrx memory");
      exit(0);
    }
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&data[0], &vrx_data[address], data_size); /* dst , src , size */
      address = ((curr_ic+1) * (data_size));
      switch (grp)
      {
      case X: /* VRX Register group X */
        ic[curr_ic].vrx.vx_codes[0] = (data[0] + (data[1] << 8));
        ic[curr_ic].vrx.vx_codes[1] = (data[2] + (data[3] << 8));
        ic[curr_ic].vrx.vx_codes[2] = (data[4] + (data[5] << 8));
        break;

      case Y: /* VRX Register group Y */
        ic[curr_ic].vrx.vx_codes[3] = (data[0] + (data[1] << 8));
        ic[curr_ic].vrx.vx_codes[4] = (data[2] + (data[3] << 8));
        ic[curr_ic].vrx.vx_codes[5] = (data[4] + (data[5] << 8));
        break;

      case Z: /* VRX Register group Z */
        ic[curr_ic].vrx.vx_codes[3] = (data[0] + (data[1] << 8));
        break;

      case ALL_GRP: /* VRX Register group ALL */
        ic[curr_ic].vrx.vx_codes[0]  = (data[0] + (data[1] << 8));
        ic[curr_ic].vrx.vx_codes[1]  = (data[2] + (data[3] << 8));
        ic[curr_ic].vrx.vx_codes[2]  = (data[4] + (data[5] << 8));
        ic[curr_ic].vrx.vx_codes[3]  = (data[6] + (data[7] << 8));
        ic[curr_ic].vrx.vx_codes[4]  = (data[8] + (data[9] << 8));
        ic[curr_ic].vrx.vx_codes[5]  = (data[10] + (data[11] << 8));
        break;

      default:
        break;
      }
    }
    free(data);
  }

  /**
  *******************************************************************************
  * Function: adBms2950RedVrParseData
  * @brief Parse Redundant Voltage Register Data.
  *
  * @details This function Parse the received Redundant voltage register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                    Total IC
  *
  * @param [in]  *ic                    cell_asic ic structure pointer
  *
  * @param [in]  grp                    Enum type register group
  *
  * @param [in]  *redvr_data            Redundant volatge reg. data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950RedVrParseData(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *redvr_data)
  {
    uint8_t *data, data_size, address = 0;
    if(grp == ALL_GRP){data_size = ALLREDVR_SIZE;}
    else {data_size = RX_DATA;}
    data = (uint8_t *)calloc(data_size, sizeof(uint8_t));
    if(data == NULL)
    {
      printf("Failed to allocate parse redvr memory");
      exit(0);
    }
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&data[0], &redvr_data[address], data_size); /* dst , src , size */
      address = ((curr_ic+1) * (data_size));
      switch (grp)
      {
      case A: /* RedVR Register group A */
        ic[curr_ic].rvr.redv_codes[0] = (data[0] + (data[1] << 8));
        ic[curr_ic].rvr.redv_codes[1] = (data[2] + (data[3] << 8));
        ic[curr_ic].rvr.redv_codes[2] = (data[4] + (data[5] << 8));
        break;

      case B: /* RedVR Register group B */
        ic[curr_ic].rvr.redv_codes[3] = (data[0] + (data[1] << 8));
        ic[curr_ic].rvr.redv_codes[4] = (data[2] + (data[3] << 8));
        ic[curr_ic].rvr.redv_codes[5] = (data[4] + (data[5] << 8));
        break;

      case ALL_GRP: /* RedVR Register group ALL */
        ic[curr_ic].rvr.redv_codes[0]  = (data[0] + (data[1] << 8));
        ic[curr_ic].rvr.redv_codes[1]  = (data[2] + (data[3] << 8));
        ic[curr_ic].rvr.redv_codes[2]  = (data[4] + (data[5] << 8));
        ic[curr_ic].rvr.redv_codes[3]  = (data[6] + (data[7] << 8));
        ic[curr_ic].rvr.redv_codes[4]  = (data[8] + (data[9] << 8));
        ic[curr_ic].rvr.redv_codes[5]  = (data[10] + (data[11] << 8));
        break;

      default:
        break;
      }
    }
    free(data);
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatusA
  * @brief Parse status A register data
  *
  * @details This function Parse the recived status A register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatusA(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].sta.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].stata.vref1p25   = (ic[curr_ic].sta.rx_data[0] | (ic[curr_ic].sta.rx_data[1] << 8));
      ic[curr_ic].stata.itmp       = (ic[curr_ic].sta.rx_data[2] | (ic[curr_ic].sta.rx_data[3] << 8));
      ic[curr_ic].stata.vreg2      = (ic[curr_ic].sta.rx_data[4] | (ic[curr_ic].sta.rx_data[5] << 8));
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatusB
  * @brief Parse status B register data
  *
  * @details This function Parse the recived status B register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatusB(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].stb.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].statb.oc1min   = ic[curr_ic].stb.rx_data[0];
      ic[curr_ic].statb.oc1max   = ic[curr_ic].stb.rx_data[1];
      ic[curr_ic].statb.oc2min   = ic[curr_ic].stb.rx_data[2];
      ic[curr_ic].statb.oc2max   = ic[curr_ic].stb.rx_data[3];
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatusC
  * @brief Parse status C register data
  *
  * @details This function Parse the recived status C register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatusC(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].stc.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      /* OCA bits */
      ic[curr_ic].statc.oc1a     = (ic[curr_ic].stc.rx_data[0] & 0x01);
      ic[curr_ic].statc.oc1a_inv = ((ic[curr_ic].stc.rx_data[0] & 0x02) >> 1);
      ic[curr_ic].statc.oc2a     = (ic[curr_ic].stc.rx_data[1] & 0x01);
      ic[curr_ic].statc.oc2a_inv = ((ic[curr_ic].stc.rx_data[1] & 0x02) >> 1);
      /* ct and cts */
      ic[curr_ic].statc.ct = (((ic[curr_ic].stc.rx_data[2] & 0x1F) << 6) | ((ic[curr_ic].stc.rx_data[3] & 0xFC) >> 2));
      ic[curr_ic].statc.cts = (ic[curr_ic].stc.rx_data[3] & 0x03);
      /* flag bits */
      ic[curr_ic].statc.otp2_med = (ic[curr_ic].stc.rx_data[4] & 0x01);
      ic[curr_ic].statc.otp2_ed = ((ic[curr_ic].stc.rx_data[4] & 0x02) >> 1);
      ic[curr_ic].statc.otp1_med = ((ic[curr_ic].stc.rx_data[4] & 0x04) >> 2);
      ic[curr_ic].statc.otp1_ed = ((ic[curr_ic].stc.rx_data[4] & 0x08) >> 3);
      ic[curr_ic].statc.vd_uv  = ((ic[curr_ic].stc.rx_data[4] & 0x10) >> 4);
      ic[curr_ic].statc.vd_ov = ((ic[curr_ic].stc.rx_data[4] & 0x20) >> 5);
      ic[curr_ic].statc.va_uv = ((ic[curr_ic].stc.rx_data[4] & 0x40) >> 6);
      ic[curr_ic].statc.va_ov = ((ic[curr_ic].stc.rx_data[4] & 0x80) >> 7);
      ic[curr_ic].statc.oscchk = (ic[curr_ic].stc.rx_data[5] & 0x01);
      ic[curr_ic].statc.tmodchk = ((ic[curr_ic].stc.rx_data[5] & 0x02) >> 1);
      ic[curr_ic].statc.thsd = ((ic[curr_ic].stc.rx_data[5] & 0x04) >> 2);
      ic[curr_ic].statc.sleep = ((ic[curr_ic].stc.rx_data[5] & 0x08) >> 3);
      ic[curr_ic].statc.spiflt  = ((ic[curr_ic].stc.rx_data[5] & 0x10) >> 4);
      ic[curr_ic].statc.insync = ((ic[curr_ic].stc.rx_data[5] & 0x20) >> 5);
      ic[curr_ic].statc.vdel = ((ic[curr_ic].stc.rx_data[5] & 0x40) >> 6);
      ic[curr_ic].statc.vde = ((ic[curr_ic].stc.rx_data[5] & 0x80) >> 7);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatusD
  * @brief Parse status D register data
  *
  * @details This function Parse the recived status D register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatusD(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].std.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      /* oc_cntr */
      ic[curr_ic].statd.oc_cntr = (ic[curr_ic].std.rx_data[5]);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatusE
  * @brief Parse status E register data
  *
  * @details This function Parse the recived status E register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatusE(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].ste.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].state.gpio   = ((ic[curr_ic].ste.rx_data[4] & 0x1E));
      ic[curr_ic].state.gpo   = ((ic[curr_ic].ste.rx_data[5] & 0x03) << 4 | ((ic[curr_ic].ste.rx_data[4] & 0xE0) >> 4) | (ic[curr_ic].ste.rx_data[4] & 0x01));
      ic[curr_ic].state.rev = ((ic[curr_ic].ste.rx_data[5] & 0xF0) >> 4);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseStatus
  * @brief Parse status register data
  *
  * @details This function Parse the recived status register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseStatus(uint8_t tIC, cell_asic *ic, GRP grp, uint8_t *data)
  {
    switch (grp)
    {
    case A: /* Status Register group A */
      adBms2950ParseStatusA(tIC, &ic[0], &data[0]);
      break;

    case B: /* Status Register group B */
      adBms2950ParseStatusB(tIC, &ic[0], &data[0]);
      break;

    case C: /* Status Register group C */
      adBms2950ParseStatusC(tIC, &ic[0], &data[0]);
      break;

    case D: /* Status Register group D */
      adBms2950ParseStatusD(tIC, &ic[0], &data[0]);
      break;

    case E: /* Status Register group E */
      adBms2950ParseStatusE(tIC, &ic[0], &data[0]);
      break;

    case ALL_GRP: /* Status Register group ALL */
      // TBD
      break;

    default:
      break;
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseComm
  * @brief Parse comm register
  *
  * @details This function Parse the recived comm register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseComm(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].com.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].rx_comm.icomm[0] = ((ic[curr_ic].com.rx_data[0] & 0xF0) >> 4);
      ic[curr_ic].rx_comm.fcomm[0] = (ic[curr_ic].com.rx_data[0] & 0x0F);
      ic[curr_ic].rx_comm.data[0] = (ic[curr_ic].com.rx_data[1]);
      ic[curr_ic].rx_comm.icomm[1] = ((ic[curr_ic].com.rx_data[2] & 0xF0) >> 4);
      ic[curr_ic].rx_comm.data[1] = (ic[curr_ic].com.rx_data[3]);
      ic[curr_ic].rx_comm.fcomm[1] = (ic[curr_ic].com.rx_data[2] & 0x0F);
      ic[curr_ic].rx_comm.icomm[2] = ((ic[curr_ic].com.rx_data[4] & 0xF0) >> 4);
      ic[curr_ic].rx_comm.data[2] = (ic[curr_ic].com.rx_data[5]);
      ic[curr_ic].rx_comm.fcomm[2] = (ic[curr_ic].com.rx_data[4] & 0x0F);
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950ParseSID
  * @brief Parse SID register
  *
  * @details This function Parse the recived sid register data.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @param [in]  *data                   data pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950ParseSID(uint8_t tIC, cell_asic *ic, uint8_t *data)
  {
    uint8_t address = 0;
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      memcpy(&ic[curr_ic].rsid.rx_data[0], &data[address], RX_DATA); /* dst , src , size */
      address = ((curr_ic+1) * (RX_DATA));
      ic[curr_ic].sid.sid[0] = ic[curr_ic].rsid.rx_data[0];
      ic[curr_ic].sid.sid[1] = ic[curr_ic].rsid.rx_data[1];
      ic[curr_ic].sid.sid[2] = ic[curr_ic].rsid.rx_data[2];
      ic[curr_ic].sid.sid[3] = ic[curr_ic].rsid.rx_data[3];
      ic[curr_ic].sid.sid[4] = ic[curr_ic].rsid.rx_data[4];
      ic[curr_ic].sid.sid[5] = ic[curr_ic].rsid.rx_data[5];
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950CreateConfiga
  * @brief Create the configation A write buffer
  *
  * @details This function create the configation A write buffer.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950CreateConfiga(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      ic[curr_ic].configa.tx_data[0] = (((ic[curr_ic].tx_cfga.refon & 0x01) << 7) | ((ic[curr_ic].tx_cfga.vs1 & 0x03) << 1) |(ic[curr_ic].tx_cfga.vs10));
      ic[curr_ic].configa.tx_data[1] = (ic[curr_ic].tx_cfga.flag_d & 0xFF);
      ic[curr_ic].configa.tx_data[2] = (((ic[curr_ic].tx_cfga.vs7 & 0x01) << 7) | ((ic[curr_ic].tx_cfga.vs6 & 0x01) << 6) | ((ic[curr_ic].tx_cfga.soak & 0x07) << 3));
      ic[curr_ic].configa.tx_data[3] = (((ic[curr_ic].tx_cfga.gpo & 0x0E) << 4) | ((ic[curr_ic].tx_cfga.gpio & 0x0F) << 1) | (ic[curr_ic].tx_cfga.gpo & 0x01));   /* GPO1 is at position 0 */
      ic[curr_ic].configa.tx_data[4] = ((ic[curr_ic].tx_cfga.gpo & 0x30) >> 4);
      ic[curr_ic].configa.tx_data[5] = (((ic[curr_ic].tx_cfga.snap_st & 0x01) << 5) | ((ic[curr_ic].tx_cfga.comm_bk & 0x01) << 3) | ((ic[curr_ic].tx_cfga.vs5 & 0x01) << 2) | ((ic[curr_ic].tx_cfga.vs4 & 0x01) << 1) | (ic[curr_ic].tx_cfga.vs3 & 0x01));
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950CreateConfigb
  * @brief Create the configation B write buffer
  *
  * @details This function create the configation B write buffer.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950CreateConfigb(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      ic[curr_ic].configb.tx_data[0] = (((ic[curr_ic].tx_cfgb.vs2 & 0x02) << 7) | (ic[curr_ic].tx_cfgb.oc1th & 0x7F));
      ic[curr_ic].configb.tx_data[1] = (((ic[curr_ic].tx_cfgb.vs2 & 0x01) << 7) | (ic[curr_ic].tx_cfgb.oc2th & 0x7F));
      ic[curr_ic].configb.tx_data[2] = (((ic[curr_ic].tx_cfgb.vs9 & 0x01) << 7) | ((ic[curr_ic].tx_cfgb.vs8 & 0x01) << 6)|((ic[curr_ic].tx_cfgb.dg2th & 0x07) << 3) | (ic[curr_ic].tx_cfgb.dg1th & 0x07));
      ic[curr_ic].configb.tx_data[3] = 0x00;
      ic[curr_ic].configb.tx_data[4] = 0x00;
      ic[curr_ic].configb.tx_data[5] = 0x00;
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950CreateClrflagData
  * @brief Create the clear flag write buffer
  *
  * @details This function create the clear flag write buffer.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950CreateClrflagData(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      ic[curr_ic].clrflag.tx_data[0] = (((ic[curr_ic].clflag.cl_oc1m & 0x01) << 1) | (ic[curr_ic].clflag.cl_oc1a & 0x01));
      ic[curr_ic].clrflag.tx_data[1] = (((ic[curr_ic].clflag.cl_oc2m & 0x01) << 1) | (ic[curr_ic].clflag.cl_oc2a & 0x01));
      ic[curr_ic].clrflag.tx_data[2] = 0x00;
      ic[curr_ic].clrflag.tx_data[3] = 0x00;
      ic[curr_ic].clrflag.tx_data[4] = ((ic[curr_ic].clflag.cl_vaov << 7) | (ic[curr_ic].clflag.cl_vauv << 6) | (ic[curr_ic].clflag.cl_vdov << 5) | (ic[curr_ic].clflag.cl_vduv << 4)
                                        |(ic[curr_ic].clflag.cl_opt1_ed << 3)| (ic[curr_ic].clflag.cl_opt1_med << 2) | (ic[curr_ic].clflag.cl_opt2_ed << 1) | (ic[curr_ic].clflag.cl_opt2_med));
      ic[curr_ic].clrflag.tx_data[5] = ((ic[curr_ic].clflag.cl_vde << 7) | (ic[curr_ic].clflag.cl_vdel << 6) | (ic[curr_ic].clflag.cl_spiflt << 4) |(ic[curr_ic].clflag.cl_sleep << 3)
                                        | (ic[curr_ic].clflag.cl_thsd << 2) | (ic[curr_ic].clflag.cl_tmode << 1) | (ic[curr_ic].clflag.cl_oscchk));
    }
  }

  /**
  *******************************************************************************
  * Function: adBms2950CreateComm
  * @brief Create the configation comm write buffer
  *
  * @details This function create the configation comm write buffer.
  *
  * Parameters:
  *
  * @param [in]  tIC                     Total IC
  *
  * @param [in]  *ic                     cell_asic ic structure pointer
  *
  * @return None
  *
  *******************************************************************************
  */
  void adBms2950CreateComm(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t curr_ic = 0; curr_ic < tIC; curr_ic++)
    {
      ic[curr_ic].com.tx_data[0] = ((ic[curr_ic].tx_comm.icomm[0] & 0x0F)  << 4  | (ic[curr_ic].tx_comm.fcomm[0]   & 0x0F));
      ic[curr_ic].com.tx_data[1] = ((ic[curr_ic].tx_comm.data[0] ));
      ic[curr_ic].com.tx_data[2] = ((ic[curr_ic].tx_comm.icomm[1] & 0x0F)  << 4 ) | (ic[curr_ic].tx_comm.fcomm[1]   & 0x0F);
      ic[curr_ic].com.tx_data[3] = ((ic[curr_ic].tx_comm.data[1]));
      ic[curr_ic].com.tx_data[4] = ((ic[curr_ic].tx_comm.icomm[2] & 0x0F)  << 4  | (ic[curr_ic].tx_comm.fcomm[2]   & 0x0F));
      ic[curr_ic].com.tx_data[5] = ((ic[curr_ic].tx_comm.data[2]));
    }
  }
}

/** @}*/
/** @}*/
