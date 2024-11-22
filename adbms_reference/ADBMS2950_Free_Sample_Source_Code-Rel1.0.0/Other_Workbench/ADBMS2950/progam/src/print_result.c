/**
********************************************************************************
*
* @file:    print_result.c
*
* @brief:   This file contains the print result function implementation.
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
/*! \addtogroup Main
*  @{
*/

/*! \addtogroup Print_Result
*  @{
*/

#include "application.h"
#ifdef MBED
extern Serial pc;
/**
*******************************************************************************
* Function: printWriteConfig
* @brief Print write config A/B result.
*
* @details This function Print write config result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @param [in]  grp      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == Config)
    {
      if(grp == A)
      {
        pc.printf("Write Config A:\n");
        pc.printf("0x%X, ", IC[ic].configa.tx_data[0]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[1]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[2]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[3]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[4]);
        pc.printf("0x%X\n\n", IC[ic].configa.tx_data[5]);
      }
      else if(grp == B)
      {
        pc.printf("Write Config B:\n");
        pc.printf("0x%X, ", IC[ic].configb.tx_data[0]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[1]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[2]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[3]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[4]);
        pc.printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
      }
      else if(grp == ALL_GRP)
      {
        pc.printf("Write Config A:\n");
        pc.printf("0x%X, ", IC[ic].configa.tx_data[0]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[1]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[2]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[3]);
        pc.printf("0x%X, ", IC[ic].configa.tx_data[4]);
        pc.printf("0x%X\n\n", IC[ic].configa.tx_data[5]);

        pc.printf("Write Config B:\n");
        pc.printf("0x%X, ", IC[ic].configb.tx_data[0]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[1]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[2]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[3]);
        pc.printf("0x%X, ", IC[ic].configb.tx_data[4]);
        pc.printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
      }
      else{ pc.printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: printReadConfig
* @brief Print Read config A/B result.
*
* @details This function Print read config result into IAR I/O terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @param [in]  TYPE     Enum type of resistor
*
* @param [in]  GRP      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == Config)
    {
      if(grp == A)
      {
        pc.printf("Read Config A:\n");
        pc.printf("0x%X, ", IC[ic].configa.rx_data[0]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[1]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[2]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[3]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[4]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[5]);
        pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else if(grp == B)
      {
        pc.printf("Read Config B:\n");
        pc.printf("0x%X, ", IC[ic].configb.rx_data[0]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[1]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[2]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[3]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[4]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[5]);
        pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else if(grp == ALL_GRP)
      {
        pc.printf("Read Config A:\n");
        pc.printf("0x%X, ", IC[ic].configa.rx_data[0]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[1]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[2]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[3]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[4]);
        pc.printf("0x%X, ", IC[ic].configa.rx_data[5]);
        pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n",IC[ic].cccrc.cfgr_pec);

        pc.printf("Read Config B:\n");
        pc.printf("0x%X, ", IC[ic].configb.rx_data[0]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[1]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[2]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[3]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[4]);
        pc.printf("0x%X, ", IC[ic].configb.rx_data[5]);
        pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else{ pc.printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: PrintDeviceSID
* @brief Print Device SID.
*
* @details This function Print Device SID into IAR I/O terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == SID)
    {
      pc.printf("Read Device SID:\n");
      pc.printf("0x%X, ", IC[ic].sid.sid[0]);
      pc.printf("0x%X, ", IC[ic].sid.sid[1]);
      pc.printf("0x%X, ", IC[ic].sid.sid[2]);
      pc.printf("0x%X, ", IC[ic].sid.sid[3]);
      pc.printf("0x%X, ", IC[ic].sid.sid[4]);
      pc.printf("0x%X, ", IC[ic].sid.sid[5]);
      pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
      pc.printf("PECError:%d\n\n",IC[ic].cccrc.sid_pec);
    }
    else{ pc.printf("Wrong Register Type Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printWriteCommData
* @brief Print Write Comm data.
*
* @details This function Print write comm data.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == Comm)
    {
      pc.printf("Write Comm Data:\n");
      pc.printf("0x%X, ", IC[ic].com.tx_data[0]);
      pc.printf("0x%X, ", IC[ic].com.tx_data[1]);
      pc.printf("0x%X, ", IC[ic].com.tx_data[2]);
      pc.printf("0x%X, ", IC[ic].com.tx_data[3]);
      pc.printf("0x%X, ", IC[ic].com.tx_data[4]);
      pc.printf("0x%X\n\n", IC[ic].com.tx_data[5]);
    }
    else{ pc.printf("Wrong Register Group Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printReadCommData
* @brief Print Read Comm Data.
*
* @details This function print read comm data.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == Comm)
    {
      pc.printf("Read Comm Data:\n");
      pc.printf("ICOM0:0x%X, ", IC[ic].rx_comm.icomm[0]);
      pc.printf("ICOM1:0x%X, ", IC[ic].rx_comm.icomm[1]);
      pc.printf("ICOM2:0x%X\n", IC[ic].rx_comm.icomm[2]);
      pc.printf("FCOM0:0x%X, ", IC[ic].rx_comm.fcomm[0]);
      pc.printf("FCOM1:0x%X, ", IC[ic].rx_comm.fcomm[1]);
      pc.printf("FCOM2:0x%X\n", IC[ic].rx_comm.fcomm[2]);
      pc.printf("DATA0:0x%X, ", IC[ic].rx_comm.data[0]);
      pc.printf("DATA1:0x%X, ", IC[ic].rx_comm.data[1]);
      pc.printf("DATA2:0x%X\n", IC[ic].rx_comm.data[2]);
      pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
      pc.printf("PECError:%d\n\n",IC[ic].cccrc.comm_pec);
    }
    else{ pc.printf("Wrong Register Type Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printCr
* @brief Print Current Result.
*
* @details This function Print current result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printCr(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read Current:\n");
    pc.printf("I1:%fmV, ", getCurrent(IC[ic].i.i1));
    pc.printf("I2:%fmV\n", getCurrent(IC[ic].i.i2));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.cr_pec);
  }
}

/**
*******************************************************************************
* Function: printVoltages
* @brief Print Voltages.
*
* @details This function Print Voltages into terminal.
*
* Parameters:
* @param [in]	tIC    Total IC
*
* @param [in]  *IC    cell_asic stucture pointer
*
* @param [in]  type    Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printVoltage(uint8_t tIC, cell_asic *IC, TYPE type)
{
  float voltage;
  uint16_t temp;
  uint8_t channel;
  uint8_t flag = 0;
  if((type == Vr)){channel = VR_SIZE;}
  else if((type == Vrx)){channel = VRX_SIZE;}
  else if (type == Rvr){ channel = RVR_SIZE;}
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d: \n",(ic+1));
    for(uint8_t index = 0; index < channel; index++)
    {
      if(type == Vr){ temp = IC[ic].vr.v_codes[index];}
      else if(type == Vrx){ temp = IC[ic].vrx.vx_codes[index]; }
      else if(type == Rvr){ temp = IC[ic].rvr.redv_codes[index]; }
      voltage = getVoltage(temp);
      if(type == Vr)
      {
        if(index == 8)
        {
          pc.printf("VREF2A = %7.4fV \n", voltage);
          flag = 1;
        }
        else if(index == (channel-1))
        {
          pc.printf("VREF2B = %7.4fV \n", voltage);
          pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          pc.printf("PECError:%d",IC[ic].cccrc.vr_pec);
        }
        else
        {
          if(flag == 1)
          {
            pc.printf("V%2d = %7.4fV \n", index, voltage);
          }
          else{pc.printf("V%2d = %7.4fV \n",(index+1), voltage);}
        }
      }
      else if(type == Vrx)
      {
        if(index == 4){pc.printf("VREF2A = %7.4fV \n", voltage);}
        else if(index == (channel-1))
        {
          pc.printf("VREF2B = %7.4fV \n", voltage);
          pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          pc.printf("PECError:%d",IC[ic].cccrc.vrx_pec);
        }
        else{pc.printf("V%2d = %7.4fV \n",(index+7), voltage);}
      }
      else if(type == Rvr)
      {
        pc.printf("V%dR=%fV \n",(index+1), voltage);
        if(index == (channel-1))
        {
          pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          pc.printf("PECError:%d",IC[ic].cccrc.rvr_pec);
        }
      }
      else{pc.printf("Wrong Register Group Select\n");}
    }
    pc.printf("\n\n");
  }
}

/**
*******************************************************************************
* Function: printVbat
* @brief Print VBAT Result.
*
* @details This function Print the VBAT result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read VBAT:\n");
    pc.printf("VBAT1: %fV, ", getVoltage(IC[ic].vbat.vbat1));
    pc.printf("VBAT2: %fV, ", getVoltage(IC[ic].vbat.vbat2));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.vbat_pec);
  }
}

/**
*******************************************************************************
* Function: printIvbat
* @brief Print IVBAT result.
*
* @details This function Print the IVBAT result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printIvbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read IVBAT:\n");
    pc.printf("I1:%fmV, ", getCurrent(IC[ic].ivbat.i1));
    pc.printf("VBAT1%fV, ", getVoltage(IC[ic].ivbat.vbat1));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.ivbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgVbat
* @brief Print AVGVBAT result.
*
* @details This function Print the avgvbat result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read AvgVbat:\n");
    pc.printf("VB1AVG:%fV, ", getAvgVbat(IC[ic].vbavg.vb1avg));
    pc.printf("VB2AVG:%fV, ", getAvgVbat(IC[ic].vbavg.vb2avg));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.avgvbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgIVbat
* @brief Print AVGIVBAT result.
*
* @details This function Print the avgivbat result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgIVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read AvgIVbat:\n");
    pc.printf("I1AVG:%fmV, ", getAvgCurrent(IC[ic].i_vbavg.i1avg));
    pc.printf("VB1AVG%fV, ", getAvgVbat(IC[ic].i_vbavg.vb1avg));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.avgivbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgCr
* @brief Print IAVG result.
*
* @details This function Print the iavg result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgCr(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    pc.printf("Read AvgCr:\n");
    pc.printf("I1AVG:%fmV, ", getAvgCurrent(IC[ic].iavg.i1avg));
    pc.printf("I2AVG:%fmV, ", getAvgCurrent(IC[ic].iavg.i2avg));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.avgcr_pec);
  }
}

/**
*******************************************************************************
* Function: printOc
* @brief Print OC result.
*
* @details This function Print the oc result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printOc(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("Read OCR:\n");
    pc.printf("OC1R: %fmV, ", getOverCurrent(IC[ic].oc.oc1r));
    pc.printf("OC2R: %fmV, ", getOverCurrent(IC[ic].oc.oc2r));
    pc.printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    pc.printf("PECError:%d\n\n",IC[ic].cccrc.oc_pec);
  }
}

/**
*******************************************************************************
* Function: PrintStatus
* @brief Print status reg. result.
*
* @details This function Print status result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @param [in]  grp      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/

void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  float voltage;
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    pc.printf("IC%d:\n",(ic+1));
    if(type == Status)
    {
      if(grp == A)
      {
        pc.printf("Status A:\n");
        voltage = getVoltage(IC[ic].stata.vref1p25);
        pc.printf("VREF1P25:%fV, ", voltage);
        pc.printf("ITMP:%f°C, ", (((IC[ic].stata.itmp * 150e-6 )+ 1.5)/0.0075)-273);
        voltage = getVoltage(IC[ic].stata.vreg2);
        pc.printf("VREG2:%fV\n", (voltage + 1.5));
        pc.printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
      }
      else if(grp == B)
      {
        pc.printf("Status B:\n");
        pc.printf("OC1MIN:0x%X, ", IC[ic].statb.oc1min);
        pc.printf("OC1MAX:0x%X, ", IC[ic].statb.oc1max);
        pc.printf("OC2MIN:0x%X, ", IC[ic].statb.oc2min);
        pc.printf("OC2MAX:0x%X\n", IC[ic].statb.oc2max);
        pc.printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
      }
      else if(grp == C)
      {
        pc.printf("Status C:\n");
        pc.printf("OC1A:0x%X, ", IC[ic].statc.oc1a);
        pc.printf("~OC1A:0x%X, ", IC[ic].statc.oc1a_inv);
        pc.printf("OC2A:0x%X, ", IC[ic].statc.oc2a);
        pc.printf("~OC2A:0x%X, ", IC[ic].statc.oc2a_inv);
        pc.printf("CT:0x%X, ", IC[ic].statc.ct);
        pc.printf("CTS:0x%X\n", IC[ic].statc.cts);

        pc.printf("VA_OV:0x%X, ", IC[ic].statc.va_ov);
        pc.printf("VA_UV:0x%X ", IC[ic].statc.va_uv);
        pc.printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
        pc.printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
        pc.printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
        pc.printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
        pc.printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
        pc.printf("OTP2_MED:0x%X\n", IC[ic].statc.otp2_med);

        pc.printf("VDE:0x%X, ", IC[ic].statc.vde);
        pc.printf("VDE1:0x%X, ", IC[ic].statc.vdel);
        pc.printf("INSYNC:0x%X, ", IC[ic].statc.insync);
        pc.printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
        pc.printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
        pc.printf("THSD:0x%X, ", IC[ic].statc.thsd);
        pc.printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
        pc.printf("OSCCHK:0x%X\n", IC[ic].statc.oscchk);

        pc.printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == D)
      {
        pc.printf("Status D:\n");
        pc.printf("OC_CNTR:0x%X, ", IC[ic].statd.oc_cntr);
        pc.printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == E)
      {
        pc.printf("Status E:\n");
        pc.printf("GPIO:0x%X, ", IC[ic].state.gpio);
        pc.printf("GPO:0x%X, ", IC[ic].state.gpo);
        pc.printf("REV_ID:0x%X\n", IC[ic].state.rev);
        pc.printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == ALL_GRP)
      {
        pc.printf("Status A:\n");
        voltage = getVoltage(IC[ic].stata.vref1p25);
        pc.printf("VREF1P25:%fV, ", voltage);
        pc.printf("ITMP:%f°C, ", (((IC[ic].stata.itmp * 150e-6 )+ 1.5)/0.0075)-273);
        voltage = getVoltage(IC[ic].stata.vreg2);
        pc.printf("VREG2:%fV\n", (voltage + 1.5));

        pc.printf("Status B:\n");
        pc.printf("OC1MIN:0x%X, ", IC[ic].statb.oc1min);
        pc.printf("OC1MAX:0x%X, ", IC[ic].statb.oc1max);
        pc.printf("OC2MIN:0x%X, ", IC[ic].statb.oc2min);
        pc.printf("OC2MAX:0x%X\n", IC[ic].statb.oc2max);

        pc.printf("Status C:\n");
        pc.printf("OC1A:0x%X, ", IC[ic].statc.oc1a);
        pc.printf("~OC1A:0x%X, ", IC[ic].statc.oc1a_inv);
        pc.printf("OC2A:0x%X, ", IC[ic].statc.oc2a);
        pc.printf("~OC2A:0x%X, ", IC[ic].statc.oc2a_inv);
        pc.printf("CT:0x%X, ", IC[ic].statc.ct);
        pc.printf("CTS:0x%X\n", IC[ic].statc.cts);
        pc.printf("VA_OV:0x%X, ", IC[ic].statc.va_ov);
        pc.printf("VA_UV:0x%X ", IC[ic].statc.va_uv);
        pc.printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
        pc.printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
        pc.printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
        pc.printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
        pc.printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
        pc.printf("OTP2_MED:0x%X\n", IC[ic].statc.otp2_med);
        pc.printf("VDE:0x%X, ", IC[ic].statc.vde);
        pc.printf("VDE1:0x%X, ", IC[ic].statc.vdel);
        pc.printf("INSYNC:0x%X, ", IC[ic].statc.insync);
        pc.printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
        pc.printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
        pc.printf("THSD:0x%X, ", IC[ic].statc.thsd);
        pc.printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
        pc.printf("OSCCHK:0x%X\n", IC[ic].statc.oscchk);

        pc.printf("Status D:\n");
        pc.printf("OC_CNTR:0x%X\n", IC[ic].statd.oc_cntr);

        pc.printf("Status E:\n");
        pc.printf("GPIO:0x%X, ", IC[ic].state.gpio);
        pc.printf("GPO:0x%X, ", IC[ic].state.gpo);
        pc.printf("REV_ID:0x%X\n", IC[ic].state.rev);
        pc.printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        pc.printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else{ pc.printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: printMenu
* @brief Print Command Menu.
*
* @details This function print all command menu.
*
* @return None
*
*******************************************************************************
*/
void printMenu()
{
  pc.printf("List of ADBMS2950 Command:\n");
  pc.printf("Write and Read Configuration: 1                              |Read Device SID: 26                 \n");
  pc.printf("Read Configuration: 2                                        |Soft Reset: 27                      \n");
  pc.printf("Start ADI1 Conversion(Single Shot): 3                        |Reset cmd counter: 28               \n");
  pc.printf("Start ADI2 Conversion(Single Shot): 4                        |SNAP(Stop Reg. updates): 29         \n");
  pc.printf("Start ADI1 Conversion(Continuous): 5                         |UNSNAP(Resume Reg. updates): 30     \n");
  pc.printf("Start ADI2 Conversion(Continuous): 6                         |Set Reset GPO Pins: 31              \n");
  pc.printf("Start ADI1 Redundant Conversion(Single Shot): 7              |GPIO SPI Write to Slave: 32         \n");
  pc.printf("Start ADI1 Redundant Conversion(Continuous): 8               |GPIO SPI Read from Slave: 33        \n");
  pc.printf("Read CR, VBAT & IVBAT Registers(Single Shot): 9              |GPIO I2C Write to Slave: 34         \n");
  pc.printf("Read CR, VBAT & IVBAT Registers(Continuous): 10              |GPIO I2C Read from Slave: 35        \n");
  pc.printf("Read Overcurrent ADC Register(Single Shot): 11               |                                    \n");
  pc.printf("Read Overcurrent ADC Register(Continuous): 12                |                                    \n");
  pc.printf("Read Average CR, VBAT & IVBAT Registers(Single Shot): 13     |                                    \n");
  pc.printf("Read Average CR, VBAT & IVBAT Registers(Continuous): 14      |                                    \n");
  pc.printf("Read All CR and VBAT Voltage Registers(Single Shot): 15      |                                    \n");
  pc.printf("Read All CR and VBAT Voltage Registers(Continuous): 16       |                                    \n");
  pc.printf("Start ADV All Channel Conversion: 17                         |                                    \n");
  pc.printf("Read All VR Registers(RDV commands): 18                      |                                    \n");
  pc.printf("Read All RVR Registers(RDRVA & RDRVB commands): 19           |                                    \n");
  pc.printf("Read All VR & RVR Registers(RDV, RDRVA & RDRVB commands): 20 |                                    \n");
  pc.printf("Read Voltage Registers(RDAUXC & RDAUXD commands)(NA): 21     |                                    \n");
  pc.printf("Start ADAUX All Channel Conversion: 22                       |                                    \n");
  pc.printf("Read ADAUX Measurement: 23                                   |                                    \n");
  pc.printf("Read Status C Register: 24                                   |                                    \n");
  pc.printf("Read All Status Registers: 25                                |                                    \n");

  pc.printf("\n");
  pc.printf("Print '0' for menu\n");
  pc.printf("Please enter command: \n");
  pc.printf("\n\n");
}
#else /* IAR */

/**
*******************************************************************************
* Function: printWriteConfig
* @brief Print write config A/B result.
*
* @details This function Print write config result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @param [in]  grp      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printWriteConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == Config)
    {
      if(grp == A)
      {
        printf("Write Config A:\n");
        printf("0x%X, ", IC[ic].configa.tx_data[0]);
        printf("0x%X, ", IC[ic].configa.tx_data[1]);
        printf("0x%X, ", IC[ic].configa.tx_data[2]);
        printf("0x%X, ", IC[ic].configa.tx_data[3]);
        printf("0x%X, ", IC[ic].configa.tx_data[4]);
        printf("0x%X\n\n", IC[ic].configa.tx_data[5]);
      }
      else if(grp == B)
      {
        printf("Write Config B:\n");
        printf("0x%X, ", IC[ic].configb.tx_data[0]);
        printf("0x%X, ", IC[ic].configb.tx_data[1]);
        printf("0x%X, ", IC[ic].configb.tx_data[2]);
        printf("0x%X, ", IC[ic].configb.tx_data[3]);
        printf("0x%X, ", IC[ic].configb.tx_data[4]);
        printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
      }
      else if(grp == ALL_GRP)
      {
        printf("Write Config A:\n");
        printf("0x%X, ", IC[ic].configa.tx_data[0]);
        printf("0x%X, ", IC[ic].configa.tx_data[1]);
        printf("0x%X, ", IC[ic].configa.tx_data[2]);
        printf("0x%X, ", IC[ic].configa.tx_data[3]);
        printf("0x%X, ", IC[ic].configa.tx_data[4]);
        printf("0x%X\n\n", IC[ic].configa.tx_data[5]);

        printf("Write Config B:\n");
        printf("0x%X, ", IC[ic].configb.tx_data[0]);
        printf("0x%X, ", IC[ic].configb.tx_data[1]);
        printf("0x%X, ", IC[ic].configb.tx_data[2]);
        printf("0x%X, ", IC[ic].configb.tx_data[3]);
        printf("0x%X, ", IC[ic].configb.tx_data[4]);
        printf("0x%X\n\n", IC[ic].configb.tx_data[5]);
      }
      else{ printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: printReadConfig
* @brief Print Read config A/B result.
*
* @details This function Print read config result into IAR I/O terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @param [in]  TYPE     Enum type of resistor
*
* @param [in]  GRP      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printReadConfig(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == Config)
    {
      if(grp == A)
      {
        printf("Read Config A:\n");
        printf("0x%X, ", IC[ic].configa.rx_data[0]);
        printf("0x%X, ", IC[ic].configa.rx_data[1]);
        printf("0x%X, ", IC[ic].configa.rx_data[2]);
        printf("0x%X, ", IC[ic].configa.rx_data[3]);
        printf("0x%X, ", IC[ic].configa.rx_data[4]);
        printf("0x%X, ", IC[ic].configa.rx_data[5]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else if(grp == B)
      {
        printf("Read Config B:\n");
        printf("0x%X, ", IC[ic].configb.rx_data[0]);
        printf("0x%X, ", IC[ic].configb.rx_data[1]);
        printf("0x%X, ", IC[ic].configb.rx_data[2]);
        printf("0x%X, ", IC[ic].configb.rx_data[3]);
        printf("0x%X, ", IC[ic].configb.rx_data[4]);
        printf("0x%X, ", IC[ic].configb.rx_data[5]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else if(grp == ALL_GRP)
      {
        printf("Read Config A:\n");
        printf("0x%X, ", IC[ic].configa.rx_data[0]);
        printf("0x%X, ", IC[ic].configa.rx_data[1]);
        printf("0x%X, ", IC[ic].configa.rx_data[2]);
        printf("0x%X, ", IC[ic].configa.rx_data[3]);
        printf("0x%X, ", IC[ic].configa.rx_data[4]);
        printf("0x%X, ", IC[ic].configa.rx_data[5]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n",IC[ic].cccrc.cfgr_pec);

        printf("Read Config B:\n");
        printf("0x%X, ", IC[ic].configb.rx_data[0]);
        printf("0x%X, ", IC[ic].configb.rx_data[1]);
        printf("0x%X, ", IC[ic].configb.rx_data[2]);
        printf("0x%X, ", IC[ic].configb.rx_data[3]);
        printf("0x%X, ", IC[ic].configb.rx_data[4]);
        printf("0x%X, ", IC[ic].configb.rx_data[5]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
      }
      else{ printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: PrintDeviceSID
* @brief Print Device SID.
*
* @details This function Print Device SID into IAR I/O terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printDeviceSID(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == SID)
    {
      printf("Read Device SID:\n");
      printf("0x%X, ", IC[ic].sid.sid[0]);
      printf("0x%X, ", IC[ic].sid.sid[1]);
      printf("0x%X, ", IC[ic].sid.sid[2]);
      printf("0x%X, ", IC[ic].sid.sid[3]);
      printf("0x%X, ", IC[ic].sid.sid[4]);
      printf("0x%X, ", IC[ic].sid.sid[5]);
      printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
      printf("PECError:%d\n\n",IC[ic].cccrc.sid_pec);
    }
    else{ printf("Wrong Register Type Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printWriteCommData
* @brief Print Write Comm data.
*
* @details This function Print write comm data.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printWriteCommData(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == Comm)
    {
      printf("Write Comm Data:\n");
      printf("0x%X, ", IC[ic].com.tx_data[0]);
      printf("0x%X, ", IC[ic].com.tx_data[1]);
      printf("0x%X, ", IC[ic].com.tx_data[2]);
      printf("0x%X, ", IC[ic].com.tx_data[3]);
      printf("0x%X, ", IC[ic].com.tx_data[4]);
      printf("0x%X\n\n", IC[ic].com.tx_data[5]);
    }
    else{ printf("Wrong Register Group Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printReadCommData
* @brief Print Read Comm Data.
*
* @details This function print read comm data.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @return None
*
*******************************************************************************
*/
void printReadCommData(uint8_t tIC, cell_asic *IC, TYPE type)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == Comm)
    {
      printf("Read Comm Data:\n");
      printf("ICOM0:0x%X, ", IC[ic].rx_comm.icomm[0]);
      printf("ICOM1:0x%X, ", IC[ic].rx_comm.icomm[1]);
      printf("ICOM2:0x%X\n", IC[ic].rx_comm.icomm[2]);
      printf("FCOM0:0x%X, ", IC[ic].rx_comm.fcomm[0]);
      printf("FCOM1:0x%X, ", IC[ic].rx_comm.fcomm[1]);
      printf("FCOM2:0x%X\n", IC[ic].rx_comm.fcomm[2]);
      printf("DATA0:0x%X, ", IC[ic].rx_comm.data[0]);
      printf("DATA1:0x%X, ", IC[ic].rx_comm.data[1]);
      printf("DATA2:0x%X\n", IC[ic].rx_comm.data[2]);
      printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
      printf("PECError:%d\n\n",IC[ic].cccrc.comm_pec);
    }
    else{ printf("Wrong Register Type Select\n"); }
  }
}

/**
*******************************************************************************
* Function: printCr
* @brief Print Current Result.
*
* @details This function Print current result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printCr(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read Current:\n");
    printf("I1:%fmV, ", getCurrent(IC[ic].i.i1));
    printf("I2:%fmV\n", getCurrent(IC[ic].i.i2));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.cr_pec);
  }
}

/**
*******************************************************************************
* Function: printVoltages
* @brief Print Voltages.
*
* @details This function Print Voltages into terminal.
*
* Parameters:
* @param [in]	tIC    Total IC
*
* @param [in]  *IC    cell_asic stucture pointer
*
* @param [in]  type    Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/
void printVoltage(uint8_t tIC, cell_asic *IC, TYPE type)
{
  float voltage;
  uint16_t temp;
  uint8_t channel;
  uint8_t flag = 0;
  if((type == Vr)){channel = VR_SIZE;}
  else if((type == Vrx)){channel = VRX_SIZE;}
  else if (type == Rvr){ channel = RVR_SIZE;}
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d: \n",(ic+1));
    for(uint8_t index = 0; index < channel; index++)
    {
      if(type == Vr){ temp = IC[ic].vr.v_codes[index];}
      else if(type == Vrx){ temp = IC[ic].vrx.vx_codes[index]; }
      else if(type == Rvr){ temp = IC[ic].rvr.redv_codes[index]; }
      voltage = getVoltage(temp);
      if(type == Vr)
      {
        if(index == 8)
        {
          printf("VREF2A = %7.4fV \n", voltage);
          flag = 1;
        }
        else if(index == (channel-1))
        {
          printf("VREF2B = %7.4fV \n", voltage);
          printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d",IC[ic].cccrc.vr_pec);
        }
        else
        {
          if(flag == 1)
          {
            printf("V%2d = %7.4fV \n", index, voltage);
          }
          else{printf("V%2d = %7.4fV \n",(index+1), voltage);}
        }
      }
      else if(type == Vrx)
      {
        if(index == 4){printf("VREF2A = %7.4fV \n", voltage);}
        else if(index == (channel-1))
        {
          printf("VREF2B = %7.4fV \n", voltage);
          printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d",IC[ic].cccrc.vrx_pec);
        }
        else{printf("V%2d = %7.4fV \n",(index+7), voltage);}
      }
      else if(type == Rvr)
      {
        printf("V%dR=%fV \n",(index+1), voltage);
        if(index == (channel-1))
        {
          printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d",IC[ic].cccrc.rvr_pec);
        }
      }
      else{printf("Wrong Register Group Select\n");}
    }
    printf("\n\n");
  }
}

/**
*******************************************************************************
* Function: printVbat
* @brief Print VBAT Result.
*
* @details This function Print the VBAT result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read VBAT:\n");
    printf("VBAT1: %fV, ", getVoltage(IC[ic].vbat.vbat1));
    printf("VBAT2: %fV, ", getVoltage(IC[ic].vbat.vbat2));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.vbat_pec);
  }
}

/**
*******************************************************************************
* Function: printIvbat
* @brief Print IVBAT result.
*
* @details This function Print the IVBAT result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printIvbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read IVBAT:\n");
    printf("I1:%fmV, ", getCurrent(IC[ic].ivbat.i1));
    printf("VBAT1%fV, ", getVoltage(IC[ic].ivbat.vbat1));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.ivbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgVbat
* @brief Print AVGVBAT result.
*
* @details This function Print the avgvbat result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read AvgVbat:\n");
    printf("VB1AVG:%fV, ", getAvgVbat(IC[ic].vbavg.vb1avg));
    printf("VB2AVG:%fV, ", getAvgVbat(IC[ic].vbavg.vb2avg));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.avgvbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgIVbat
* @brief Print AVGIVBAT result.
*
* @details This function Print the avgivbat result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgIVbat(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read AvgIVbat:\n");
    printf("I1AVG:%fmV, ", getAvgCurrent(IC[ic].i_vbavg.i1avg));
    printf("VB1AVG%fV, ", getAvgVbat(IC[ic].i_vbavg.vb1avg));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.avgivbat_pec);
  }
}

/**
*******************************************************************************
* Function: printAvgCr
* @brief Print IAVG result.
*
* @details This function Print the iavg result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printAvgCr(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    printf("Read AvgCr:\n");
    printf("I1AVG:%fmV, ", getAvgCurrent(IC[ic].iavg.i1avg));
    printf("I2AVG:%fmV, ", getAvgCurrent(IC[ic].iavg.i2avg));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.avgcr_pec);
  }
}

/**
*******************************************************************************
* Function: printOc
* @brief Print OC result.
*
* @details This function Print the oc result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *ic      cell_asic stucture pointer
*
* @return None
*
*******************************************************************************
*/
void printOc(uint8_t tIC, cell_asic *IC)
{
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("Read OCR:\n");
    printf("OC1R: %fmV, ", getOverCurrent(IC[ic].oc.oc1r));
    printf("OC2R: %fmV, ", getOverCurrent(IC[ic].oc.oc2r));
    printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
    printf("PECError:%d\n\n",IC[ic].cccrc.oc_pec);
  }
}

/**
*******************************************************************************
* Function: PrintStatus
* @brief Print status reg. result.
*
* @details This function Print status result into terminal.
*
* Parameters:
* @param [in]	tIC      Total IC
*
* @param [in]  *IC      cell_asic stucture pointer
*
* @param [in]  type     Enum type of resistor
*
* @param [in]  grp      Enum type of resistor group
*
* @return None
*
*******************************************************************************
*/

void printStatus(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
{
  float voltage;
  for(uint8_t ic = 0; ic < tIC; ic++)
  {
    printf("IC%d:\n",(ic+1));
    if(type == Status)
    {
      if(grp == A)
      {
        printf("Status A:\n");
        voltage = getVoltage(IC[ic].stata.vref1p25);
        printf("VREF1P25:%fV, ", voltage);
        printf("ITMP:%f°C, ", (((IC[ic].stata.itmp * 150e-6 )+ 1.5)/0.0075)-273);
        voltage = getVoltage(IC[ic].stata.vreg2);
        printf("VREG2:%fV\n", (voltage + 1.5));
        printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
      }
      else if(grp == B)
      {
        printf("Status B:\n");
        printf("OC1MIN:0x%X, ", IC[ic].statb.oc1min);
        printf("OC1MAX:0x%X, ", IC[ic].statb.oc1max);
        printf("OC2MIN:0x%X, ", IC[ic].statb.oc2min);
        printf("OC2MAX:0x%X\n", IC[ic].statb.oc2max);
        printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
      }
      else if(grp == C)
      {
        printf("Status C:\n");
        printf("OC1A:0x%X, ", IC[ic].statc.oc1a);
        printf("~OC1A:0x%X, ", IC[ic].statc.oc1a_inv);
        printf("OC2A:0x%X, ", IC[ic].statc.oc2a);
        printf("~OC2A:0x%X, ", IC[ic].statc.oc2a_inv);
        printf("CT:0x%X, ", IC[ic].statc.ct);
        printf("CTS:0x%X\n", IC[ic].statc.cts);

        printf("VA_OV:0x%X, ", IC[ic].statc.va_ov);
        printf("VA_UV:0x%X ", IC[ic].statc.va_uv);
        printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
        printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
        printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
        printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
        printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
        printf("OTP2_MED:0x%X\n", IC[ic].statc.otp2_med);

        printf("VDE:0x%X, ", IC[ic].statc.vde);
        printf("VDE1:0x%X, ", IC[ic].statc.vdel);
        printf("INSYNC:0x%X, ", IC[ic].statc.insync);
        printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
        printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
        printf("THSD:0x%X, ", IC[ic].statc.thsd);
        printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
        printf("OSCCHK:0x%X\n", IC[ic].statc.oscchk);

        printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == D)
      {
        printf("Status D:\n");
        printf("OC_CNTR:0x%X, ", IC[ic].statd.oc_cntr);
        printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == E)
      {
        printf("Status E:\n");
        printf("GPIO:0x%X, ", IC[ic].state.gpio);
        printf("GPO:0x%X, ", IC[ic].state.gpo);
        printf("REV_ID:0x%X\n", IC[ic].state.rev);
        printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else if(grp == ALL_GRP)
      {
        printf("Status A:\n");
        voltage = getVoltage(IC[ic].stata.vref1p25);
        printf("VREF1P25:%fV, ", voltage);
        printf("ITMP:%f°C, ", (((IC[ic].stata.itmp * 150e-6 )+ 1.5)/0.0075)-273);
        voltage = getVoltage(IC[ic].stata.vreg2);
        printf("VREG2:%fV\n", (voltage + 1.5));

        printf("Status B:\n");
        printf("OC1MIN:0x%X, ", IC[ic].statb.oc1min);
        printf("OC1MAX:0x%X, ", IC[ic].statb.oc1max);
        printf("OC2MIN:0x%X, ", IC[ic].statb.oc2min);
        printf("OC2MAX:0x%X\n", IC[ic].statb.oc2max);

        printf("Status C:\n");
        printf("OC1A:0x%X, ", IC[ic].statc.oc1a);
        printf("~OC1A:0x%X, ", IC[ic].statc.oc1a_inv);
        printf("OC2A:0x%X, ", IC[ic].statc.oc2a);
        printf("~OC2A:0x%X, ", IC[ic].statc.oc2a_inv);
        printf("CT:0x%X, ", IC[ic].statc.ct);
        printf("CTS:0x%X\n", IC[ic].statc.cts);
        printf("VA_OV:0x%X, ", IC[ic].statc.va_ov);
        printf("VA_UV:0x%X ", IC[ic].statc.va_uv);
        printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
        printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
        printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
        printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
        printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
        printf("OTP2_MED:0x%X\n", IC[ic].statc.otp2_med);
        printf("VDE:0x%X, ", IC[ic].statc.vde);
        printf("VDE1:0x%X, ", IC[ic].statc.vdel);
        printf("INSYNC:0x%X, ", IC[ic].statc.insync);
        printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
        printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
        printf("THSD:0x%X, ", IC[ic].statc.thsd);
        printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
        printf("OSCCHK:0x%X\n", IC[ic].statc.oscchk);

        printf("Status D:\n");
        printf("OC_CNTR:0x%X\n", IC[ic].statd.oc_cntr);

        printf("Status E:\n");
        printf("GPIO:0x%X, ", IC[ic].state.gpio);
        printf("GPO:0x%X, ", IC[ic].state.gpo);
        printf("REV_ID:0x%X\n", IC[ic].state.rev);
        printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
      }
      else{ printf("Wrong Register Group Select\n"); }
    }
  }
}

/**
*******************************************************************************
* Function: printMenu
* @brief Print Command Menu.
*
* @details This function print all command menu.
*
* @return None
*
*******************************************************************************
*/
void printMenu()
{
  printf("List of ADBMS2950 Command:\n");
  printf("Write and Read Configuration: 1                              |Read Device SID: 26                 \n");
  printf("Read Configuration: 2                                        |Soft Reset: 27                      \n");
  printf("Start ADI1 Conversion(Single Shot): 3                        |Reset cmd counter: 28               \n");
  printf("Start ADI2 Conversion(Single Shot): 4                        |SNAP(Stop Reg. updates): 29         \n");
  printf("Start ADI1 Conversion(Continuous): 5                         |UNSNAP(Resume Reg. updates): 30     \n");
  printf("Start ADI2 Conversion(Continuous): 6                         |Set Reset GPO Pins: 31              \n");
  printf("Start ADI1 Redundant Conversion(Single Shot): 7              |GPIO SPI Write to Slave: 32         \n");
  printf("Start ADI1 Redundant Conversion(Continuous): 8               |GPIO SPI Read from Slave: 33        \n");
  printf("Read CR, VBAT & IVBAT Registers(Single Shot): 9              |GPIO I2C Write to Slave: 34         \n");
  printf("Read CR, VBAT & IVBAT Registers(Continuous): 10              |GPIO I2C Read from Slave: 35        \n");
  printf("Read Overcurrent ADC Register(Single Shot): 11               |                                    \n");
  printf("Read Overcurrent ADC Register(Continuous): 12                |                                    \n");
  printf("Read Average CR, VBAT & IVBAT Registers(Single Shot): 13     |                                    \n");
  printf("Read Average CR, VBAT & IVBAT Registers(Continuous): 14      |                                    \n");
  printf("Read All CR and VBAT Voltage Registers(Single Shot): 15      |                                    \n");
  printf("Read All CR and VBAT Voltage Registers(Continuous): 16       |                                    \n");
  printf("Start ADV All Channel Conversion: 17                         |                                    \n");
  printf("Read All VR Registers(RDV commands): 18                      |                                    \n");
  printf("Read All RVR Registers(RDRVA & RDRVB commands): 19           |                                    \n");
  printf("Read All VR & RVR Registers(RDV, RDRVA & RDRVB commands): 20 |                                    \n");
  printf("Read Voltage Registers(RDAUXC & RDAUXD commands)(NA): 21     |                                    \n");
  printf("Start ADAUX All Channel Conversion: 22                       |                                    \n");
  printf("Read ADAUX Measurement: 23                                   |                                    \n");
  printf("Read Status C Register: 24                                   |                                    \n");
  printf("Read All Status Registers: 25                                |                                    \n");

  printf("\n");
  printf("Print '0' for menu\n");
  printf("Please enter command: \n");
  printf("\n\n");
}
#endif /* MBED */

/**
*******************************************************************************
* Function: printMsg
* @brief Print Message.
*
* @details This function print message into terminal.
* Parameters:
* @param [in]	msg    Message string
*
* @return None
*
*******************************************************************************
*/
void printMsg(char *msg)
{
#ifdef MBED
  pc.printf("%s\n\n", msg);
#else /* IAR */
  printf("%s\n\n", msg);
#endif /* MBED */
}

/**
*******************************************************************************
* Function: printPollAdcConvTime
* @brief Print Poll adc conversion Time.
*
* @details This function print poll adc conversion Time.
*
* @return None
*
*******************************************************************************
*/
void printPollAdcConvTime(int count)
{
#ifdef MBED
  pc.printf("Adc Conversion Time = %fms\n", (float)(count/1000.0));
#else /* IAR */
  printf("Adc Conversion Time = %fms\n", (float)(count/64000.0));
#endif /* MBED */
}

/**
*******************************************************************************
* Function: printResultCount
* @brief Print Result Count.
*
* @details This function print the continuous measurment result count.
*
* @return None
*
*******************************************************************************
*/
void printResultCount(int count)
{
#ifdef MBED
  pc.printf("Result Count:%d\n", (count+1));
#else /* IAR */
  printf("Result Count:%d\n", (count+1));
#endif /* MBED */
}

/**
*******************************************************************************
* Function: readUserInupt
* @brief Read command input & print into console.
*
* @details This function print the input command & print into console.
*
* @return None
*
*******************************************************************************
*/
void readUserInupt(int *user_command)
{
#ifdef MBED
    pc.scanf("%d", user_command);
    pc.printf("Enter cmd:%d\n", *user_command);
#else /* IAR */
    scanf("%d", user_command);
    printf("Enter cmd:%d\n", *user_command);
#endif /* MBED */
}

/**
*******************************************************************************
* Function: getVoltage
* @brief Get Voltage with multiplication factor.
*
* @details This function calculates the voltage.
*
* Parameters:
* @param [in]	data    Register value(uint16_t)
*
* @return Voltage(float)
*
*******************************************************************************
*/
float getVoltage(int data)
{
  float voltage;
  voltage = 100e-6 * (int16_t)data; /* Interpreting as 16-bit to be sure of length so signed works */
  return voltage;
}

/**
*******************************************************************************
* Function: getCurrent
* @brief Get Current with multiplication factor.
*
* @details This function calculates the current.
*
* Parameters:
* @param [in]	data    Register value(uint32_t)
*
* @return Current(float)
*
*******************************************************************************
*/
float getCurrent(uint32_t data)
{
  float current;
  current = 1e-6 * ((int32_t)(data << (32-18)) >> (32-18));
  return current;
}

/**
*******************************************************************************
* Function: getAvgCurrent
* @brief Get Average Current with multiplication factor.
*
* @details This function calculates the current.
*
* Parameters:
* @param [in]	data    Register value(uint32_t)
*
* @return Current(float)
*
*******************************************************************************
*/
float getAvgCurrent(uint32_t data)
{
  float current;
  current = 1e-6 * 0.125 * ((int32_t)(data << (32-24)) >> (32-24));
  return current;
}

/**
*******************************************************************************
* Function: getAvgVbat
* @brief Get Average VBAT with multiplication factor.
*
* @details This function calculates the avg vbat.
*
* Parameters:
* @param [in]	data    Register value(uint32_t)
*
* @return Current(float)
*
*******************************************************************************
*/
float getAvgVbat(uint32_t data)
{
  float avgvbat;
  avgvbat = 100e-6 * 0.125 * ((int32_t)(data << (32-24)) >> (32-24));
  return avgvbat;
}

/**
*******************************************************************************
* Function: getOverCurrent
* @brief Get Over Current with multiplication factor.
*
* @details This function calculates the over current.
*
* Parameters:
* @param [in]	data    Register value(uint8_t)
*
* @return Current(float)
*
*******************************************************************************
*/
float getOverCurrent(uint8_t data)
{
  float over_current;
  over_current = 0.005 * ((int8_t)(data << (8-7)) >> (8-7));
  return over_current;
}

/** @}*/
/** @}*/