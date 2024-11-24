
// SOURCE: print_result.c (6830) +  serialPrintResult.c (2950)

#include "common.h"
#include "adbms6830_printResult.h"
#include "adbms6830_data.h"

namespace AD68_NS
{
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
  * @brief Print read config result.
  *
  * @details This function Print read config result into terminal.
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
          printf("REFON:0x%X, ", IC[ic].rx_cfga.refon);
          printf("CTH:0x%X\n", IC[ic].rx_cfga.cth & 0x07);
          printf("FLAG_D[0]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x01));
          printf("FLAG_D[1]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x02)>>1);
          printf("FLAG_D[2]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x04)>>2);
          printf("FLAG_D[3]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x08)>>3);
          printf("FLAG_D[4]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x10)>>4);
          printf("FLAG_D[5]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x20)>>5);
          printf("FLAG_D[6]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x40)>>6);
          printf("FLAG_D[7]:0x%X\n", (IC[ic].rx_cfga.flag_d & 0x80)>>7);
          printf("OWA[2:0]:0x%X, ", (IC[ic].rx_cfga.owa));
          printf("OWRNG:0x%X, ", (IC[ic].rx_cfga.owrng));
          printf("SOAKON:0x%X, ", (IC[ic].rx_cfga.soakon));
          printf("GPO:0x%X, ", (IC[ic].rx_cfga.gpo));
          printf("FC:0x%X, ", (IC[ic].rx_cfga.fc));
          printf("COMM_BK:0x%X, ", (IC[ic].rx_cfga.comm_bk));
          printf("MUTE_ST:0x%X, ", (IC[ic].rx_cfga.mute_st));
          printf("SNAP:0x%X\n\n", (IC[ic].rx_cfga.snap));
          printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
        }
        else if(grp == B)
        {
          printf("Read Config B:\n");
          printf("VUV:0x%X, ", IC[ic].rx_cfgb.vuv);
          printf("VOV:0x%X, ", IC[ic].rx_cfgb.vov);
          printf("DCTO:0x%X, ", IC[ic].rx_cfgb.dcto);
          printf("DTRNG:0x%X, ", IC[ic].rx_cfgb.dtrng);
          printf("DTMEN:0x%X, ", IC[ic].rx_cfgb.dtmen);
          printf("DCC:0x%X\n\n", IC[ic].rx_cfgb.dcc);
          printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n",IC[ic].cccrc.cfgr_pec);
        }
        else if(grp == ALL_GRP)
        {
          printf("Read Config A:\n");
          printf("REFON:0x%X, ", IC[ic].rx_cfga.refon);
          printf("CTH:0x%X\n", IC[ic].rx_cfga.cth & 0x07);
          printf("FLAG_D[0]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x01));
          printf("FLAG_D[1]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x02)>>1);
          printf("FLAG_D[2]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x04)>>2);
          printf("FLAG_D[3]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x08)>>3);
          printf("FLAG_D[4]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x10)>>4);
          printf("FLAG_D[5]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x20)>>5);
          printf("FLAG_D[6]:0x%X, ", (IC[ic].rx_cfga.flag_d & 0x40)>>6);
          printf("FLAG_D[7]:0x%X\n", (IC[ic].rx_cfga.flag_d & 0x80)>>7);
          printf("OWA[2:0]:0x%X, ", (IC[ic].rx_cfga.owa));
          printf("OWRNG:0x%X, ", (IC[ic].rx_cfga.owrng));
          printf("SOAKON:0x%X, ", (IC[ic].rx_cfga.soakon));
          printf("GPO:0x%X, ", (IC[ic].rx_cfga.gpo));
          printf("FC:0x%X, ", (IC[ic].rx_cfga.fc));
          printf("COMM_BK:0x%X, ", (IC[ic].rx_cfga.comm_bk));
          printf("MUTE_ST:0x%X, ", (IC[ic].rx_cfga.mute_st));
          printf("SNAP:0x%X\n\n", (IC[ic].rx_cfga.snap));

          printf("Read Config B:\n");
          printf("VUV:0x%X, ", IC[ic].rx_cfgb.vuv);
          printf("VOV:0x%X, ", IC[ic].rx_cfgb.vov);
          printf("DCTO:0x%X, ", IC[ic].rx_cfgb.dcto);
          printf("DTRNG:0x%X, ", IC[ic].rx_cfgb.dtrng);
          printf("DTMEN:0x%X, ", IC[ic].rx_cfgb.dtmen);
          printf("DCC:0x%X\n\n", IC[ic].rx_cfgb.dcc);
          printf("CCount:%d,", IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n", IC[ic].cccrc.cfgr_pec);
        }
        else{ printf("Wrong Register Group Select\n"); }
      }
    }
  }

  /**
   *******************************************************************************
  * Function: printVoltages
  * @brief Print Voltages.
  *
  * @details This function Print Voltages into IAR I/O terminal.
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
  void printVoltages(uint8_t tIC, cell_asic *IC, TYPE type)
  {
    float voltage;
    int16_t temp;
    uint8_t channel;
    if((type == Cell) || (type == AvgCell) || (type == F_volt) || (type == S_volt))
    {
      channel = CELL;
    }
    else if (type == Aux){ channel = AUX;}
    else if (type == RAux){channel = RAUX;}
    for(uint8_t ic = 0; ic < tIC; ic++)
    {
      printf("IC%d:\n",(ic+1));
      for(uint8_t index = 0; index < channel; index++)
      {
        if(type == Cell){ temp = IC[ic].cell.c_codes[index]; }
        else if(type == AvgCell){ temp = IC[ic].acell.ac_codes[index]; }
        else if(type == F_volt){ temp = IC[ic].fcell.fc_codes[index]; }
        else if(type == S_volt){ temp = IC[ic].scell.sc_codes[index]; }
        else if(type == Aux){ temp = IC[ic].aux.a_codes[index]; }
        else if(type == RAux){ temp = IC[ic].raux.ra_codes[index]; }
        voltage = getVoltage(temp);
        if(type == Cell)
        {
          printf("C%2d=%fV,\n",(index+1), voltage);
          if(index == (channel-1))
          {
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.cell_pec);
          }
        }
        else if(type == AvgCell)
        {
          printf("AC%2d=%fV,\n",(index+1), voltage);
          if(index == (channel-1))
          {
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.acell_pec);
          }
        }
        else if(type == F_volt)
        {
          printf("FC%2d=%fV,\n",(index+1), voltage);
          if(index == (channel-1))
          {
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.fcell_pec);
          }
        }
        else if(type == S_volt)
        {
          printf("S%2d=%fV,\n",(index+1), voltage);
          if(index == (channel-1))
          {
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.scell_pec);
          }
        }
        else if(type == Aux)
        {
          if(index <= 9)
          {
            printf("AUX%2d=%fV,\n",(index+1), voltage);
          }
          else if(index == 10)
          {
            printf("VMV:%2fV,\n",(20 * voltage));
          }
          else if(index == 11)
          {
            printf("V+:%fV,\n",(20 * voltage));
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.aux_pec);
          }
        }
        else if(type == RAux)
        {
          printf("RAUX%d=%2fV,\n",(index+1), voltage);
          if(index == (channel-1))
          {
            printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
            printf("PECError:%d",IC[ic].cccrc.raux_pec);
          }
        }
        else{ printf("Wrong Register Group Select\n"); }
      }
      printf("\n\n");
    }
  }

  /**
   *******************************************************************************
  * Function: PrintStatus
  * @brief Print status reg. result.
  *
  * @details This function Print status result into IAR I/O terminal.
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
          voltage = getVoltage(IC[ic].stata.vref2);
          printf("VREF2:%fV, ", voltage);
          voltage = getVoltage(IC[ic].stata.vref3);
          printf("VREF3:%fV, ", voltage);
          voltage = getVoltage(IC[ic].stata.itmp);
          printf("ITMP:%f�C\n", (voltage/0.0075)-273);

          printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
        }
        else if(grp == B)
        {
          printf("Status B:\n");
          voltage = getVoltage(IC[ic].statb.va);
          printf("VA:%fV, ", voltage);
          voltage = getVoltage(IC[ic].statb.vd);
          printf("VD:%fV, ", voltage);
          voltage = getVoltage(IC[ic].statb.vr4k);
          printf("VR4K:%fV\n", voltage);

          printf("CCount:%d, ",IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n",IC[ic].cccrc.stat_pec);
        }
        else if(grp == C)
        {
          printf("Status C:\n");
          printf("CSFLT:0x%X, ", IC[ic].statc.cs_flt);

          printf("OTP2_MED:0x%X, ", IC[ic].statc.otp2_med);
          printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
          printf("OTP1_MED:0x%X ", IC[ic].statc.otp1_med);
          printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
          printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
          printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
          printf("VA_UV:0x%X, ", IC[ic].statc.va_uv);
          printf("VA_OV:0x%X\n", IC[ic].statc.va_ov);

          printf("OSCCHK:0x%X, ", IC[ic].statc.oscchk);
          printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
          printf("THSD:0x%X, ", IC[ic].statc.thsd);
          printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
          printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
          printf("COMP:0x%X, ", IC[ic].statc.comp);
          printf("VDEL:0x%X, ", IC[ic].statc.vdel);
          printf("VDE:0x%X\n", IC[ic].statc.vde);

          printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
        }
        else if(grp == D)
        {
          printf("Status D:\n");
          printf("C1UV:0x%X, ", IC[ic].statd.c_uv[0]);
          printf("C2UV:0x%X, ", IC[ic].statd.c_uv[1]);
          printf("C3UV:0x%X, ", IC[ic].statd.c_uv[2]);
          printf("C4UV:0x%X, ", IC[ic].statd.c_uv[3]);
          printf("C5UV:0x%X, ", IC[ic].statd.c_uv[4]);
          printf("C6UV:0x%X, ", IC[ic].statd.c_uv[5]);
          printf("C7UV:0x%X, ", IC[ic].statd.c_uv[6]);
          printf("C8UV:0x%X, ", IC[ic].statd.c_uv[7]);
          printf("C9UV:0x%X, ", IC[ic].statd.c_uv[8]);
          printf("C10UV:0x%X, ", IC[ic].statd.c_uv[9]);
          printf("C11UV:0x%X, ", IC[ic].statd.c_uv[10]);
          printf("C12UV:0x%X, ", IC[ic].statd.c_uv[11]);
          printf("C13UV:0x%X, ", IC[ic].statd.c_uv[12]);
          printf("C14UV:0x%X, ", IC[ic].statd.c_uv[13]);
          printf("C15UV:0x%X, ", IC[ic].statd.c_uv[14]);
          printf("C16UV:0x%X\n", IC[ic].statd.c_uv[15]);

          printf("C1OV:0x%X, ", IC[ic].statd.c_ov[0]);
          printf("C2OV:0x%X, ", IC[ic].statd.c_ov[1]);
          printf("C3OV:0x%X, ", IC[ic].statd.c_ov[2]);
          printf("C4OV:0x%X, ", IC[ic].statd.c_ov[3]);
          printf("C5OV:0x%X, ", IC[ic].statd.c_ov[4]);
          printf("C6OV:0x%X, ", IC[ic].statd.c_ov[5]);
          printf("C7OV:0x%X, ", IC[ic].statd.c_ov[6]);
          printf("C8OV:0x%X, ", IC[ic].statd.c_ov[7]);
          printf("C9OV:0x%X, ", IC[ic].statd.c_ov[8]);
          printf("C10OV:0x%X, ", IC[ic].statd.c_ov[9]);
          printf("C11OV:0x%X, ", IC[ic].statd.c_ov[10]);
          printf("C12OV:0x%X, ", IC[ic].statd.c_ov[11]);
          printf("C13OV:0x%X, ", IC[ic].statd.c_ov[12]);
          printf("C14OV:0x%X, ", IC[ic].statd.c_ov[13]);
          printf("C15OV:0x%X, ", IC[ic].statd.c_ov[14]);
          printf("C16OV:0x%X\n", IC[ic].statd.c_ov[15]);

          printf("CTS:0x%X, ", IC[ic].statd.cts);
          printf("CT:0x%X, ", IC[ic].statd.ct);
          printf("OC_CNTR:0x%X\n", IC[ic].statd.oc_cntr);

          printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
        }
        else if(grp == E)
        {
          printf("Status E:\n");
          printf("GPI:0x%X, ", IC[ic].state.gpi);
          printf("REV_ID:0x%X\n", IC[ic].state.rev);

          printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
        }
        else if(grp == ALL_GRP)
        {
          printf("Status A:\n");
          voltage = getVoltage(IC[ic].stata.vref2);
          printf("VREF2:%fV, ", voltage);
          voltage = getVoltage(IC[ic].stata.vref3);
          printf("VREF3:%fV, ", voltage);
          voltage = getVoltage(IC[ic].stata.itmp);
          printf("ITMP:%f�C\n\n", (voltage/0.0075)-273);

          printf("Status B:\n");
          voltage = getVoltage(IC[ic].statb.va);
          printf("VA:%fV, ", voltage);
          voltage = getVoltage(IC[ic].statb.vd);
          printf("VD:%fV, ", voltage);
          voltage = getVoltage(IC[ic].statb.vr4k);
          printf("VR4K:%fV\n\n", voltage);

          printf("Status C:\n");
          printf("CSFLT:0x%X, ", IC[ic].statc.cs_flt);

          printf("OTP2_MED:0x%X, ", IC[ic].statc.otp2_med);
          printf("OTP2_ED:0x%X, ", IC[ic].statc.otp2_ed);
          printf("OTP1_MED:0x%X, ", IC[ic].statc.otp1_med);
          printf("OTP1_ED:0x%X, ", IC[ic].statc.otp1_ed);
          printf("VD_UV:0x%X, ", IC[ic].statc.vd_uv);
          printf("VD_OV:0x%X, ", IC[ic].statc.vd_ov);
          printf("VA_UV:0x%X, ", IC[ic].statc.va_uv);
          printf("VA_OV:0x%X\n", IC[ic].statc.va_ov);

          printf("OSCCHK:0x%X, ", IC[ic].statc.oscchk);
          printf("TMODCHK:0x%X, ", IC[ic].statc.tmodchk);
          printf("THSD:0x%X, ", IC[ic].statc.thsd);
          printf("SLEEP:0x%X, ", IC[ic].statc.sleep);
          printf("SPIFLT:0x%X, ", IC[ic].statc.spiflt);
          printf("COMP:0x%X, ", IC[ic].statc.comp);
          printf("VDEL:0x%X, ", IC[ic].statc.vdel);
          printf("VDE:0x%X\n\n", IC[ic].statc.vde);

          printf("Status D:\n");
          printf("C1UV:0x%X, ", IC[ic].statd.c_uv[0]);
          printf("C2UV:0x%X, ", IC[ic].statd.c_uv[1]);
          printf("C3UV:0x%X, ", IC[ic].statd.c_uv[2]);
          printf("C4UV:0x%X, ", IC[ic].statd.c_uv[3]);
          printf("C5UV:0x%X, ", IC[ic].statd.c_uv[4]);
          printf("C6UV:0x%X, ", IC[ic].statd.c_uv[5]);
          printf("C7UV:0x%X, ", IC[ic].statd.c_uv[6]);
          printf("C8UV:0x%X, ", IC[ic].statd.c_uv[7]);
          printf("C9UV:0x%X, ", IC[ic].statd.c_uv[8]);
          printf("C10UV:0x%X, ", IC[ic].statd.c_uv[9]);
          printf("C11UV:0x%X, ", IC[ic].statd.c_uv[10]);
          printf("C12UV:0x%X, ", IC[ic].statd.c_uv[11]);
          printf("C13UV:0x%X, ", IC[ic].statd.c_uv[12]);
          printf("C14UV:0x%X, ", IC[ic].statd.c_uv[13]);
          printf("C15UV:0x%X, ", IC[ic].statd.c_uv[14]);
          printf("C16UV:0x%X\n", IC[ic].statd.c_uv[15]);

          printf("C1OV:0x%X, ", IC[ic].statd.c_ov[0]);
          printf("C2OV:0x%X, ", IC[ic].statd.c_ov[1]);
          printf("C3OV:0x%X, ", IC[ic].statd.c_ov[2]);
          printf("C4OV:0x%X, ", IC[ic].statd.c_ov[3]);
          printf("C5OV:0x%X, ", IC[ic].statd.c_ov[4]);
          printf("C6OV:0x%X, ", IC[ic].statd.c_ov[5]);
          printf("C7OV:0x%X, ", IC[ic].statd.c_ov[6]);
          printf("C8OV:0x%X, ", IC[ic].statd.c_ov[7]);
          printf("C9OV:0x%X, ", IC[ic].statd.c_ov[8]);
          printf("C10OV:0x%X, ", IC[ic].statd.c_ov[9]);
          printf("C11OV:0x%X, ", IC[ic].statd.c_ov[10]);
          printf("C12OV:0x%X, ", IC[ic].statd.c_ov[11]);
          printf("C13OV:0x%X, ", IC[ic].statd.c_ov[12]);
          printf("C14OV:0x%X, ", IC[ic].statd.c_ov[13]);
          printf("C15OV:0x%X, ", IC[ic].statd.c_ov[14]);
          printf("C16OV:0x%X\n", IC[ic].statd.c_ov[15]);

          printf("CTS:0x%X, ", IC[ic].statd.cts);
          printf("CT:0x%X\n\n", IC[ic].statd.ct);

          printf("Status E:\n");
          printf("GPI:0x%X, ", IC[ic].state.gpi);
          printf("REV_ID:0x%X\n\n", IC[ic].state.rev);

          printf("CCount:%d, ", IC[ic].cccrc.cmd_cntr);
          printf("PECError:%d\n\n", IC[ic].cccrc.stat_pec);
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
      if(type == Sid)
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
  * Function: printWritePwmDutyCycle
  * @brief Print Write Pwm Duty Cycle.
  *
  * @details This function Print write pwm duty cycle value.
  *
  * Parameters:
  * @param [in]	tIC      Total IC
  *
  * @param [in]  *IC      cell_asic stucture pointer
  *
  * @param [in]  type     Enum type of resistor
  *
  * @param [in]  grp      Enum group of resistor
  *
  * @return None
  *
  *******************************************************************************
  */
  void printWritePwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
  {
    for(uint8_t ic = 0; ic < tIC; ic++)
    {
      printf("IC%d:\n",(ic+1));
      if(grp == A)
      {
        printf("Write Pwma Duty Cycle:\n");
        printf("0x%X, ", IC[ic].pwma.tx_data[0]);
        printf("0x%X, ", IC[ic].pwma.tx_data[1]);
        printf("0x%X, ", IC[ic].pwma.tx_data[2]);
        printf("0x%X, ", IC[ic].pwma.tx_data[3]);
        printf("0x%X, ", IC[ic].pwma.tx_data[4]);
        printf("0x%X\n\n", IC[ic].pwma.tx_data[5]);
      }
      else if(grp == B)
      {
        printf("Write Pwmb Duty Cycle:\n");
        printf("0x%X, ", IC[ic].pwmb.tx_data[0]);
        printf("0x%X\n\n", IC[ic].pwmb.tx_data[1]);
      }
      else if(grp == ALL_GRP)
      {
        printf("Write Pwma Duty Cycle:\n");
        printf("0x%X, ", IC[ic].pwma.tx_data[0]);
        printf("0x%X, ", IC[ic].pwma.tx_data[1]);
        printf("0x%X, ", IC[ic].pwma.tx_data[2]);
        printf("0x%X, ", IC[ic].pwma.tx_data[3]);
        printf("0x%X, ", IC[ic].pwma.tx_data[4]);
        printf("0x%X\n", IC[ic].pwma.tx_data[5]);

        printf("Write Pwmb Duty Cycle:\n");
        printf("0x%X, ", IC[ic].pwmb.tx_data[0]);
        printf("0x%X\n\n", IC[ic].pwmb.tx_data[1]);
      }
      else{ printf("Wrong Register Group Select\n"); }
    }
  }

  /**
  *******************************************************************************
  * Function: printReadPwmDutyCycle
  * @brief Print Read Pwm Duty Cycle.
  *
  * @details This function print read pwm duty cycle value.
  *
  * Parameters:
  * @param [in]	tIC      Total IC
  *
  * @param [in]  *IC      cell_asic stucture pointer
  *
  * @param [in]  type     Enum type of resistor
  *
  * @param [in]  grp      Enum group of resistor
  *
  * @return None
  *
  *******************************************************************************
  */
  void printReadPwmDutyCycle(uint8_t tIC, cell_asic *IC, TYPE type, GRP grp)
  {
    for(uint8_t ic = 0; ic < tIC; ic++)
    {
      printf("IC%d:\n",(ic+1));
      if(grp == A)
      {
        printf("Read PWMA Duty Cycle:\n");
        printf("PWM1:0x%X, ", IC[ic].PwmA.pwma[0]);
        printf("PWM2:0x%X, ", IC[ic].PwmA.pwma[1]);
        printf("PWM3:0x%X, ", IC[ic].PwmA.pwma[2]);
        printf("PWM4:0x%X, ", IC[ic].PwmA.pwma[3]);
        printf("PWM5:0x%X, ", IC[ic].PwmA.pwma[4]);
        printf("PWM6:0x%X, ", IC[ic].PwmA.pwma[5]);
        printf("PWM7:0x%X, ", IC[ic].PwmA.pwma[6]);
        printf("PWM8:0x%X, ", IC[ic].PwmA.pwma[7]);
        printf("PWM9:0x%X, ", IC[ic].PwmA.pwma[8]);
        printf("PWM10:0x%X, ", IC[ic].PwmA.pwma[9]);
        printf("PWM11:0x%X, ", IC[ic].PwmA.pwma[10]);
        printf("PWM12:0x%X, ", IC[ic].PwmA.pwma[11]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.pwm_pec);
      }
      else if(grp == B)
      {
        printf("Read PWMB Duty Cycle:\n");
        printf("PWM13:0x%X, ", IC[ic].PwmB.pwmb[0]);
        printf("PWM14:0x%X, ", IC[ic].PwmB.pwmb[1]);
        printf("PWM15:0x%X, ", IC[ic].PwmB.pwmb[2]);
        printf("PWM16:0x%X, ", IC[ic].PwmB.pwmb[3]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.pwm_pec);
      }
      else if(grp == ALL_GRP)
      {
        printf("Read PWMA Duty Cycle:\n");
        printf("PWM1:0x%X, ", IC[ic].PwmA.pwma[0]);
        printf("PWM2:0x%X, ", IC[ic].PwmA.pwma[1]);
        printf("PWM3:0x%X, ", IC[ic].PwmA.pwma[2]);
        printf("PWM4:0x%X, ", IC[ic].PwmA.pwma[3]);
        printf("PWM5:0x%X, ", IC[ic].PwmA.pwma[4]);
        printf("PWM6:0x%X, ", IC[ic].PwmA.pwma[5]);
        printf("PWM7:0x%X, ", IC[ic].PwmA.pwma[6]);
        printf("PWM8:0x%X, ", IC[ic].PwmA.pwma[7]);
        printf("PWM9:0x%X, ", IC[ic].PwmA.pwma[8]);
        printf("PWM10:0x%X, ", IC[ic].PwmA.pwma[9]);
        printf("PWM11:0x%X, ", IC[ic].PwmA.pwma[10]);
        printf("PWM12:0x%X\n", IC[ic].PwmA.pwma[11]);

        printf("Read PWMB Duty Cycle:\n");
        printf("PWM13:0x%X, ", IC[ic].PwmB.pwmb[0]);
        printf("PWM14:0x%X, ", IC[ic].PwmB.pwmb[1]);
        printf("PWM15:0x%X, ", IC[ic].PwmB.pwmb[2]);
        printf("PWM16:0x%X, ", IC[ic].PwmB.pwmb[3]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.pwm_pec);
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
        printf("ICOM0:0x%X, ", IC[ic].comm.icomm[0]);
        printf("ICOM1:0x%X, ", IC[ic].comm.icomm[1]);
        printf("ICOM2:0x%X\n", IC[ic].comm.icomm[2]);
        printf("FCOM0:0x%X, ", IC[ic].comm.fcomm[0]);
        printf("FCOM1:0x%X, ", IC[ic].comm.fcomm[1]);
        printf("FCOM2:0x%X\n", IC[ic].comm.fcomm[2]);
        printf("DATA0:0x%X, ", IC[ic].comm.data[0]);
        printf("DATA1:0x%X, ", IC[ic].comm.data[1]);
        printf("DATA2:0x%X\n", IC[ic].comm.data[2]);
        printf("CCount:%d,",IC[ic].cccrc.cmd_cntr);
        printf("PECError:%d\n\n",IC[ic].cccrc.comm_pec);
      }
      else{ printf("Wrong Register Type Select\n"); }
    }
  }

  /**
  *******************************************************************************
  * Function: printDiagnosticTestResult
  * @brief Print diagnostic test result.
  *
  * @details This function Print diagnostic test result (PASS,FAIL) into console terminal.
  *
  * Parameters:
  * @param [in]	tIC      Total IC
  *
  * @param [in]  *IC      cell_asic stucture pointer
  *
  * @param [in]  TEST     Enum type diagnostic test
  *
  * @return None
  *
  *******************************************************************************
  */
  void printDiagnosticTestResult(uint8_t tIC, cell_asic *IC, DIAGNOSTIC_TYPE type)
  {
    if(type == OSC_MISMATCH)
    {
      printf("OSC Diagnostic Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.osc_mismatch);
      }
      printf("\n\n");
    }

    else if(type == SUPPLY_ERROR)
    {
      printf("Force Supply Error Detection Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.supply_error);
      }
      printf("\n\n");
    }

    else if(type == THSD)
    {
      printf("Thsd Diagnostic Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.thsd);
      }
      printf("\n\n");
    }

    else if(type == FUSE_ED)
    {
      printf("Fuse_ed Diagnostic Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.fuse_ed);
      }
      printf("\n\n");
    }

    else if(type == FUSE_MED)
    {
      printf("Fuse_med Diagnostic Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.fuse_med);
      }
      printf("\n\n");
    }

    else if(type == TMODCHK)
    {
      printf("TMODCHK Diagnostic Test:\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:",(ic+1));
        diagnosticTestResultPrint(IC[ic].diag_result.tmodchk);
      }
      printf("\n\n");
    }
    else{printf("Wrong Diagnostic Selected\n");}
  }

  /**
  *******************************************************************************
  * Function: diagnosticResultPrint
  * @brief Print diagnostic (PASS/FAIL) result.
  *
  * @details This function print diagnostic (PASS/FAIL) result into console.
  *
  * Parameters:
  * @param [in]	result   Result byte
  *
  * @return None
  *
  *******************************************************************************
  */
  void diagnosticTestResultPrint(uint8_t result)
  {
    if(result == 1)
    {
      printf("PASS\n");
    }
    else
    {
      printf("FAIL\n");
    }
  }

  /**
  *******************************************************************************
  * Function: printOpenWireTestResult
  * @brief Print open wire test result.
  *
  * @details This function print open wire test result.
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
  void printOpenWireTestResult(uint8_t tIC, cell_asic *IC, TYPE type)
  {
    if(type == Cell)
    {
      printf("Cell Open Wire Test\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:\n",(ic+1));
        for(uint8_t cell = 0; cell < CELL; cell++)
        {
          printf("CELL%d:",(cell+1));
          openWireResultPrint(IC[ic].diag_result.cell_ow[cell]);
        }
        printf("\n\n");
      }
    }
    else if(type == S_volt)
    {
      printf("Cell redundant Open Wire Test\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:\n",(ic+1));
        for(uint8_t cell = 0; cell < CELL; cell++)
        {
          printf("CELL%d:",(cell+1));
          openWireResultPrint(IC[ic].diag_result.cellred_ow[cell]);
        }
        printf("\n\n");
      }
    }
    else if(type == Aux)
    {
      printf("Aux Open Wire Test\n");
      for(uint8_t ic = 0; ic < tIC; ic++)
      {
        printf("IC%d:\n",(ic+1));
        for(uint8_t gpio = 0; gpio < (AUX-2); gpio++)
        {
          printf("GPIO%d:",(gpio+1));
          openWireResultPrint(IC[ic].diag_result.aux_ow[gpio]);
        }
        printf("\n\n");
      }
    }
    else{printf("Wrong Resistor Type Selected\n");}
  }

  /**
  *******************************************************************************
  * Function: openWireResultPrint
  * @brief Print open wire (OPEN/CLOSE) result.
  *
  * @details This function print open wire result into console.
  *
  * Parameters:
  * @param [in]	result   Result byte
  *
  * @return None
  *
  *******************************************************************************
  */
  void openWireResultPrint(uint8_t result)
  {
    if(result == 1)
    {
      printf(" OPEN\n");
    }
    else
    {
      printf(" CLOSE\n");
    }
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
    printf("Adc Conversion Time = %d us\n", count); // Changed original to display us instead
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
    printf("List of ADBMS6830 Command:\n");
    printf("Write and Read Configuration: 1 \n");
    printf("Read Configuration: 2 \n");
    printf("Start Cell Voltage Conversion: 3 \n");
    printf("Read Cell Voltages: 4 \n");
    printf("Start S-Voltage Conversion: 5 \n");
    printf("Read S-Voltages: 6 \n");
    printf("Start Avg Cell Voltage Conversion: 7 \n");
    printf("Read Avg Cell Voltages: 8 \n");
    printf("Start F-Cell Voltage Conversion: 9 \n");
    printf("Read F-Cell Voltages: 10 \n");
    printf("Start Aux Voltage Conversion: 11 \n");
    printf("Read Aux Voltages: 12 \n");
    printf("Start RAux Voltage Conversion: 13 \n");
    printf("Read RAux Voltages: 14 \n");
    printf("Read Status Registers: 15 \n");
    printf("Loop Measurements: 16 \n");
    printf("Clear Cell registers: 17 \n");
    printf("Clear Aux registers: 18 \n");
    printf("Clear Spin registers: 19 \n");
    printf("Clear Fcell registers: 20 \n");
    printf("Write Configuration: 21 \n");

    printf("\n");
    printf("Print '0' for menu\n");
    printf("Please enter command: \n");
    printf("\n\n");
  }

  /**
   *******************************************************************************
  * Function: getVoltage
  * @brief Get voltages with multiplication factor.
  *
  * @details This function calculate the voltage.
  *
  * Parameters:
  * @param [in]	data    voltages(uint16_t)
  *
  * @return voltage(float)
  *
  *******************************************************************************
  */
  float getVoltage(int data)
  {
      float voltage_float; //voltage in Volts
      voltage_float = ((data + 10000) * 0.000150);
      return voltage_float;
  }
}
