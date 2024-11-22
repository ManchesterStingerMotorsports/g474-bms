
// SOURCE: adBms_Application.c (6830) + adi_2950/adbms_2950 (2950)

#include "common.h"
#include "adbms_libWrapper.h"

#include "adbms_utility.h"
#include "adbms6830_parseCreate.h"
#include "adbms_mcuWrapper.h"

#include "adbms_config.h"
#include "adbms_cmdlist.h"

#include "adbms6830_printResult.h"
#include "adbms2950_printResult.h"


namespace AD68_NS
{
  /**
  *******************************************************************************
  * @brief Set configuration register A. Refer to the data sheet
  *        Set configuration register B. Refer to the data sheet
  *******************************************************************************
  */
  void adBms6830_init_config(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      /* Init config A */
      ic[cic].tx_cfga.refon = PWR_UP;
  //    ic[cic].cfga.cth = CVT_8_1mV;
  //    ic[cic].cfga.flag_d = ConfigA_Flag(FLAG_D0, FLAG_SET) | ConfigA_Flag(FLAG_D1, FLAG_SET);
  //    ic[cic].cfga.gpo = ConfigA_Gpo(GPO2, GPO_SET) | ConfigA_Gpo(GPO10, GPO_SET);
      ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
  //    ic[cic].cfga.soakon = SOAKON_CLR;
  //    ic[cic].cfga.fc = IIR_FPA256;

      /* Init config B */
  //    ic[cic].cfgb.dtmen = DTMEN_ON;
      ic[cic].tx_cfgb.vov = SetOverVoltageThreshold(OV_THRESHOLD);
      ic[cic].tx_cfgb.vuv = SetUnderVoltageThreshold(UV_THRESHOLD);
  //    ic[cic].cfgb.dcc = ConfigB_DccBit(DCC16, DCC_BIT_SET);
  //    SetConfigB_DischargeTimeOutValue(tIC, &ic[cic], RANG_0_TO_63_MIN, TIME_1MIN_OR_0_26HR);
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
  }

  /**
  *******************************************************************************
  * @brief Write and Read Configuration Register A/B
  *******************************************************************************
  */
  void adBms6830_write_read_config(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
    adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
    printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
    printReadConfig(tIC, &ic[0], Config, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Write Configuration Register A/B
  *******************************************************************************
  */
  void adBms6830_write_config(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
    printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Read Configuration Register A/B
  *******************************************************************************
  */
  void adBms6830_read_config(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
    printReadConfig(tIC, &ic[0], Config, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Start ADC Cell Voltage Measurement
  *******************************************************************************
  */
  void adBms6830_start_adc_cell_voltage_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Cell conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Cell Voltages
  *******************************************************************************
  */
  void adBms6830_read_cell_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDCVA, Cell, A);
    adBmsReadData(tIC, &ic[0], RDCVB, Cell, B);
    adBmsReadData(tIC, &ic[0], RDCVC, Cell, C);
    adBmsReadData(tIC, &ic[0], RDCVD, Cell, D);
    adBmsReadData(tIC, &ic[0], RDCVE, Cell, E);
    adBmsReadData(tIC, &ic[0], RDCVF, Cell, F);
    printVoltages(tIC, &ic[0], Cell);
  }

  /**
  *******************************************************************************
  * @brief Start ADC S-Voltage Measurement
  *******************************************************************************
  */
  void adBms6830_start_adc_s_voltage_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms6830_Adsv(CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, CELL_OPEN_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("S-Voltage conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read S-Voltages
  *******************************************************************************
  */
  void adBms6830_read_s_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDSVA, S_volt, A);
    adBmsReadData(tIC, &ic[0], RDSVB, S_volt, B);
    adBmsReadData(tIC, &ic[0], RDSVC, S_volt, C);
    adBmsReadData(tIC, &ic[0], RDSVD, S_volt, D);
    adBmsReadData(tIC, &ic[0], RDSVE, S_volt, E);
    adBmsReadData(tIC, &ic[0], RDSVF, S_volt, F);
    printVoltages(tIC, &ic[0], S_volt);
  }

  /**
  *******************************************************************************
  * @brief Start Avarage Cell Voltage Measurement
  *******************************************************************************
  */
  void adBms6830_start_avgcell_voltage_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms6830_Adcv(RD_ON, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Avg Cell voltage conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Avarage Cell Voltages
  *******************************************************************************
  */
  void adBms6830_read_avgcell_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDACA, AvgCell, A);
    adBmsReadData(tIC, &ic[0], RDACB, AvgCell, B);
    adBmsReadData(tIC, &ic[0], RDACC, AvgCell, C);
    adBmsReadData(tIC, &ic[0], RDACD, AvgCell, D);
    adBmsReadData(tIC, &ic[0], RDACE, AvgCell, E);
    adBmsReadData(tIC, &ic[0], RDACF, AvgCell, F);
    printVoltages(tIC, &ic[0], AvgCell);
  }

  /**
  *******************************************************************************
  * @brief Start Filtered Cell Voltages Measurement
  *******************************************************************************
  */
  void adBms6830_start_fcell_voltage_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("F Cell voltage conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Filtered Cell Voltages
  *******************************************************************************
  */
  void adBms6830_read_fcell_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDFCA, F_volt, A);
    adBmsReadData(tIC, &ic[0], RDFCB, F_volt, B);
    adBmsReadData(tIC, &ic[0], RDFCC, F_volt, C);
    adBmsReadData(tIC, &ic[0], RDFCD, F_volt, D);
    adBmsReadData(tIC, &ic[0], RDFCE, F_volt, E);
    adBmsReadData(tIC, &ic[0], RDFCF, F_volt, F);
    printVoltages(tIC, &ic[0], F_volt);
  }

  /**
  *******************************************************************************
  * @brief Start AUX, VMV, V+ Voltages Measurement
  *******************************************************************************
  */
  void adBms6830_start_aux_voltage_measurment(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      /* Init config A */
      ic[cic].tx_cfga.refon = PWR_UP;
      ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE, AUX_CH_TO_CONVERT);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Aux voltage conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read AUX, VMV, V+ Voltages
  *******************************************************************************
  */
  void adBms6830_read_aux_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDAUXA, Aux, A);
    adBmsReadData(tIC, &ic[0], RDAUXB, Aux, B);
    adBmsReadData(tIC, &ic[0], RDAUXC, Aux, C);
    adBmsReadData(tIC, &ic[0], RDAUXD, Aux, D);
    printVoltages(tIC, &ic[0], Aux);
  }

  /**
  *******************************************************************************
  * @brief Start Redundant GPIO Voltages Measurement
  *******************************************************************************
  */
  void adBms6830_start_raux_voltage_measurment(uint8_t tIC,  cell_asic *ic)
  {
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      /* Init config A */
      ic[cic].tx_cfga.refon = PWR_UP;
      ic[cic].tx_cfga.gpo = 0X3FF; /* All GPIO pull down off */
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBms6830_Adax2(AUX_CH_TO_CONVERT);
    pladc_count = adBmsPollAdc(PLADC);
    printf("RAux voltage conversion completed\n");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Redundant GPIO Voltages
  *******************************************************************************
  */
  void adBms6830_read_raux_voltages(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDRAXA, RAux, A);
    adBmsReadData(tIC, &ic[0], RDRAXB, RAux, B);
    adBmsReadData(tIC, &ic[0], RDRAXC, RAux, C);
    adBmsReadData(tIC, &ic[0], RDRAXD, RAux, D);
    printVoltages(tIC, &ic[0], RAux);
  }

  /**
  *******************************************************************************
  * @brief Read Status Reg. A, B, C, D and E.
  *******************************************************************************
  */
  void adBms6830_read_status_registers(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
    adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE, AUX_CH_TO_CONVERT);
    pladc_count = adBmsPollAdc(PLADC);
    adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    pladc_count = pladc_count + adBmsPollAdc(PLADC);

    adBmsReadData(tIC, &ic[0], RDSTATA, Status, A);
    adBmsReadData(tIC, &ic[0], RDSTATB, Status, B);
    adBmsReadData(tIC, &ic[0], RDSTATC, Status, C);
    adBmsReadData(tIC, &ic[0], RDSTATD, Status, D);
    adBmsReadData(tIC, &ic[0], RDSTATE, Status, E);
    printPollAdcConvTime(pladc_count);
    printStatus(tIC, &ic[0], Status, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Clear Cell measurement reg.
  *******************************************************************************
  */
  void adBms6830_clear_cell_measurement(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(CLRCELL);
    printf("Cell Registers Cleared\n\n");
  }

  /**
  *******************************************************************************
  * @brief Clear Aux measurement reg.
  *******************************************************************************
  */
  void adBms6830_clear_aux_measurement(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(CLRAUX);
    printf("Aux Registers Cleared\n\n");
  }

  /**
  *******************************************************************************
  * @brief Clear spin measurement reg.
  *******************************************************************************
  */
  void adBms6830_clear_spin_measurement(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(CLRSPIN);
    printf("Spin Registers Cleared\n\n");
  }

  /**
  *******************************************************************************
  * @brief Clear fcell measurement reg.
  *******************************************************************************
  */
  void adBms6830_clear_fcell_measurement(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(CLRFC);
    printf("Fcell Registers Cleared\n\n");
  }
}



///////////////////////////////////////////////////// 2950 //////////////////////////////////////////////////////////////



namespace AD29_NS
{
  /**
  *******************************************************************************
  * @brief Set configuration register A. Refer to the data sheet
  *        Set configuration register B. Refer to the data sheet
  *******************************************************************************
  */
  void adi2950_init_config(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      /* Init config A */
      ic[cic].tx_cfga.refon = PWR_UP;

      /* Init config B */
      ic[cic].tx_cfgb.vs2 = VSM_SGND;
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
  }

  /**
  *******************************************************************************
  * @brief Write and Read Configuration Register A/B
  *******************************************************************************
  */
  void adi2950_write_read_config(uint8_t tIC, cell_asic *ic)
  {
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      /* Init config A */
      ic[cic].tx_cfga.refon = PWR_UP;
      /* Init config B */
      ic[cic].tx_cfgb.vs2 = VSM_SGND;
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsWriteData(tIC, &ic[0], WRCFGB, Config, B);
    adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
    printWriteConfig(tIC, &ic[0], Config, ALL_GRP);
    printReadConfig(tIC, &ic[0], Config, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Read Configuration Register A/B
  *******************************************************************************
  */
  void adi2950_read_config(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDCFGB, Config, B);
    printReadConfig(tIC, &ic[0], Config, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Start Adi1 Single Measurement
  *******************************************************************************
  */
  void adi2950_start_adi1_single_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi1(REDUNDANT_MEASUREMENT, CONTINUOUS_MEASUREMENT, OW_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Adi1 conversion completed");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Start Adi1 Continuous Measurement
  *******************************************************************************
  */
  void adi2950_start_adi1_continuous_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi1(REDUNDANT_MEASUREMENT, CONTINUOUS, OW_WIRE_DETECTION);
    Delay_ms(8); /* As per data sheet current register update rate(1ms) min  & average current register update rate (8ms) max*/
    printf("Adi1 conversion completed");
  }

  /**
  *******************************************************************************
  * @brief Start Adi2 Single Measurement
  *******************************************************************************
  */
  void adi2950_start_adi2_single_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi2(CONTINUOUS_MEASUREMENT, OW_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Adi2 conversion completed");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Start Adi2 Continuous Measurement
  *******************************************************************************
  */
  void adi2950_start_adi2_continuous_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi2(CONTINUOUS, OW_WIRE_DETECTION);
    Delay_ms(8); /* As per data sheet current register update rate(1ms) min  & average current register update rate (8ms) max*/
    printf("Adi2 conversion completed");
  }

  /**
  *******************************************************************************
  * @brief Start Adi1 Redundant Single Measurement
  *******************************************************************************
  */
  void adi2950_start_adi1_redundant_single_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi1(RD_ON, CONTINUOUS_MEASUREMENT, OW_WIRE_DETECTION);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Adi1 redundant conversion completed");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Start Adi1 Redundant Continuous Measurement
  *******************************************************************************
  */
  void adi2950_start_adi1_redundant_continuous_measurment(uint8_t tIC)
  {
    soft_reset(tIC);
    adBmsWakeupIc(tIC);
    adBms2950_Adi1(RD_ON, CONTINUOUS, OW_WIRE_DETECTION);
    Delay_ms(8); /* As per data sheet current register update rate(1ms) min  & average current register update rate (8ms) max*/
    printf("Adi1 redundant conversion completed");
  }

  /**
  *******************************************************************************
  * @brief Read Current, Battery & Current and Battery Registers
  *******************************************************************************
  */
  void adi2950_read_cr_vbat_ivbat_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDI, Cr, NONE);        /* Current Register Group */
      adBmsReadData(tIC, &ic[0], RDVBAT, Vbat, NONE);   /* Battery Voltage Group*/
      adBmsReadData(tIC, &ic[0], RDIVBAT, Ivbat, NONE); /* Current and Battery Voltage Group */
      printCr(tIC, &ic[0]);
      printVbat(tIC, &ic[0]);
      printIvbat(tIC, &ic[0]);
  }

  /**
  *******************************************************************************
  * @brief Read Overcurrent ADC Register
  *******************************************************************************
  */
  void adi2950_read_ocr_register(uint8_t tIC, cell_asic *ic)
  {
      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        /* Init config A */
        ic[cic].tx_cfga.refon = PWR_UP;
      }
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDOCR, Oc, NONE);
      printOc(tIC, &ic[0]);
  }

  /**
  *******************************************************************************
  * @brief Read (Average Current), (Average Batter)  & (Average Battery Current and Voltage) Registers
  *******************************************************************************
  */
  void adi2950_read_avgcr_avgvbat_avgivbat_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDIAV, AvgCr, NONE);
      adBmsReadData(tIC, &ic[0], RDVBAV, AvgVbat, NONE);
      adBmsReadData(tIC, &ic[0], RDIVBAV, AvgIvbat, NONE);
      printAvgCr(tIC, &ic[0]);
      printAvgVbat(tIC, &ic[0]);
      printAvgIVbat(tIC, &ic[0]);
  }

  /**
  *******************************************************************************
  * @brief Read All Current & Battery Voltage Registers
  *******************************************************************************
  */
  void adi2950_all_current_battery_voltage_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDI, Cr, NONE);
      adBmsReadData(tIC, &ic[0], RDVBAT, Vbat, NONE);
      adBmsReadData(tIC, &ic[0], RDIVBAT, Ivbat, NONE);

      adBmsReadData(tIC, &ic[0], RDOCR, Oc, NONE);

      adBmsReadData(tIC, &ic[0], RDIAV, AvgCr, NONE);
      adBmsReadData(tIC, &ic[0], RDVBAV, AvgVbat, NONE);
      adBmsReadData(tIC, &ic[0], RDIVBAV, AvgIvbat, NONE);
      adBmsReadData(tIC, &ic[0], RDIVBAV, AvgIvbat, NONE);

      printCr(tIC, &ic[0]);
      printVbat(tIC, &ic[0]);
      printIvbat(tIC, &ic[0]);
      printOc(tIC, &ic[0]);
      printAvgCr(tIC, &ic[0]);
      printAvgVbat(tIC, &ic[0]);
      printAvgIVbat(tIC, &ic[0]);
      printAvgIVbat(tIC, &ic[0]);
  }

  /**
  *******************************************************************************
  * @brief Start Adv All Channel Measurement
  *******************************************************************************
  */
  void adi2950_start_adv_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms2950_Adv(OW_WIRE_DETECTION, RR_VCH0_VCH8);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Adv conversion completed");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Vr registers
  *******************************************************************************
  */
  void adi2950_read_vr_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDVA, Vr, A);
      adBmsReadData(tIC, &ic[0], RDVB, Vr, B);
      adBmsReadData(tIC, &ic[0], RDVC, Vr, C);
      adBmsReadData(tIC, &ic[0], RDVD, Vr, D);
      printVoltage(tIC, &ic[0], Vr);
  }

  /**
  *******************************************************************************
  * @brief Read Rvr registers
  *******************************************************************************
  */
  void adi2950_read_rvr_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDRVA, Rvr, A);
      adBmsReadData(tIC, &ic[0], RDRVB, Rvr, B);
      printVoltage(tIC, &ic[0], Rvr);
  }

  void adi2950_read_vr_rvr_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDVA, Vr, A);
      adBmsReadData(tIC, &ic[0], RDVB, Vr, B);
      adBmsReadData(tIC, &ic[0], RDVC, Vr, C);
      adBmsReadData(tIC, &ic[0], RDVD, Vr, D);
      adBmsReadData(tIC, &ic[0], RDRVA, Rvr, A);
      adBmsReadData(tIC, &ic[0], RDRVB, Rvr, B);
      printVoltage(tIC, &ic[0], Vr);
      printVoltage(tIC, &ic[0], Rvr);
  }

  /**
  *******************************************************************************
  * @brief Read Vrx registers
  *******************************************************************************
  */
  void adi2950_read_vrx_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDAUXC, Vrx, X);
      adBmsReadData(tIC, &ic[0], RDAUXD, Vrx, Y);
      printVoltage(tIC, &ic[0], Vrx);
  }

  /**
  *******************************************************************************
  * @brief Start Adaux Measurement
  *******************************************************************************
  */
  void adi2950_start_adaux_measurment(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    adBms2950_Adaux(AUX_CH_TO_CONVERT);
    pladc_count = adBmsPollAdc(PLADC);
    printf("Adaux conversion completed");
    printPollAdcConvTime(pladc_count);
  }

  /**
  *******************************************************************************
  * @brief Read Adaux status A measurment
  *******************************************************************************
  */
  void adi2950_read_adaux_measurment(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDSTATA, Status, A); /*!< Status A */
      printStatus(tIC, &ic[0], Status, A);
  }

  /**
  *******************************************************************************
  * @brief Read Status C Register
  *******************************************************************************
  */
  void adi2950_read_status_c_register(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDSTATC, Status, C); /*!< Status C */
      printStatus(tIC, &ic[0], Status, C);
  }

  /**
  *******************************************************************************
  * @brief Read All Status Registers
  *******************************************************************************
  */
  void adi2950_read_all_status_registers(uint8_t tIC, cell_asic *ic)
  {
      adBmsWakeupIc(tIC);
      adBmsReadData(tIC, &ic[0], RDSTATA, Status, A); /*!< Status A */
      adBmsReadData(tIC, &ic[0], RDSTATB, Status, B); /*!< Status B */
      adBmsReadData(tIC, &ic[0], RDSTATC, Status, C); /*!< Status C */
      adBmsReadData(tIC, &ic[0], RDSTATD, Status, D); /*!< Status D */
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Status E */
      printStatus(tIC, &ic[0], Status, ALL_GRP);
  }

  /**
  *******************************************************************************
  * @brief Read Device SID
  *******************************************************************************
  */
  void adi2950_read_device_sid(uint8_t tIC, cell_asic *ic)
  {
    adBmsWakeupIc(tIC);
    adBmsReadData(tIC, &ic[0], RDSID, SID, NONE);
    printDeviceSID(tIC, &ic[0], SID);
  }

  /**
  *******************************************************************************
  * @brief Soft Reset
  *******************************************************************************
  */
  void adi2950_soft_reset(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(SRST);
    printf("Soft Reset Done");
  }

  /**
  *******************************************************************************
  * @brief Reset command counter
  *******************************************************************************
  */
  void adi2950_reset_cmd_count(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(RSTCC);
    printf("Command Counter Reset Done");
  }

  /**
  *******************************************************************************
  * @brief Snapshot
  *******************************************************************************
  */
  void adi2950_snap(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(SNAP);
    printf("Snap Done");
  }

  /**
  *******************************************************************************
  * @brief Release Snapshot
  *******************************************************************************
  */
  void adi2950_unsnap(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(UNSNAP);
    printf("Unsnap Done");
  }

  /**
  *******************************************************************************
  * @brief Set and reset the gpo pins(to drive output on gpo pins)
  *******************************************************************************
  */
  void adi2950_set_reset_gpo_pins(uint8_t tIC, cell_asic *ic)
  {
    int option, gpo;
    printf("Please Enter: \n1:Set\n2:Reset\n");
    readUserInupt(&option);
    adBmsWakeupIc(tIC);
    for(uint8_t cic = 0; cic < tIC; cic++)
    {
      ic[cic].tx_cfga.refon = PWR_UP;
      if(option == 1)
      {
        printf("IC[%d]: Select GPO Pin Drive to High\n", (cic+1));
        printf("0 : GPO1 \n");
        printf("1 : GPO2 \n");
        printf("2 : GPO3 \n");
        printf("3 : GPO4 \n");
        printf("4 : GPO5 \n");
        printf("5 : GPO6 \n");
        scanf("%d",&gpo);
        ic[cic].tx_cfga.gpo = adBms2950ConfigA_Gpo((GPO)gpo, GPO_SET);    /* Gpos pins drive to high */
      }
      else if(option == 2)
      {
        printf("IC[%d]: Select GPO Pin Drive to Low\n", (cic+1));
        printf("0 : GPO1 \n");
        printf("1 : GPO2 \n");
        printf("2 : GPO3 \n");
        printf("3 : GPO4 \n");
        printf("4 : GPO5 \n");
        printf("5 : GPO6 \n");
        scanf("%d",&gpo);
        ic[cic].tx_cfga.gpo = adBms2950ConfigA_Gpo((GPO)gpo, GPO_CLR);    /* Gpo pins drive to low*/
      }
    }
    adBmsWakeupIc(tIC);
    adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
    adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read status E for gpo pin status*/
    printWriteConfig(tIC, &ic[0], Config, A);
    printReadConfig(tIC, &ic[0], Config, A);
    printStatus(tIC, &ic[0], Status, E);
  }

  /**
  *******************************************************************************
  * @brief GPIO SPI Write to Slave.
  *        Refer to the data sheet.
  *******************************************************************************
  */
  void adi2950_gpio_spi_write_to_slave(uint8_t tIC, cell_asic *ic)
  {
      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].tx_cfga.refon = PWR_UP;
        ic[cic].tx_cfga.gpio = 0xF;            /*!< All gpios pull down disable */

        ic[cic].tx_comm.icomm[0] = 0x8;        /*!< Generate a CSBM Low signal */
        ic[cic].tx_comm.fcomm[0] = 0x0;        /*!< Holds CSBM low */
        ic[cic].tx_comm.icomm[1] = 0x8;        /*!< Generate a CSBM Low signal */
        ic[cic].tx_comm.fcomm[1] = 0x9;        /*!< CSBM high */
        ic[cic].tx_comm.icomm[2] = 0xF;        /*!< No transmit */
        ic[cic].tx_comm.fcomm[2] = 0x9;        /*!< CSBM high */

        ic[cic].tx_comm.data[0] = 0x55;        /*!< data1, write the data on COMM register (D0,D1,D2) for sending the data on SPI bus*/
        ic[cic].tx_comm.data[1] = 0xAA;        /*!< data2 */
        ic[cic].tx_comm.data[2] = 0xFF;        /*!< data3 */
      }
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read Status E for gpio status*/
      adBmsWriteData(tIC, &ic[0], WRCOMM, Comm, NONE);
      adBms2950_Stcomm();
      adBmsReadData(tIC, &ic[0], RDCOMM, Comm, NONE);
      printf("GPIO SPI Write to Slave Completed");
      printWriteConfig(tIC, &ic[0], Config, A);
      printReadConfig(tIC, &ic[0], Config, A);
      printStatus(tIC, &ic[0], Status, E);
      printWriteCommData(tIC, &ic[0], Comm);
      printReadCommData(tIC, &ic[0], Comm);
  }

  /**
  *******************************************************************************
  * @brief GPIO SPI Read from Slave.
  *        Refer to the data sheet.
  *******************************************************************************
  */
  void adi2950_gpio_spi_read_from_slave(uint8_t tIC, cell_asic *ic)
  {
      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].tx_cfga.refon = PWR_UP;
        ic[cic].tx_cfga.gpio = 0xF;            /*!< All gpios pull down disable */

        ic[cic].tx_comm.icomm[0] = 0x8;        /*!< Generate a CSBM Low signal */
        ic[cic].tx_comm.fcomm[0] = 0x0;        /*!< Holds CSBM low */
        ic[cic].tx_comm.icomm[1] = 0x8;        /*!< Generate a CSBM Low signal */
        ic[cic].tx_comm.fcomm[1] = 0x9;        /*!< CSBM high */
        ic[cic].tx_comm.icomm[2] = 0xF;        /*!< No transmit */
        ic[cic].tx_comm.fcomm[2] = 0x9;        /*!< CSBM high */

        ic[cic].tx_comm.data[0] = 0x55;        /*!< data1, write the data on COMM register (D0,D1,D2) for sending the data on SPI bus*/
        ic[cic].tx_comm.data[1] = 0xAA;        /*!< data2 */
        ic[cic].tx_comm.data[2] = 0xFF;        /*!< data3 */
      }
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read Status E for gpio status*/
      adBmsWriteData(tIC, &ic[0], WRCOMM, Comm, NONE);
      adBms2950_Stcomm();
      adBmsReadData(tIC, &ic[0], RDCOMM, Comm, NONE);
      printf("GPIO SPI Read from Slave Completed");
      printWriteConfig(tIC, &ic[0], Config, A);
      printReadConfig(tIC, &ic[0], Config, A);
      printStatus(tIC, &ic[0], Status, E);
      printWriteCommData(tIC, &ic[0], Comm);
      printReadCommData(tIC, &ic[0], Comm);
  }

  /**
  *******************************************************************************
  * @brief GPIO I2C Write on the GPIO Port.
  *        Refer to the data sheet.
  *******************************************************************************
  */
  void adi2950_gpio_i2c_write_to_slave(uint8_t tIC, cell_asic *ic)
  {
      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].tx_cfga.refon = PWR_UP;
        ic[cic].tx_cfga.gpio = 0xF;            /*! All gpios pull down disable*/

        ic[cic].tx_comm.icomm[0] = 0x6;        /*!< Generate a START Signal on I2C Port Followed by Data Transmission */
        ic[cic].tx_comm.fcomm[0] = 0x8;        /*!< Master Generated an NACK Signal*/
        ic[cic].tx_comm.icomm[1] = 0x0;        /*!<! Blank, SDA Held Low Between Bytes*/
        ic[cic].tx_comm.fcomm[1] = 0x8;        /*!< Master Generated an NACK Signal*/
        ic[cic].tx_comm.icomm[2] = 0x0;        /*!< Blank, SDA Held Low Between Bytes*/
        ic[cic].tx_comm.fcomm[2] = 0x9;        /*!< Master Generated a NACK Signal, Master Generated a STOP Signal*/

        ic[cic].tx_comm.data[0] = 0xA0;        /*!< data1, write the data on COMM register (D0,D1,D2) for sending the data on I2C bus*/
        ic[cic].tx_comm.data[1] = 0x00;        /*!< data2*/
        ic[cic].tx_comm.data[2] = 0x25;        /*!< data3*/
      }
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read Status E for gpio status*/
      adBmsWriteData(tIC, &ic[0], WRCOMM, Comm, NONE);
      adBms2950_Stcomm();
      adBmsReadData(tIC, &ic[0], RDCOMM, Comm, NONE);
      printf("GPIO I2C Write data to Slave completed");
      printWriteConfig(tIC, &ic[0], Config, A);
      printReadConfig(tIC, &ic[0], Config, A);
      printStatus(tIC, &ic[0], Status, E);
      printWriteCommData(tIC, &ic[0], Comm);
      printReadCommData(tIC, &ic[0], Comm);
  }

  /**
  *******************************************************************************
  * @brief GPIO I2C Read from the GPIO Ports(using eeprom 24AA01)
  *        Refer to the data sheet.
  *******************************************************************************
  */
  void adi2950_gpio_i2c_read_from_slave(uint8_t tIC, cell_asic *ic)
  {
      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].tx_cfga.refon = PWR_UP;
        ic[cic].tx_cfga.gpio = 0xF;            /*!< All gpios pull down disable*/

        ic[cic].tx_comm.icomm[0] = 0x6;        /*!< Generate a START Signal on I2C Port Followed by Data Transmission*/
        ic[cic].tx_comm.fcomm[0] = 0x8;        /*!< Master Generated an NACK Signal*/
        ic[cic].tx_comm.icomm[1] = 0x0;        /*!< Blank, SDA Held Low Between Bytes*/
        ic[cic].tx_comm.fcomm[1] = 0x8;        /*!< Master Generated an NACK Signal*/
        ic[cic].tx_comm.icomm[2] = 0x6;        /*!< Blank, SDA Held Low Between Bytes*/
        ic[cic].tx_comm.fcomm[2] = 0x8;        /*!< Master Generated a NACK Signal, Master Generated a STOP Signal*/

        ic[cic].tx_comm.data[0] = 0xA0;        /*!<data1*/
        ic[cic].tx_comm.data[1] = 0x00;        /*!<data2*/
        ic[cic].tx_comm.data[2] = 0xA1;        /*!<data3*/
      }
      /***************************** Write to slave devcie *****************/
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read Status E for gpio status*/
      adBmsWriteData(tIC, &ic[0], WRCOMM, Comm, NONE);
      adBms2950_Stcomm();
      printWriteConfig(tIC, &ic[0], Config, A);
      printReadConfig(tIC, &ic[0], Config, A);
      printWriteCommData(tIC, &ic[0], Comm);
      printStatus(tIC, &ic[0], Status, E);

      for(uint8_t cic = 0; cic < tIC; cic++)
      {
        ic[cic].tx_cfga.refon = PWR_UP;
        ic[cic].tx_cfga.gpio = 0xF;            /*!< All gpios pull down disable */

        ic[cic].tx_comm.icomm[0] = 0x0;        /*!< SDA held low */
        ic[cic].tx_comm.fcomm[0] = 0x9;        /*!<Slave NACk+ master STOP */
        ic[cic].tx_comm.icomm[1] = 0x7;        /*!< SDA held high */
        ic[cic].tx_comm.fcomm[1] = 0x9;        /*!< Slave NACk+ master STOP */
        ic[cic].tx_comm.icomm[2] = 0x7;        /*!<SDA held high */
        ic[cic].tx_comm.fcomm[2] = 0x9;        /*!< Slave NACk+ master STOP */

        ic[cic].tx_comm.data[0] = 0xFF;        /*!<data1 */
        ic[cic].tx_comm.data[1] = 0xFF;        /*!<data2 */
        ic[cic].tx_comm.data[2] = 0xFF;        /*!<data3 */
      }
      /***************************** Read from slave devcie *****************/
      adBmsWakeupIc(tIC);
      adBmsWriteData(tIC, &ic[0], WRCFGA, Config, A);
      adBmsReadData(tIC, &ic[0], RDSTATE, Status, E); /*!< Read Status E for gpio status*/
      adBms2950_Stcomm();
      adBmsReadData(tIC, &ic[0], RDCOMM, Comm, NONE);
      printf("GPIO I2C Read data from Slave completed");
      printReadCommData(tIC, &ic[0], Comm);
      printStatus(tIC, &ic[0], Status, E);
  }

  /**
  *******************************************************************************
  * @brief Soft Reset
  *******************************************************************************
  */
  void soft_reset(uint8_t tIC)
  {
    adBmsWakeupIc(tIC);
    spiSendCmd(SRST);
    Delay_ms(8); /*  After SRST cmd 8ms delay required*/
  }
}
