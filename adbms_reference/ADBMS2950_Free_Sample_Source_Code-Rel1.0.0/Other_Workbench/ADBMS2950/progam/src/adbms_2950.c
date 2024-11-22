/**
********************************************************************************
*
* @file:    adbms_2950.c
*
* @brief:   This file contains the test cases implementation.
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

/*! \addtogroup Test_Cases
*  @{
*/
#include "adbms2950.h"
#include "application.h"
#ifdef MBED
extern Serial pc;
#endif /* MBED */
/*******************************************************************************
* @brief Setup Variables
* The following variables can be modified to configure the software.
*******************************************************************************/
/*!< ********************************GLOBAL VARIABLES****************************/
/*!< ADC Command Configurations */
VCH   VOLTAGE_MEASUREMENT     = SM_V1;
RD    REDUNDANT_MEASUREMENT   = RD_OFF;
ACH   AUX_CH_TO_CONVERT       = ALL;
CONT  CONTINUOUS_MEASUREMENT  = SINGLE;
OW    OW_WIRE_DETECTION       = OW_OFF;
ERR   INJECT_ERR_SPI_READ     = WITHOUT_ERR;
uint32_t pladc_count;
/*!< ****************************************************************************/

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
  printMsg("Adi1 conversion completed");
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
  printMsg("Adi1 conversion completed");
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
  printMsg("Adi2 conversion completed");
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
  printMsg("Adi2 conversion completed");
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
  printMsg("Adi1 redundant conversion completed");
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
  printMsg("Adi1 redundant conversion completed");
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
  printMsg("Adv conversion completed");
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
  printMsg("Adaux conversion completed");
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
  printMsg("Soft Reset Done");
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
  printMsg("Command Counter Reset Done");
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
  printMsg("Snap Done");
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
  printMsg("Unsnap Done");
}

/**
*******************************************************************************
* @brief Set and reset the gpo pins(to drive output on gpo pins)
*******************************************************************************
*/
void adi2950_set_reset_gpo_pins(uint8_t tIC, cell_asic *ic)
{
  int option, gpo;
  printMsg("Please Enter: \n1:Set\n2:Reset\n");
  readUserInupt(&option);
  adBmsWakeupIc(tIC);
  for(uint8_t cic = 0; cic < tIC; cic++)
  {
    ic[cic].tx_cfga.refon = PWR_UP;
    if(option == 1)
    {
#ifdef MBED
      pc.printf("IC[%d]: Select GPO Pin Drive to High\n", (cic+1));
      pc.printf("0 : GPO1 \n");
      pc.printf("1 : GPO2 \n");
      pc.printf("2 : GPO3 \n");
      pc.printf("3 : GPO4 \n");
      pc.printf("4 : GPO5 \n");
      pc.printf("5 : GPO6 \n");
      pc.scanf("%d",&gpo);
#else
      printf("IC[%d]: Select GPO Pin Drive to High\n", (cic+1));
      printf("0 : GPO1 \n");
      printf("1 : GPO2 \n");
      printf("2 : GPO3 \n");
      printf("3 : GPO4 \n");
      printf("4 : GPO5 \n");
      printf("5 : GPO6 \n");
      scanf("%d",&gpo);
#endif
      ic[cic].tx_cfga.gpo = adBms2950ConfigA_Gpo((GPO)gpo, GPO_SET);    /* Gpos pins drive to high */
    }
    else if(option == 2)
    {
#ifdef MBED
      pc.printf("IC[%d]: Select GPO Pin Drive to Low\n", (cic+1));
      pc.printf("0 : GPO1 \n");
      pc.printf("1 : GPO2 \n");
      pc.printf("2 : GPO3 \n");
      pc.printf("3 : GPO4 \n");
      pc.printf("4 : GPO5 \n");
      pc.printf("5 : GPO6 \n");
      scanf("%d",&gpo);
#else
      printf("IC[%d]: Select GPO Pin Drive to Low\n", (cic+1));
      printf("0 : GPO1 \n");
      printf("1 : GPO2 \n");
      printf("2 : GPO3 \n");
      printf("3 : GPO4 \n");
      printf("4 : GPO5 \n");
      printf("5 : GPO6 \n");
      scanf("%d",&gpo);
#endif
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
    printMsg("GPIO SPI Write to Slave Completed");
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
    printMsg("GPIO SPI Read from Slave Completed");
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
    printMsg("GPIO I2C Write data to Slave completed");
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
    printMsg("GPIO I2C Read data from Slave completed");
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

/** @}*/
/** @}*/