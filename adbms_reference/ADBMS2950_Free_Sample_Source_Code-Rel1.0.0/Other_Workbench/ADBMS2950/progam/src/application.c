/**
********************************************************************************
*
* @file:    application.c
*
* @brief:   This file contains the application test cases.
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

/*! \addtogroup Application
*  @{
*/
/*============= I N C L U D E S =============*/
/*============== D E F I N E S ===============*/
/*============= E X T E R N A L S ============*/
/*============= E N U M E R A T O R S ============*/

#include "application.h"
#ifdef MBED
extern Serial pc;
#endif /* MBED */
/**
*******************************************************************************
* @brief Setup Variables
* The following variables can be modified to configure the software.
*******************************************************************************
*/

#define TOTAL_IC 1
cell_asic IC[TOTAL_IC];
int loop_measurment_count = 10;      /* Loop measurment count (default count)*/
int loop_measurment_time = 1;        /* milliseconds(mS)*/
int loop_count = 0;

void app_main()
{
  printMenu();
  while(1)
  {
    int user_command;
    readUserInupt(&user_command);
    adi2950_init_config(TOTAL_IC, &IC[0]);
    run_command(user_command); /*!< Run test case */
  }
}

void run_command(int cmd)
{
  switch(cmd)
  {
  case 1:
    adi2950_write_read_config(TOTAL_IC, &IC[0]);
    break;

  case 2:
    adi2950_read_config(TOTAL_IC, &IC[0]);
    break;

  case 3:
    adi2950_start_adi1_single_measurment(TOTAL_IC);
    break;

  case 4:
    adi2950_start_adi2_single_measurment(TOTAL_IC);
    break;

  case 5:
    adi2950_start_adi1_continuous_measurment(TOTAL_IC);
    break;

  case 6:
    adi2950_start_adi2_continuous_measurment(TOTAL_IC);
    break;

  case 7:
    adi2950_start_adi1_redundant_single_measurment(TOTAL_IC);
    break;

  case 8:
    adi2950_start_adi1_redundant_continuous_measurment(TOTAL_IC);
    break;

  case 9:
    adi2950_read_cr_vbat_ivbat_registers(TOTAL_IC, &IC[0]);
    break;

  case 10:
    loop_count = 0;
    printMsg("Please enter loop measurment count:");
    readUserInupt(&loop_measurment_count);
    while(loop_count < loop_measurment_count)
    {
      printResultCount(loop_count);
      adi2950_read_cr_vbat_ivbat_registers(TOTAL_IC, &IC[0]);
      Delay_ms(loop_measurment_time);
      loop_count = loop_count + 1;
    }
    break;

  case 11:
    adi2950_read_ocr_register(TOTAL_IC, &IC[0]);
    break;

  case 12:
    loop_count = 0;
    printMsg("Please enter loop measurment count:");
    readUserInupt(&loop_measurment_count);
    while(loop_count < loop_measurment_count)
    {
      printResultCount(loop_count);
      adi2950_read_ocr_register(TOTAL_IC, &IC[0]);
      Delay_ms(loop_measurment_time);
      loop_count = loop_count + 1;
    }
    break;

  case 13:
    adi2950_read_avgcr_avgvbat_avgivbat_registers(TOTAL_IC, &IC[0]);
    break;

  case 14:
    loop_count = 0;
    printMsg("Please enter loop measurment count:");
    readUserInupt(&loop_measurment_count);
    while(loop_count < loop_measurment_count)
    {
      printResultCount(loop_count);
      adi2950_read_avgcr_avgvbat_avgivbat_registers(TOTAL_IC, &IC[0]);
      Delay_ms(loop_measurment_time);
      loop_count = loop_count + 1;
    }
    break;

  case 15:
    adi2950_all_current_battery_voltage_registers(TOTAL_IC, &IC[0]);
    break;

  case 16:
    printMsg("Please enter loop measurment count:");
    readUserInupt(&loop_measurment_count);
    loop_count = 0;
    while(loop_count < loop_measurment_count)
    {
      printResultCount(loop_count);
      adi2950_all_current_battery_voltage_registers(TOTAL_IC, &IC[0]);
      Delay_ms(loop_measurment_time);
      loop_count = loop_count + 1;
    }
    break;

  case 17:
   adi2950_start_adv_measurment(TOTAL_IC);
    break;

  case 18:
    adi2950_read_vr_registers(TOTAL_IC, &IC[0]);
    break;

  case 19:
    adi2950_read_rvr_registers(TOTAL_IC, &IC[0]);
    break;

  case 20:
    adi2950_read_vr_rvr_registers(TOTAL_IC, &IC[0]);
    break;

   case 21:
    adi2950_read_vrx_registers(TOTAL_IC, &IC[0]);
    break;

   case 22:
    adi2950_start_adaux_measurment(TOTAL_IC);
    break;

   case 23:
    adi2950_read_adaux_measurment(TOTAL_IC, &IC[0]);
    break;

   case 24:
    adi2950_read_status_c_register(TOTAL_IC, &IC[0]);
    break;

   case 25:
    adi2950_read_all_status_registers(TOTAL_IC, &IC[0]);
    break;

   case 26:
    adi2950_read_device_sid(TOTAL_IC, &IC[0]);
    break;

   case 27:
    adi2950_soft_reset(TOTAL_IC);
    break;

   case 28:
    adi2950_reset_cmd_count(TOTAL_IC);
    break;

   case 29:
    adi2950_snap(TOTAL_IC);
    break;

   case 30:
    adi2950_unsnap(TOTAL_IC);
    break;

   case 31:
    adi2950_set_reset_gpo_pins(TOTAL_IC, &IC[0]);
    break;

   case 32:
    adi2950_gpio_spi_write_to_slave(TOTAL_IC, &IC[0]);
    break;

   case 33:
    adi2950_gpio_spi_read_from_slave(TOTAL_IC, &IC[0]);
    break;

   case 34:
    adi2950_gpio_i2c_write_to_slave(TOTAL_IC, &IC[0]);
    break;

   case 35:
    adi2950_gpio_i2c_read_from_slave(TOTAL_IC, &IC[0]);
    break;

  case 0:
    printMenu();
    break;

  default:
    printMsg("Incorrect Option");
    break;
  }
}
/** @}*/
/** @}*/