



void adbms_main()
{
  printMenu();
  adBms6830_init_config(TOTAL_IC, &IC[0]);
  while(1)
  {
    int user_command;
#ifdef MBED
    pc.scanf("%d", &user_command);
    pc.printf("Enter cmd:%d\n", user_command);
#else
    scanf("%d", &user_command);
    printf("Enter cmd:%d\n", user_command);
#endif
    run_command(user_command);
  }
}

void run_command(int cmd)
{
  switch(cmd)
  {

  case 1:
    adBms6830_write_read_config(TOTAL_IC, &IC[0]);
    break;

  case 2:
    adBms6830_read_config(TOTAL_IC, &IC[0]);
    break;

  case 3:
    adBms6830_start_adc_cell_voltage_measurment(TOTAL_IC);
    break;

  case 4:
    adBms6830_read_cell_voltages(TOTAL_IC, &IC[0]);
    break;

  case 5:
    adBms6830_start_adc_s_voltage_measurment(TOTAL_IC);
    break;

  case 6:
    adBms6830_read_s_voltages(TOTAL_IC, &IC[0]);
    break;

  case 7:
    adBms6830_start_avgcell_voltage_measurment(TOTAL_IC);
    break;

  case 8:
    adBms6830_read_avgcell_voltages(TOTAL_IC, &IC[0]);
    break;

  case 9:
    adBms6830_start_fcell_voltage_measurment(TOTAL_IC);
    break;

  case 10:
    adBms6830_read_fcell_voltages(TOTAL_IC, &IC[0]);
    break;

  case 11:
    adBms6830_start_aux_voltage_measurment(TOTAL_IC, &IC[0]);
    break;

  case 12:
    adBms6830_read_aux_voltages(TOTAL_IC, &IC[0]);
    break;

  case 13:
    adBms6830_start_raux_voltage_measurment(TOTAL_IC, &IC[0]);
    break;

  case 14:
    adBms6830_read_raux_voltages(TOTAL_IC, &IC[0]);
    break;

  case 15:
    adBms6830_read_status_registers(TOTAL_IC, &IC[0]);
    break;

  case 16:
    loop_count = 0;
    adBmsWakeupIc(TOTAL_IC);
    adBmsWriteData(TOTAL_IC, &IC[0], WRCFGA, Config, A);
    adBmsWriteData(TOTAL_IC, &IC[0], WRCFGB, Config, B);
    adBmsWakeupIc(TOTAL_IC);
    adBms6830_Adcv(REDUNDANT_MEASUREMENT, CONTINUOUS, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    Delay_ms(1); // ADCs are updated at their conversion rate is 1ms
    adBms6830_Adcv(RD_ON, CONTINUOUS, DISCHARGE_PERMITTED, RESET_FILTER, CELL_OPEN_WIRE_DETECTION);
    Delay_ms(1); // ADCs are updated at their conversion rate is 1ms
    adBms6830_Adsv(CONTINUOUS, DISCHARGE_PERMITTED, CELL_OPEN_WIRE_DETECTION);
    Delay_ms(8); // ADCs are updated at their conversion rate is 8ms
    while(loop_count < LOOP_MEASUREMENT_COUNT)
    {
      measurement_loop();
      Delay_ms(MEASUREMENT_LOOP_TIME);
      loop_count = loop_count + 1;
    }
    printMenu();
    break;

  case 17:
    adBms6830_clear_cell_measurement(TOTAL_IC);
    break;

  case 18:
    adBms6830_clear_aux_measurement(TOTAL_IC);
    break;

  case 19:
    adBms6830_clear_spin_measurement(TOTAL_IC);
    break;

  case 20:
    adBms6830_clear_fcell_measurement(TOTAL_IC);
    break;

  case 21:
    adBms6830_write_config(TOTAL_IC, &IC[0]);
    break;

  case 0:
    printMenu();
    break;

  default:
#ifdef MBED
    pc.printf("Incorrect Option\n\n");
#else
    printf("Incorrect Option\n\n");
#endif
    break;
  }
}



/**
*******************************************************************************
* @brief Loop measurment.
*******************************************************************************
*/
void measurement_loop()
{
  if(MEASURE_CELL == ENABLED)
  {
    adBmsReadData(TOTAL_IC, &IC[0], RDCVA, Cell, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDCVB, Cell, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDCVC, Cell, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDCVD, Cell, D);
    adBmsReadData(TOTAL_IC, &IC[0], RDCVE, Cell, E);
    adBmsReadData(TOTAL_IC, &IC[0], RDCVF, Cell, F);
    printVoltages(TOTAL_IC, &IC[0], Cell);
  }

  if(MEASURE_AVG_CELL == ENABLED)
  {
    adBmsReadData(TOTAL_IC, &IC[0], RDACA, AvgCell, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDACB, AvgCell, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDACC, AvgCell, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDACD, AvgCell, D);
    adBmsReadData(TOTAL_IC, &IC[0], RDACE, AvgCell, E);
    adBmsReadData(TOTAL_IC, &IC[0], RDACF, AvgCell, F);
    printVoltages(TOTAL_IC, &IC[0], AvgCell);
  }

  if(MEASURE_F_CELL == ENABLED)
  {
    adBmsReadData(TOTAL_IC, &IC[0], RDFCA, F_volt, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDFCB, F_volt, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDFCC, F_volt, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDFCD, F_volt, D);
    adBmsReadData(TOTAL_IC, &IC[0], RDFCE, F_volt, E);
    adBmsReadData(TOTAL_IC, &IC[0], RDFCF, F_volt, F);
    printVoltages(TOTAL_IC, &IC[0], F_volt);
  }

  if(MEASURE_S_VOLTAGE == ENABLED)
  {
    adBmsReadData(TOTAL_IC, &IC[0], RDSVA, S_volt, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDSVB, S_volt, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDSVC, S_volt, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDSVD, S_volt, D);
    adBmsReadData(TOTAL_IC, &IC[0], RDSVE, S_volt, E);
    adBmsReadData(TOTAL_IC, &IC[0], RDSVF, S_volt, F);
    printVoltages(TOTAL_IC, &IC[0], S_volt);
  }

  if(MEASURE_AUX == ENABLED)
  {
    adBms6830_Adax(AUX_OPEN_WIRE_DETECTION, OPEN_WIRE_CURRENT_SOURCE, AUX_CH_TO_CONVERT);
    adBmsPollAdc(PLAUX1);
    adBmsReadData(TOTAL_IC, &IC[0], RDAUXA, Aux, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDAUXB, Aux, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDAUXC, Aux, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDAUXD, Aux, D);
    printVoltages(TOTAL_IC, &IC[0], Aux);
  }

  if(MEASURE_RAUX == ENABLED)
  {
    adBmsWakeupIc(TOTAL_IC);
    adBms6830_Adax2(AUX_CH_TO_CONVERT);
    adBmsPollAdc(PLAUX2);
    adBmsReadData(TOTAL_IC, &IC[0], RDRAXA, RAux, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDRAXB, RAux, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDRAXC, RAux, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDRAXD, RAux, D);
    printVoltages(TOTAL_IC, &IC[0], RAux);
  }

  if(MEASURE_STAT == ENABLED)
  {
    adBmsReadData(TOTAL_IC, &IC[0], RDSTATA, Status, A);
    adBmsReadData(TOTAL_IC, &IC[0], RDSTATB, Status, B);
    adBmsReadData(TOTAL_IC, &IC[0], RDSTATC, Status, C);
    adBmsReadData(TOTAL_IC, &IC[0], RDSTATD, Status, D);
    adBmsReadData(TOTAL_IC, &IC[0], RDSTATE, Status, E);
    printStatus(TOTAL_IC, &IC[0], Status, ALL_GRP);
  }
}


/// 2950 /////////////////////////////////////////////////////////////////////////////////


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

void run_command(int cmd) /// same name
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





