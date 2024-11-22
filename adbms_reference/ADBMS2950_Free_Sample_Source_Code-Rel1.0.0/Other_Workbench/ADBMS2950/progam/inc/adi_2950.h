/**
********************************************************************************
*
* @file:    adi_2950.h
*
* @brief:   This file contains the test cases functions.
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

#ifndef __ADI2950_H
#define __ADI2950_H

#include "application.h"
void adi2950_init_config(uint8_t tIC, cell_asic *ic);
void adi2950_write_read_config(uint8_t tIC, cell_asic *ic);
void adi2950_read_config(uint8_t tIC, cell_asic *ic);
void adi2950_start_adi1_single_measurment(uint8_t tIC);
void adi2950_start_adi1_continuous_measurment(uint8_t tIC);
void adi2950_start_adi2_single_measurment(uint8_t tIC);
void adi2950_start_adi2_continuous_measurment(uint8_t tIC);
void adi2950_start_adi1_redundant_single_measurment(uint8_t tIC);
void adi2950_start_adi1_redundant_continuous_measurment(uint8_t tIC);
void adi2950_read_cr_vbat_ivbat_registers(uint8_t tIC, cell_asic *ic);
void adi2950_read_ocr_register(uint8_t tIC, cell_asic *ic);
void adi2950_read_avgcr_avgvbat_avgivbat_registers(uint8_t tIC, cell_asic *ic);
void adi2950_all_current_battery_voltage_registers(uint8_t tIC, cell_asic *ic);
void adi2950_start_adv_measurment(uint8_t tIC);
void adi2950_read_vr_registers(uint8_t tIC, cell_asic *ic);
void adi2950_read_rvr_registers(uint8_t tIC, cell_asic *ic);
void adi2950_read_vr_rvr_registers(uint8_t tIC, cell_asic *ic);
void adi2950_read_vrx_registers(uint8_t tIC, cell_asic *ic);
void adi2950_start_adaux_measurment(uint8_t tIC);
void adi2950_read_adaux_measurment(uint8_t tIC, cell_asic *ic);
void adi2950_read_status_c_register(uint8_t tIC, cell_asic *ic);
void adi2950_read_all_status_registers(uint8_t tIC, cell_asic *ic);
void adi2950_read_device_sid(uint8_t tIC, cell_asic *ic);
void adi2950_soft_reset(uint8_t tIC);
void adi2950_reset_cmd_count(uint8_t tIC);
void adi2950_snap(uint8_t tIC);
void adi2950_unsnap(uint8_t tIC);
void adi2950_set_reset_gpo_pins(uint8_t tIC, cell_asic *ic);
void adi2950_gpio_spi_write_to_slave(uint8_t tIC, cell_asic *ic);
void adi2950_gpio_spi_read_from_slave(uint8_t tIC, cell_asic *ic);
void adi2950_gpio_i2c_write_to_slave(uint8_t tIC, cell_asic *ic);
void adi2950_gpio_i2c_read_from_slave(uint8_t tIC, cell_asic *ic);
void soft_reset(uint8_t tIC);

#endif /* __ADI2950_H */

/** @}*/
/** @}*/