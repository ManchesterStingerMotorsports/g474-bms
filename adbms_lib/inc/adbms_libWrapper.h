
// SOURCE: adBms_Application.c (6830) + adi_2950/adbms_2950 (2950)

#pragma once

#include "common.h"

#include "adbms2950_data.h"
#include "adbms6830_data.h"


namespace AD68_NS
{
  void adBms6830_init_config(uint8_t tIC, cell_asic *ic);
  void adBms6830_write_read_config(uint8_t tIC, cell_asic *ic);
  void adBms6830_write_config(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_config(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_adc_cell_voltage_measurment(uint8_t tIC);
  void adBms6830_read_cell_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_adc_s_voltage_measurment(uint8_t tIC);
  void adBms6830_read_s_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_avgcell_voltage_measurment(uint8_t tIC);
  void adBms6830_read_avgcell_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_fcell_voltage_measurment(uint8_t tIC);
  void adBms6830_read_fcell_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_aux_voltage_measurment(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_aux_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_start_raux_voltage_measurment(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_raux_voltages(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_status_registers(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_device_sid(uint8_t tIC, cell_asic *ic);
  void adBms6830_set_reset_gpio_pins(uint8_t tIC, cell_asic *ic);
  void adBms6830_enable_mute(uint8_t tIC, cell_asic *ic);
  void adBms6830_disable_mute(uint8_t tIC, cell_asic *ic);
  void adBms6830_soft_reset(uint8_t tIC);
  void adBms6830_reset_cmd_count(uint8_t tIC);
  void adBms6830_reset_pec_error_flag(uint8_t tIC, cell_asic *ic);
  void adBms6830_snap(uint8_t tIC);
  void adBms6830_unsnap(uint8_t tIC);
  void adBms6830_clear_cell_measurement(uint8_t tIC);
  void adBms6830_clear_aux_measurement(uint8_t tIC);
  void adBms6830_clear_spin_measurement(uint8_t tIC);
  void adBms6830_clear_fcell_measurement(uint8_t tIC);
  void adBms6830_clear_ovuv_measurement(uint8_t tIC);
  void adBms6830_clear_all_flags(uint8_t tIC, cell_asic *ic);
  void adBms6830_set_dcc_discharge(uint8_t tIC, cell_asic *ic);
  void adBms6830_clear_dcc_discharge(uint8_t tIC, cell_asic *ic);
  void adBms6830_write_read_pwm_duty_cycle(uint8_t tIC, cell_asic *ic);
  void adBms6830_gpio_spi_communication(uint8_t tIC, cell_asic *ic);
  void adBms6830_gpio_i2c_write_to_slave(uint8_t tIC, cell_asic *ic);
  void adBms6830_gpio_i2c_read_from_slave(uint8_t tIC, cell_asic *ic);
  void adBms6830_set_dtrng_dcto_value(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_osc_mismatch_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_thermal_shutdown_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_supply_error_detection_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_thermal_shutdown_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_fuse_ed_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_fuse_med_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_run_tmodchk_self_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_check_latent_fault_csflt_status_bits(uint8_t tIC, cell_asic *ic);
  void adBms6830_check_rdstatc_err_bit_functionality(uint8_t tIC, cell_asic *ic);
  void adBms6830_cell_openwire_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_redundant_cell_openwire_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_cell_ow_volatge_collect(uint8_t tIC, cell_asic *ic, TYPE type, OW_C_S ow_c_s);
  void adBms6830_aux_openwire_test(uint8_t tIC, cell_asic *ic);
  void adBms6830_gpio_pup_up_down_volatge_collect(uint8_t tIC, cell_asic *ic, PUP pup);
  void adBms6830_open_wire_detection_condtion_check(uint8_t tIC, cell_asic *ic, TYPE type);
  void adBms6830_read_rdcvall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdacall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdsall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdfcall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdcsall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdacsall_voltage(uint8_t tIC, cell_asic *ic);
  void adBms6830_read_rdasall_voltage(uint8_t tIC, cell_asic *ic);
}

namespace AD29_NS
{
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
}


/** @}*/
/** @}*/
