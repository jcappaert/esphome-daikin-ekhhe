#pragma once

#include <string>

namespace esphome {
namespace daikin_ekhhe {

// Define names as static constants
// this should match what is in const.py on the Python side
static const std::string A_LOW_WAT_T_PROBE       = "low_water_temp_probe";
static const std::string B_UP_WAT_T_PROBE        = "upper_water_temp_probe";
static const std::string C_DEFROST_T_PROBE       = "defrost_temp_probe";
static const std::string D_SUPPLY_AIR_T_PROBE    = "supply_air_temp_probe";
static const std::string E_EVA_INLET_T_PROBE     = "evaporator_inlet_gas_temp_probe";
static const std::string F_EVA_OUTLET_T_PROBE    = "evaporator_outlet_gas_temp_probe";
static const std::string G_COMP_GAS_T_PROBE      = "compressor_discharge_gas_temp_probe";
static const std::string H_SOLAR_T_PROBE         = "solar_collector_temp_probe";
static const std::string I_EEV_STEP              = "eev_opening_step";
static const std::string J_POWER_FW_VERSION      = "power_board_firmware_version";
static const std::string L_UI_FW_VERSION         = "ui_firmware_version";

static const std::string DIG1_CONFIG             = "dig1_config";
static const std::string DIG2_CONFIG             = "dig2_config";
static const std::string DIG3_CONFIG             = "dig3_config";

static const std::string POWER_STATUS            = "power_status";
static const std::string OPERATIONAL_MODE        = "operational_mode";
static const std::string CURRENT_TIME            = "current_time";

static const std::string AUTO_T_TEMPERATURE      = "auto_target_temperature";
static const std::string ECO_T_TEMPERATURE       = "eco_target_temperature";
static const std::string BOOST_T_TEMPERATURE     = "boost_target_temperature";
static const std::string ELECTRIC_T_TEMPERATURE  = "electric_target_temperature";
static const std::string VAC_DAYS                = "vacation_days";

static const std::string P1_LOW_WAT_PROBE_HYST   = "low_water_hp_hysteris";
static const std::string P2_HEAT_ON_DELAY        = "elec_heater_switch_on_delay";
static const std::string P3_ANTL_SET_T           = "antilegionella_setpoint_temp";
static const std::string P4_ANTL_DURATION        = "antilegionella_duration";
static const std::string P5_DEFROST_MODE         = "defrosting_mode";
static const std::string P6_EHEATER_DEFROSTING   = "elec_heater_during_defrosting";
static const std::string P7_DEFROST_CYCLE_DELAY  = "defrosting_cycle_delay";
static const std::string P8_DEFR_START_THRES     = "defrosting_start_temp_threshold";
static const std::string P9_DEFR_STOP_THRES      = "defrosting_stop_temp_threshold";
static const std::string P10_DEFR_MAX_DURATION   = "max_defrosting_duration";

static const std::string P11_DISP_WAT_T_PROBE    = "display_water_probe_temp";
static const std::string P12_EXT_PUMP_MODE       = "external_pump_mode";
static const std::string P13_HW_CIRC_PUMP_MODE   = "hw_circ_pump_mode";
static const std::string P14_EVA_BLOWER_TYPE     = "evaporator_blower_type";
static const std::string P15_SAFETY_SW_TYPE      = "safety_flow_switch_type";
static const std::string P16_SOLAR_MODE_INT      = "solar_mode_integration";
static const std::string P17_HP_START_DELAY_DIG1 = "dig1_hp_start_delay";
static const std::string P18_LOW_WAT_T_DIG1      = "dig1_low_water_temp_threshold";
static const std::string P19_LOW_WAT_T_HYST      = "low_wat_probe_solar_hysteresis";
static const std::string P20_SOL_DRAIN_THRES     = "solar_drain_temp_threshold";

static const std::string P21_LOW_WAT_T_HP_STOP   = "low_water_hp_pv_mode_threshold";
static const std::string P22_UP_WAT_T_EH_STOP    = "upper_water_eh_pv_mode_threshold";
static const std::string P23_PV_MODE_INT         = "pv_mode_integration";
static const std::string P24_OFF_PEAK_MODE       = "off_peak_working_mode";
static const std::string P25_UP_WAT_T_OFFSET     = "upper_water_temp_probe_offset";
static const std::string P26_LOW_WAT_T_OFFSET    = "lower_water_temp_probe_offset";
static const std::string P27_INLET_T_OFFSET      = "air_inlet_temp_probe_offset";
static const std::string P28_DEFR_T_OFFSET       = "defrost_temp_probe_offset";
static const std::string P29_ANTL_START_HR       = "antilegionella_start_hour";
static const std::string P30_UP_WAT_T_EH_HYST    = "upper_water_hysteresis_eh";

static const std::string P31_HP_PERIOD_AUTO      = "hp_period_auto_mode_calc";
static const std::string P32_EH_AUTO_TRES        = "eh_auto_mode_temp_threshold";
static const std::string P33_EEV_CONTROL         = "eev_control";
static const std::string P34_EEV_SH_PERIOD       = "eev_superheating_calc_period";
static const std::string P35_EEV_SH_SETPOINT     = "eev_control_superheating_setpoint";
static const std::string P36_EEV_DSH_SETPOINT    = "eev_control_desuperheating_setpoint";
static const std::string P37_EEV_STEP_DEFR       = "defrosting_mode_eev_step";
static const std::string P38_EEV_MIN_STEP_AUTO   = "auto_mode_min_eev_step";
static const std::string P39_EEV_MODE            = "eev_control_mode";
static const std::string P40_EEV_INIT_STEP       = "eev_step_opening_initial";

static const std::string P41_AKP1_THRES          = "eev_kp1_gain_akp1_temp_threshold";
static const std::string P42_AKP2_THRES          = "eev_kp2_gain_akp2_temp_threshold";
static const std::string P43_AKP3_THRES          = "eev_kp3_gain_akp3_temp_threshold";
static const std::string P44_EEV_KP1_GAIN        = "eev_kp1_gain";
static const std::string P45_EEV_KP2_GAIN        = "eev_kp2_gain";
static const std::string P46_EEV_KP3_GAIN        = "eev_kp3_gain";
static const std::string P47_MAX_INLET_T_HP      = "max_inlet_temp_hp_working";
static const std::string P48_MIN_INLET_T_HP      = "min_inlet_temp_hp_working";
static const std::string P49_EVA_INLET_THRES     = "evaporator_inlet_temp_threshold_blower";
static const std::string P50_ANTIFREEZE_SET      = "antifreeze_low_water_temp_setpoint";

static const std::string P51_EVA_HIGH_SET        = "evaporator_blower_high_speed_set";
static const std::string P52_EVA_LOW_SET         = "evaporator_blower_low_speed_set";

}  // namespace daikin_ekhhe
}  // namespace esphome
