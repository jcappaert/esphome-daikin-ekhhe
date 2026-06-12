#include "daikin_ekhhe.h"
#include "daikin_ekhhe_metadata.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"

#include <map>
#include <string>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

void DaikinEkhheComponent::update_dd_b1_bit_sensors_() {
  if (heating_demand_ == nullptr && hp_active_ == nullptr && eh_active_ == nullptr) {
    return;
  }
  size_t dd_index = 0;
  const RawFrameEntry *dd_entry = find_latest_dd_frame_(dd_index);
  if (dd_entry == nullptr) {
    return;
  }
  uint8_t b1 = dd_entry->data[1];
  bool demand = (b1 & 0x40) != 0;
  bool hp_active = (b1 & 0x01) != 0;
  bool eh_active = (b1 & 0x02) != 0;

  if (!have_last_dd_bits_ || demand != last_dd_demand_) {
    if (heating_demand_ != nullptr) {
      heating_demand_->publish_state(demand);
    }
  }
  if (!have_last_dd_bits_ || hp_active != last_hp_active_) {
    if (hp_active_ != nullptr) {
      hp_active_->publish_state(hp_active);
    }
  }
  if (!have_last_dd_bits_ || eh_active != last_eh_active_) {
    if (eh_active_ != nullptr) {
      eh_active_->publish_state(eh_active);
    }
  }

  last_dd_demand_ = demand;
  last_hp_active_ = hp_active;
  last_eh_active_ = eh_active;
  have_last_dd_bits_ = true;
}

void DaikinEkhheComponent::parse_dd_packet(std::vector<uint8_t> buffer) {
  const float lower_water_temperature = static_cast<int8_t>(buffer[DD_PACKET_A_IDX]);
  const float upper_water_temperature = static_cast<int8_t>(buffer[DD_PACKET_B_IDX]);

#if defined(USE_WATER_HEATER)
  update_water_heater_temperature_cache_(lower_water_temperature, upper_water_temperature);
#endif

  // update sensors
  std::map<std::string, float> sensor_values = {
      {A_LOW_WAT_T_PROBE,      lower_water_temperature},
      {B_UP_WAT_T_PROBE,       upper_water_temperature},
      {C_DEFROST_T_PROBE,      (int8_t)buffer[DD_PACKET_C_IDX]},
      {D_SUPPLY_AIR_T_PROBE,   (int8_t)buffer[DD_PACKET_D_IDX]},
      {E_EVA_INLET_T_PROBE,    (int8_t)buffer[DD_PACKET_E_IDX]},
      {F_EVA_OUTLET_T_PROBE,   (int8_t)buffer[DD_PACKET_F_IDX]},
      {G_COMP_GAS_T_PROBE,     buffer[DD_PACKET_G_IDX]},
      {H_SOLAR_T_PROBE,        buffer[DD_PACKET_H_IDX]},
      {I_EEV_STEP,             (float)((buffer[DD_PACKET_I_IDX] << 8) | buffer[DD_PACKET_I_IDX + 1])},
      {FAN_SPEED_RPM,          (float)((buffer[DD_PACKET_FAN_RPM_IDX] << 8) | buffer[DD_PACKET_FAN_RPM_IDX + 1])},
  };

  for (const auto &entry : sensor_values) {
    set_sensor_value(entry.first, entry.second);
  }

  set_text_sensor_value_(J_POWER_FW_VERSION, "U" + std::to_string(buffer[DD_PACKET_J_FW_IDX]));

  // update binary_sensors
  const bool p01_tank_lower_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x04) != 0;
  const bool p02_tank_upper_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x02) != 0;
  const bool p03_defrost_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x01) != 0;
  const bool p04_inlet_air_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x08) != 0;
  const bool p05_evaporator_inlet_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x80) != 0;
  const bool p06_evaporator_outlet_probe_fault = (buffer[DD_PACKET_ALARM_IDX] & 0x01) != 0;
  const bool p07_compressor_flow_probe_fault = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x10) != 0;
  const bool p08_solar_collector_probe_fault = (buffer[DD_PACKET_ALARM2_IDX] & 0x01) != 0;
  const bool e01_high_pressure_protection = (buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x40) != 0;
  const bool e02_solar_recirculation_alarm = (buffer[DD_PACKET_ALARM_IDX] & 0x02) != 0;
  const bool e03_electronic_fan_fault = (buffer[DD_PACKET_ALARM2_IDX] & 0x08) != 0;
  const bool pa_heat_pump_temp_unsuitable_alarm = (buffer[DD_PACKET_ALARM_IDX] & 0x10) != 0;
  const bool master_fault = p01_tank_lower_probe_fault || p02_tank_upper_probe_fault || p03_defrost_probe_fault ||
                            p04_inlet_air_probe_fault || p05_evaporator_inlet_probe_fault ||
                            p06_evaporator_outlet_probe_fault || p07_compressor_flow_probe_fault ||
                            p08_solar_collector_probe_fault || e01_high_pressure_protection ||
                            e02_solar_recirculation_alarm || e03_electronic_fan_fault ||
                            pa_heat_pump_temp_unsuitable_alarm;

  std::map<std::string, bool> binary_sensor_values = {
      {DIG1_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x01)},
      {DIG2_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x02)},
      {DIG3_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x04)},
      {MASTER_FAULT, master_fault},
      {P01_TANK_LOWER_PROBE_FAULT, p01_tank_lower_probe_fault},
      {P02_TANK_UPPER_PROBE_FAULT, p02_tank_upper_probe_fault},
      {P03_DEFROST_PROBE_FAULT, p03_defrost_probe_fault},
      {P04_INLET_AIR_PROBE_FAULT, p04_inlet_air_probe_fault},
      {P05_EVAPORATOR_INLET_PROBE_FAULT, p05_evaporator_inlet_probe_fault},
      {P06_EVAPORATOR_OUTLET_PROBE_FAULT, p06_evaporator_outlet_probe_fault},
      {P07_COMPRESSOR_FLOW_PROBE_FAULT, p07_compressor_flow_probe_fault},
      {P08_SOLAR_COLLECTOR_PROBE_FAULT, p08_solar_collector_probe_fault},
      {E01_HIGH_PRESSURE_PROTECTION, e01_high_pressure_protection},
      {E02_SOLAR_RECIRCULATION_ALARM, e02_solar_recirculation_alarm},
      {E03_ELECTRONIC_FAN_FAULT, e03_electronic_fan_fault},
      {PA_HEAT_PUMP_TEMP_UNSUITABLE_ALARM, pa_heat_pump_temp_unsuitable_alarm},
  };

  for (const auto &entry : binary_sensor_values) {
    set_binary_sensor_value(entry.first, entry.second);
  }

  return;
}

void DaikinEkhheComponent::update_time_band_state_from_bus_(const std::vector<uint8_t> &buffer,
                                                            bool d2_packet, bool force) {
  const uint8_t flag_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_FLAG_IDX)
                                     : static_cast<uint8_t>(CC_PACKET_TIME_BAND_FLAG_IDX);
  const uint8_t start_hour_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_START_HOUR_IDX)
                                           : static_cast<uint8_t>(CC_PACKET_TIME_BAND_START_HOUR_IDX);
  const uint8_t start_minute_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_START_MINUTE_IDX)
                                             : static_cast<uint8_t>(CC_PACKET_TIME_BAND_START_MINUTE_IDX);
  const uint8_t end_hour_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_END_HOUR_IDX)
                                         : static_cast<uint8_t>(CC_PACKET_TIME_BAND_END_HOUR_IDX);
  const uint8_t end_minute_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_END_MINUTE_IDX)
                                           : static_cast<uint8_t>(CC_PACKET_TIME_BAND_END_MINUTE_IDX);
  const uint8_t mode_idx = d2_packet ? static_cast<uint8_t>(D2_PACKET_TIME_BAND_MODE_IDX)
                                     : static_cast<uint8_t>(CC_PACKET_TIME_BAND_MODE_IDX);

  if (buffer.size() <= mode_idx) {
    return;
  }
  if ((time_band_state_.staged_dirty || tx_ui_sync_active_(TxOperationKind::TIME_BAND)) && !force) {
    return;
  }

  time_band_state_.initialized = true;
  time_band_state_.staged_dirty = false;
  time_band_state_.flag = buffer[flag_idx];
  time_band_state_.start_hour = buffer[start_hour_idx];
  time_band_state_.start_minute = buffer[start_minute_idx];
  time_band_state_.end_hour = buffer[end_hour_idx];
  time_band_state_.end_minute = buffer[end_minute_idx];
  time_band_state_.mode = buffer[mode_idx];

  set_number_value(TIME_BAND_START_HOUR, time_band_state_.start_hour);
  set_number_value(TIME_BAND_START_MINUTE, time_band_state_.start_minute);
  set_number_value(TIME_BAND_END_HOUR, time_band_state_.end_hour);
  set_number_value(TIME_BAND_END_MINUTE, time_band_state_.end_minute);
  set_select_value(TIME_BAND_MODE, time_band_state_.mode);
}

void DaikinEkhheComponent::parse_d2_packet(std::vector<uint8_t> buffer) {
  update_time_band_state_from_bus_(buffer, true);

  // update numbers
  std::map<std::string, float> number_values = {
      {P1_LOW_WAT_PROBE_HYST,     buffer[D2_PACKET_P1_IDX]},
      {P2_HEAT_ON_DELAY,          buffer[D2_PACKET_P2_IDX]},
      {VAC_DAYS,                  buffer[D2_PACKET_VAC_DAYS]},
      {P3_ANTL_SET_T,             buffer[D2_PACKET_P3_IDX]},
      {P4_ANTL_DURATION,          buffer[D2_PACKET_P4_IDX]},
      {P7_DEFROST_CYCLE_DELAY,    buffer[D2_PACKET_P7_IDX]},
      {P8_DEFR_START_THRES,       (int8_t)buffer[D2_PACKET_P8_IDX]},
      {P9_DEFR_STOP_THRES,        buffer[D2_PACKET_P9_IDX]},
      {P10_DEFR_MAX_DURATION,     buffer[D2_PACKET_P10_IDX]},
      {P17_HP_START_DELAY_DIG1,   buffer[D2_PACKET_P17_IDX]},
      {P18_LOW_WAT_T_DIG1,        buffer[D2_PACKET_P18_IDX]},
      {P19_LOW_WAT_T_HYST,        buffer[D2_PACKET_P19_IDX]},
      {P20_SOL_DRAIN_THRES,       buffer[D2_PACKET_P20_IDX]},
      {P21_LOW_WAT_T_HP_STOP,     buffer[D2_PACKET_P21_IDX]},
      {P22_UP_WAT_T_EH_STOP,      buffer[D2_PACKET_P22_IDX]},
      {P25_UP_WAT_T_OFFSET,       (int8_t)buffer[D2_PACKET_P25_IDX]},
      {P26_LOW_WAT_T_OFFSET,      (int8_t)buffer[D2_PACKET_P26_IDX]},
      {P27_INLET_T_OFFSET,        (int8_t)buffer[D2_PACKET_P27_IDX]},
      {P28_DEFR_T_OFFSET,         (int8_t)buffer[D2_PACKET_P28_IDX]},
      {P29_ANTL_START_HR,         buffer[D2_PACKET_P29_IDX]},
      {P30_UP_WAT_T_EH_HYST,      buffer[D2_PACKET_P30_IDX]},
      {P31_HP_PERIOD_AUTO,        buffer[D2_PACKET_P31_IDX]},
      {P32_EH_AUTO_TRES,          buffer[D2_PACKET_P32_IDX]},
      {P34_EEV_SH_PERIOD,         buffer[D2_PACKET_P34_IDX]},
      {P35_EEV_SH_SETPOINT,       (int8_t)buffer[D2_PACKET_P35_IDX]},
      {P36_EEV_DSH_SETPOINT,      buffer[D2_PACKET_P36_IDX]},
      {P37_EEV_STEP_DEFR,         buffer[D2_PACKET_P37_IDX]},      
      {P38_EEV_MIN_STEP_AUTO,     buffer[D2_PACKET_P38_IDX]},
      {P40_EEV_INIT_STEP,         buffer[D2_PACKET_P40_IDX]},
      {P41_AKP1_THRES,            (int8_t)buffer[D2_PACKET_P41_IDX]},
      {P42_AKP2_THRES,            (int8_t)buffer[D2_PACKET_P42_IDX]},
      {P43_AKP3_THRES,            (int8_t)buffer[D2_PACKET_P43_IDX]},
      {P44_EEV_KP1_GAIN,          (int8_t)buffer[D2_PACKET_P44_IDX]},
      {P45_EEV_KP2_GAIN,          (int8_t)buffer[D2_PACKET_P45_IDX]},
      {P46_EEV_KP3_GAIN,          (int8_t)buffer[D2_PACKET_P46_IDX]},
      {P47_MAX_INLET_T_HP,        buffer[D2_PACKET_P47_IDX]},
      {P48_MIN_INLET_T_HP,        (int8_t)buffer[D2_PACKET_P48_IDX]},
      {P49_EVA_INLET_THRES,       buffer[D2_PACKET_P49_IDX]},
      {P50_ANTIFREEZE_SET,        buffer[D2_PACKET_P50_IDX]},
      {P51_EVA_HIGH_SET,          buffer[D2_PACKET_P51_IDX]},
      {P52_EVA_LOW_SET,           buffer[D2_PACKET_P52_IDX]},
      {P54_LOW_PRESS_BYPASS,      buffer[D2_PACKET_P54_IDX]},

      {ECO_T_TEMPERATURE,         buffer[D2_PACKET_ECO_TTARGET_IDX]},
      {AUTO_T_TEMPERATURE,        buffer[D2_PACKET_AUTO_TTARGET_IDX]},
      {BOOST_T_TEMPERATURE,       buffer[D2_PACKET_BOOST_TTGARGET_IDX]},
      {ELECTRIC_T_TEMPERATURE,    buffer[D2_PACKET_ELECTRIC_TTARGET_IDX]},
  };


  for (const auto &entry : number_values) {
    set_number_value(entry.first, entry.second);
  }

  // update selects
  std::map<std::string, float> select_values = {
      {OPERATIONAL_MODE,       buffer[D2_PACKET_MODE_IDX]},
      {P12_EXT_PUMP_MODE,      buffer[D2_PACKET_P12_IDX]},
      {P14_EVA_BLOWER_TYPE,    buffer[D2_PACKET_P14_IDX]},
      {P16_SOLAR_MODE_INT,     buffer[D2_PACKET_P16_IDX]},
      {P23_PV_MODE_INT,        buffer[D2_PACKET_P23_IDX]},
      {P24_OFF_PEAK_MODE,      buffer[D2_PACKET_P24_IDX]},
  };

  for (const auto &entry : select_values) {
    set_select_value(entry.first, entry.second);
  }

  for (const auto &entry : SELECT_FIELD_SPECS) {
    const std::string &param_name = entry.first;
    const SelectFieldSpec &field = entry.second;
    if (field_is_extended(field.family) || field.bit_position == BIT_POSITION_NO_BITMASK) {
      continue;
    }
    if (field.index >= buffer.size()) {
      continue;
    }
    const uint8_t value = extract_field_value(
        buffer, field.index, field.bit_position,
        effective_bit_width(field.bit_position, field.bit_width));
    set_select_value(param_name, value);
  }

#if defined(USE_SWITCH)
  if (D2_PACKET_MASK2_IDX < buffer.size()) {
    const bool pending_silent_write =
        single_field_tx_matches_(TxPacketFamily::MAIN, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    const bool deferred_silent_write =
        has_deferred_user_tx_(TxPacketFamily::MAIN, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    if (!pending_silent_write && !deferred_silent_write) {
      set_switch_value(SILENT_MODE,
                       extract_field_value(buffer, D2_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1) != 0);
    }
  }
#endif

  update_timestamp(buffer[D2_PACKET_HOUR_IDX], buffer[D2_PACKET_MIN_IDX]);

#if defined(USE_WATER_HEATER)
  update_water_heater_main_cache_from_bus_(buffer, true);
#endif

  return;
}

void DaikinEkhheComponent::parse_d4_packet(std::vector<uint8_t> buffer) {
  const bool profile_ui_sync_active = tx_ui_sync_active_(TxOperationKind::PROFILE_RESTORE);
  const ProfileState &profile_sync_profile =
      tx_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
  const bool profile_d4_sync_matched =
      profile_ui_sync_active && profile_sync_profile.valid &&
      profile_matches_packet(true, profile_sync_profile.extended_data,
                             profile_sync_profile.extended_length, buffer, true);
  const bool pending_profile_active = profile_restore_tx_active_();

  for (const auto &entry : NUMBER_FIELD_SPECS) {
    const std::string &param_name = entry.first;
    const NumberFieldSpec &field = entry.second;
    if (!field_is_extended(field.family)) {
      continue;
    }
    uint8_t param_index = field.index;
    if (param_index >= buffer.size()) {
      continue;
    }
    if (single_field_tx_matches_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_active && !profile_d4_sync_matched)) &&
        is_profile_managed_field(true, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    set_number_value(param_name, decode_number_field(buffer, field));
  }
  for (const auto &entry : SELECT_FIELD_SPECS) {
    const std::string &param_name = entry.first;
    const SelectFieldSpec &field = entry.second;
    if (!field_is_extended(field.family)) {
      continue;
    }
    uint8_t param_index = field.index;
    if (param_index >= buffer.size()) {
      continue;
    }
    if (single_field_tx_matches_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_active && !profile_d4_sync_matched)) &&
        is_profile_managed_field(true, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    set_select_value(param_name, buffer[param_index]);
  }
}

void DaikinEkhheComponent::parse_c1_packet(std::vector<uint8_t> buffer) {
  if (tx_ui_sync_active_(TxOperationKind::RESTORE_DEFAULTS) &&
      restore_defaults_match_packet(buffer, false, true)) {
    tx_ui_sync_.extended_synced = true;
    if (tx_ui_sync_.main_synced) {
      DAIKIN_DBG(TAG, "Restore defaults UI synced: cc_c1_cycles=%u",
                 tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    }
  }

  if (tx_ui_sync_active_(TxOperationKind::PROFILE_RESTORE)) {
    const ProfileState &profile_sync_profile =
        tx_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
    if (profile_sync_profile.valid &&
        profile_matches_packet(true, profile_sync_profile.extended_data,
                               profile_sync_profile.extended_length, buffer, false)) {
      tx_ui_sync_.extended_synced = true;
      if (tx_ui_sync_.main_synced) {
        DAIKIN_DBG(TAG, "%s UI synced: cc_c1_cycles=%u",
                   tx_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                   tx_ui_sync_.cycles_waited + 1);
        reset_tx_ui_sync_();
      }
    }
  }

  if (!tx_ui_sync_active_(TxOperationKind::SINGLE_FIELD) ||
      tx_ui_sync_.single_field.family != TxPacketFamily::EXTENDED) {
    return;
  }

  const auto &target = tx_ui_sync_.single_field;
  const bool c1_sync_matched =
      field_matches_target_(buffer, target.index, target.value, target.bit_position, target.bit_width);
  if (c1_sync_matched) {
    DAIKIN_DBG(TAG, "TX UI synced: family=extended index=%u bit=%u value=0x%02X c1_cycles=%u",
               target.index, target.bit_position, target.value,
               tx_ui_sync_.cycles_waited + 1);
    reset_tx_ui_sync_();
    return;
  }

  tx_ui_sync_.cycles_waited++;
  if (tx_ui_sync_.cycles_waited >= kTxUiSyncMaxCycles) {
    DAIKIN_WARN(TAG, "TX UI sync timeout: family=extended index=%u bit=%u value=0x%02X cycles=%u",
                target.index, target.bit_position, target.value,
                tx_ui_sync_.cycles_waited);
    reset_tx_ui_sync_();
  }
}

void DaikinEkhheComponent::parse_cc_packet(std::vector<uint8_t> buffer) {
  const bool single_field_ui_sync_active = tx_ui_sync_active_(TxOperationKind::SINGLE_FIELD);
  const bool single_field_main_ui_sync_active =
      single_field_ui_sync_active && tx_ui_sync_.single_field.family == TxPacketFamily::MAIN;
  const auto &single_field_sync = tx_ui_sync_.single_field;
  const bool restore_ui_sync_active = tx_ui_sync_active_(TxOperationKind::RESTORE_DEFAULTS);
  const bool profile_ui_sync_active = tx_ui_sync_active_(TxOperationKind::PROFILE_RESTORE);
  const bool time_band_ui_sync_active = tx_ui_sync_active_(TxOperationKind::TIME_BAND);
  const auto &time_band_sync = tx_ui_sync_.time_band;
#if defined(USE_WATER_HEATER)
  const bool water_heater_ui_sync_active = tx_ui_sync_active_(TxOperationKind::WATER_HEATER);
  const auto &water_heater_sync = tx_ui_sync_.water_heater;
  const bool water_heater_cc_sync_matched =
      water_heater_ui_sync_active &&
      water_heater_tx_matches_base_packet_(water_heater_sync, buffer);
#endif
  const bool cc_sync_matched =
      single_field_main_ui_sync_active &&
      field_matches_target_(buffer, single_field_sync.index, single_field_sync.value,
                            single_field_sync.bit_position, single_field_sync.bit_width);
  const bool restore_cc_sync_matched =
      restore_ui_sync_active && restore_defaults_match_packet(buffer, false, false);
  const ProfileState &profile_sync_profile =
      tx_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
  const bool profile_cc_sync_matched =
      profile_ui_sync_active && profile_sync_profile.valid &&
      profile_matches_packet(false, profile_sync_profile.main_data,
                             profile_sync_profile.main_length, buffer, false);
  const bool time_band_cc_sync_matched =
      time_band_ui_sync_active &&
      time_band_matches_packet_(buffer, false, time_band_sync.flag,
                                time_band_sync.start_hour, time_band_sync.start_minute,
                                time_band_sync.end_hour, time_band_sync.end_minute,
                                time_band_sync.mode);
  const bool pending_profile_active = profile_restore_tx_active_();

  update_time_band_state_from_bus_(buffer, false, time_band_cc_sync_matched);

  for (const auto &entry : NUMBER_FIELD_SPECS) {
    const std::string &param_name = entry.first;
    const NumberFieldSpec &field = entry.second;
    if (field_is_extended(field.family)) {
      continue;
    }
    uint8_t param_index = field.index;
    if (single_field_tx_matches_(TxPacketFamily::MAIN, param_index, BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if (single_field_main_ui_sync_active && single_field_sync.index == param_index &&
        single_field_sync.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    if ((restore_tx_active_() || (restore_ui_sync_active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    set_number_value(param_name, decode_number_field(buffer, field));
  }

  for (const auto &entry : SELECT_FIELD_SPECS) {
    const std::string &param_name = entry.first;
    const SelectFieldSpec &field = entry.second;
    if (field_is_extended(field.family)) {
      continue;
    }
    uint8_t param_index = field.index;
    uint8_t bit_position = field.bit_position;
    uint8_t bit_width = field.bit_width;
    if (single_field_tx_matches_(TxPacketFamily::MAIN, param_index, bit_position) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, bit_position)) {
      continue;
    }
    if (single_field_main_ui_sync_active && single_field_sync.index == param_index &&
        single_field_sync.bit_position == bit_position && !cc_sync_matched) {
      continue;
    }
    if ((restore_tx_active_() || (restore_ui_sync_active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, bit_position)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, bit_position)) {
      continue;
    }
    uint8_t value = bit_position == BIT_POSITION_NO_BITMASK
                        ? buffer[param_index]
                        : extract_field_value(buffer, param_index, bit_position,
                                              effective_bit_width(bit_position, bit_width));
    set_select_value(param_name, value);
  }

#if defined(USE_SWITCH)
  if (CC_PACKET_MASK2_IDX < buffer.size()) {
    const bool pending_silent_write =
        single_field_tx_matches_(TxPacketFamily::MAIN, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    const bool deferred_silent_write =
        has_deferred_user_tx_(TxPacketFamily::MAIN, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    const bool waiting_for_silent_ui_sync =
        single_field_main_ui_sync_active &&
        single_field_sync.index == CC_PACKET_MASK2_IDX &&
        single_field_sync.bit_position == SILENT_MODE_BIT_POSITION && !cc_sync_matched;
    const bool waiting_for_silent_profile_sync =
        (pending_profile_active || (profile_ui_sync_active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    if (!pending_silent_write && !deferred_silent_write && !waiting_for_silent_ui_sync &&
        !waiting_for_silent_profile_sync) {
      set_switch_value(SILENT_MODE,
                       extract_field_value(buffer, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1) != 0);
    }
  }
#endif

  set_text_sensor_value_(L_UI_FW_VERSION, "U" + std::to_string(buffer[CC_PACKET_L_FW_IDX]));

  update_timestamp(buffer[CC_PACKET_HOUR_IDX], buffer[CC_PACKET_MIN_IDX]);

#if defined(USE_WATER_HEATER)
  if (!water_heater_ui_sync_active || water_heater_cc_sync_matched) {
    update_water_heater_main_cache_from_bus_(buffer, false);
  }
#endif

  if (single_field_main_ui_sync_active) {
    if (cc_sync_matched) {
      DAIKIN_DBG(TAG, "TX UI synced: index=%u bit=%u value=0x%02X cc_cycles=%u",
                 single_field_sync.index, single_field_sync.bit_position, single_field_sync.value,
                 tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kTxUiSyncMaxCycles) {
        DAIKIN_WARN(TAG, "TX UI sync timeout: index=%u bit=%u value=0x%02X cycles=%u",
                    single_field_sync.index, single_field_sync.bit_position, single_field_sync.value,
                    tx_ui_sync_.cycles_waited);
        reset_tx_ui_sync_();
      }
    }
  }

  if (restore_ui_sync_active) {
    if (restore_cc_sync_matched) {
      tx_ui_sync_.main_synced = true;
    }
    if (tx_ui_sync_.main_synced && tx_ui_sync_.extended_synced) {
      DAIKIN_DBG(TAG, "Restore defaults UI synced: cc_c1_cycles=%u", tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kRestoreUiSyncMaxCycles) {
        uint8_t current_value = 0;
        const bool extended_pending = tx_ui_sync_.main_synced && !tx_ui_sync_.extended_synced;
        const std::vector<uint8_t> &sync_buffer = extended_pending ? last_c1_packet_ : buffer;
        const RestoreFieldSpec *field = first_restore_mismatch(sync_buffer, false, current_value,
                                                               extended_pending);
        if (field != nullptr) {
          DAIKIN_WARN(TAG,
                      "Restore defaults UI sync timeout: family=%s first_mismatch=%s expected=0x%02X current=0x%02X cc_c1_cycles=%u",
                      extended_pending ? "extended" : "main",
                      field->name, field->value, current_value, tx_ui_sync_.cycles_waited);
        } else {
          DAIKIN_WARN(TAG,
                      "Restore defaults UI sync timeout: main_synced=%u extended_synced=%u cc_c1_cycles=%u",
                      tx_ui_sync_.main_synced, tx_ui_sync_.extended_synced,
                      tx_ui_sync_.cycles_waited);
        }
        reset_tx_ui_sync_();
      }
    }
  }

  if (profile_ui_sync_active) {
    if (profile_cc_sync_matched) {
      tx_ui_sync_.main_synced = true;
    }
    if (tx_ui_sync_.main_synced && tx_ui_sync_.extended_synced) {
      DAIKIN_DBG(TAG, "%s UI synced: cc_c1_cycles=%u",
                 tx_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                 tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kProfileUiSyncMaxCycles) {
        uint8_t expected_value = 0;
        uint8_t current_value = 0;
        const bool extended_pending = tx_ui_sync_.main_synced && !tx_ui_sync_.extended_synced;
        const std::vector<uint8_t> &sync_buffer = extended_pending ? last_c1_packet_ : buffer;
        const uint8_t *profile_data = extended_pending ? profile_sync_profile.extended_data
                                                       : profile_sync_profile.main_data;
        const size_t profile_length = extended_pending ? profile_sync_profile.extended_length
                                                       : profile_sync_profile.main_length;
        const ManagedFieldSpec *field = first_profile_mismatch(
            extended_pending, profile_data, profile_length, sync_buffer, false,
            expected_value, current_value);
        if (field != nullptr) {
          DAIKIN_WARN(TAG,
                      "%s UI sync timeout: family=%s first_mismatch=%s expected=0x%02X current=0x%02X cc_c1_cycles=%u",
                      tx_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                      extended_pending ? "extended" : "main",
                      field->name, expected_value, current_value, tx_ui_sync_.cycles_waited);
        } else {
          DAIKIN_WARN(TAG, "%s UI sync timeout: main_synced=%u extended_synced=%u cc_c1_cycles=%u",
                      tx_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                      tx_ui_sync_.main_synced, tx_ui_sync_.extended_synced,
                      tx_ui_sync_.cycles_waited);
        }
        reset_tx_ui_sync_();
      }
    }
  }

  if (time_band_ui_sync_active) {
    if (time_band_cc_sync_matched) {
      DAIKIN_DBG(TAG, "Time-band UI synced: cc_cycles=%u", tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kTimeBandUiSyncMaxCycles) {
        DAIKIN_WARN(TAG,
                    "Time-band UI sync timeout: expected flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u cc_cycles=%u",
                    time_band_sync.flag, time_band_sync.start_hour,
                    time_band_sync.start_minute, time_band_sync.end_hour,
                    time_band_sync.end_minute, time_band_sync.mode,
                    tx_ui_sync_.cycles_waited);
        reset_tx_ui_sync_();
      }
    }
  }

#if defined(USE_WATER_HEATER)
  if (water_heater_ui_sync_active) {
    if (water_heater_cc_sync_matched) {
      DAIKIN_DBG(TAG, "Native water heater UI synced: cc_cycles=%u", tx_ui_sync_.cycles_waited + 1);
      reset_tx_ui_sync_();
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kTxUiSyncMaxCycles) {
        const WaterHeaterTxField *first_mismatch = nullptr;
        water_heater_tx_matches_base_packet_(water_heater_sync, buffer, &first_mismatch);
        if (first_mismatch != nullptr && first_mismatch->write_index < buffer.size()) {
          uint8_t current_value = buffer[first_mismatch->write_index];
          uint8_t expected_value = first_mismatch->write_value;
          if (first_mismatch->write_bit_position != BIT_POSITION_NO_BITMASK) {
            current_value = extract_field_value(buffer, first_mismatch->write_index,
                                                first_mismatch->write_bit_position,
                                                first_mismatch->write_bit_width);
            expected_value &= field_value_mask(first_mismatch->write_index,
                                               first_mismatch->write_bit_position,
                                               first_mismatch->write_bit_width);
          }
          DAIKIN_WARN(TAG,
                      "Native water heater UI sync timeout: first_mismatch=%s expected=0x%02X current=0x%02X cc_cycles=%u",
                      first_mismatch->name, expected_value, current_value, tx_ui_sync_.cycles_waited);
        } else {
          DAIKIN_WARN(TAG, "Native water heater UI sync timeout: fields=%u cc_cycles=%u",
                      static_cast<unsigned>(water_heater_sync.fields.size()),
                      tx_ui_sync_.cycles_waited);
        }
        reset_tx_ui_sync_();
        update_water_heater_main_cache_from_bus_(buffer, false);
      }
    }
  }
#endif

  return;
}

}  // namespace daikin_ekkhe
}  // namespace esphome
