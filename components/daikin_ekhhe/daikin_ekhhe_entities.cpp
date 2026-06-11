#include "daikin_ekhhe.h"
#include "daikin_ekhhe_metadata.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"

#include <cmath>
#include <ctime>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

bool DaikinEkhheComponent::should_publish_float_(const std::string &key, float value,
                                                 std::map<std::string, float> &last_values,
                                                 std::map<std::string, uint32_t> &last_publish_ms,
                                                 uint32_t min_interval_ms, float epsilon,
                                                 uint32_t refresh_ms) {
  uint32_t now = millis();
  auto it = last_values.find(key);
  if (it == last_values.end()) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  float last_value = it->second;
  uint32_t last_ms = last_publish_ms[key];
  float delta = fabsf(value - last_value);
  bool refresh_due = refresh_ms > 0 && (now - last_ms) >= refresh_ms;
  bool interval_ok = min_interval_ms == 0 || (now - last_ms) >= min_interval_ms;

  if (delta > epsilon && interval_ok) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  if (refresh_due) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  return false;
}

bool DaikinEkhheComponent::should_publish_bool_(const std::string &key, bool value,
                                                std::map<std::string, bool> &last_values,
                                                std::map<std::string, uint32_t> &last_publish_ms,
                                                uint32_t refresh_ms) {
  uint32_t now = millis();
  auto it = last_values.find(key);
  if (it == last_values.end()) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  uint32_t last_ms = last_publish_ms[key];
  bool refresh_due = refresh_ms > 0 && (now - last_ms) >= refresh_ms;
  if (it->second != value || refresh_due) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  return false;
}

bool DaikinEkhheComponent::should_publish_text_(const std::string &key, const std::string &value,
                                                std::map<std::string, std::string> &last_values,
                                                std::map<std::string, uint32_t> &last_publish_ms,
                                                uint32_t refresh_ms) {
  uint32_t now = millis();
  auto it = last_values.find(key);
  if (it == last_values.end()) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  uint32_t last_ms = last_publish_ms[key];
  bool refresh_due = refresh_ms > 0 && (now - last_ms) >= refresh_ms;
  if (it->second != value || refresh_due) {
    last_values[key] = value;
    last_publish_ms[key] = now;
    return true;
  }

  return false;
}

void DaikinEkhheComponent::register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor) {
  if (sensor != nullptr) {
    sensors_[sensor_name] = sensor;
  }
}

void DaikinEkhheComponent::register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor) {
  if (binary_sensor != nullptr) {
    binary_sensors_[sensor_name] = binary_sensor;
  }
}

void DaikinEkhheComponent::register_number(const std::string &number_name, esphome::number::Number *number) {
  if (number != nullptr) {
    numbers_[number_name] = number;
    if (number_name == TX_SEND_CALIBRATION) {
      number->publish_state(this->tx_delay_after_d2_ms_);
    }
  }
}

void DaikinEkhheComponent::register_select(const std::string &select_name, select::Select *select) {
  if (select != nullptr) {
    selects_[select_name] = static_cast<DaikinEkhheSelect *>(select);
  }
}

void DaikinEkhheComponent::register_text_sensor(const std::string &text_sensor_name,
                                                esphome::text_sensor::TextSensor *sensor) {
  if (sensor != nullptr) {
    text_sensors_[text_sensor_name] = sensor;
  }
}

void DaikinEkhheComponent::register_timestamp_sensor(text_sensor::TextSensor *sensor) {
  this->timestamp_sensor_ = sensor;
}

#if defined(USE_SWITCH)
void DaikinEkhheComponent::register_switch(const std::string &switch_name, switch_::Switch *sw) {
  if (sw != nullptr) {
    switches_[switch_name] = sw;
  }
}
#endif

void DaikinEkhheComponent::register_known_good_profile_status_sensor(esphome::text_sensor::TextSensor *sensor) {
  this->known_good_profile_status_sensor_ = sensor;
  publish_profile_status_(true);
}

void DaikinEkhheComponent::register_auto_snapshot_status_sensor(esphome::text_sensor::TextSensor *sensor) {
  this->auto_snapshot_status_sensor_ = sensor;
  publish_profile_status_(false);
}

void DaikinEkhheComponent::set_sensor_value(const std::string &sensor_name, float value) {
  if (sensors_.find(sensor_name) != sensors_.end()) {
    if (!cycle_publish_allowed_) {
      return;
    }
    if (should_publish_float_(sensor_name, value, last_published_sensor_values_,
                              last_published_sensor_ms_, kFastPublishMinIntervalMs,
                              kFloatPublishEpsilon, 0)) {
      defer([this, sensor_name, value]() {
        sensors_[sensor_name]->publish_state(value);
      });
    }
  }

}

void DaikinEkhheComponent::set_text_sensor_value_(const std::string &text_sensor_name, const std::string &value) {
  if (text_sensors_.find(text_sensor_name) == text_sensors_.end()) {
    return;
  }
  if (!cycle_publish_allowed_) {
    return;
  }
  if (should_publish_text_(text_sensor_name, value, last_published_text_values_,
                           last_published_text_ms_, kSlowPublishRefreshMs)) {
    defer([this, text_sensor_name, value]() {
      text_sensors_[text_sensor_name]->publish_state(value);
    });
  }
}

void DaikinEkhheComponent::set_binary_sensor_value(const std::string &sensor_name, bool value) {
  if (binary_sensors_.find(sensor_name) != binary_sensors_.end()) {
    if (!cycle_publish_allowed_) {
      return;
    }
    if (should_publish_bool_(sensor_name, value, last_published_binary_values_,
                             last_published_binary_ms_, kSlowPublishRefreshMs)) {
      defer([this, sensor_name, value]() {
        binary_sensors_[sensor_name]->publish_state(value);
      });
    }
  }
}

#if defined(USE_SWITCH)
void DaikinEkhheComponent::set_switch_value(const std::string &switch_name, bool value) {
  if (switches_.find(switch_name) != switches_.end()) {
    if (!cycle_publish_allowed_) {
      return;
    }
    if (should_publish_bool_(switch_name, value, last_published_switch_values_,
                             last_published_switch_ms_, kSlowPublishRefreshMs)) {
      defer([this, switch_name, value]() {
        switches_[switch_name]->publish_state(value);
      });
    }
  }
}
#endif

void DaikinEkhheComponent::set_number_value(const std::string &number_name, float value) {
  // This sets a number value that's been gotten from the UART stream, not something that's 
  // set through the API or UI
  if (numbers_.find(number_name) != numbers_.end()) {
    if (!cycle_publish_allowed_) {
      return;
    }
    if (should_publish_float_(number_name, value, last_published_number_values_,
                              last_published_number_ms_, 0, kFloatPublishEpsilon,
                              kSlowPublishRefreshMs)) {
      defer([this, number_name, value]() {
          numbers_[number_name]->publish_state(value);
      });
    }
  }
}

void DaikinEkhheComponent::set_select_value(const std::string &select_name, int value) {
  // This sets a select value that's been gotten from the UART stream, not something that's 
  // set through the API or UI
  if (selects_.count(select_name)) {
    if (!cycle_publish_allowed_) {
      return;
    }
    DaikinEkhheSelect *select = selects_[select_name];

        // Find the corresponding string option for the numeric value
        for (const auto &entry : select->get_select_mappings()) {
            if (entry.second == value) {                
                if (should_publish_text_(select_name, entry.first, last_published_select_values_,
                                         last_published_select_ms_, kSlowPublishRefreshMs)) {
                  // Update ESPHome with the new selected value
                  defer([this, select, entry]() {
                    select->publish_state(entry.first);
                  });
                }
                return;
            }
        }
  }
}

void DaikinEkhheComponent::update_timestamp(uint8_t hour, uint8_t minute) {
    if (this->timestamp_sensor_ != nullptr) {
        if (!cycle_publish_allowed_) {
            return;
        }
        ESPTime now = (*clock).now();

        if (!now.is_valid()) {
            DAIKIN_WARN(TAG, "Time not available yet. Skipping timestamp update.");
            return;
        }

        // Create a struct to hold the time data
        struct tm timeinfo = now.to_c_tm();  // Get current UTC time in struct tm

        // Apply the received hour & minute from UART
        timeinfo.tm_hour = hour;
        timeinfo.tm_min = minute;
        timeinfo.tm_sec = 0;  // Default to 0 seconds

        // Convert back to time_t for consistency
        time_t adjusted_time = mktime(&timeinfo);

        // Format as ISO 8601 UTC timestamp
        char timestamp[25];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", gmtime(&adjusted_time));

        // Publish the timestamp to Home Assistant
        uint32_t now_ms = millis();
        bool refresh_due = (now_ms - last_published_timestamp_ms_) >= kTimestampRefreshMs;
        if (last_published_timestamp_ != timestamp || refresh_due) {
          last_published_timestamp_ = timestamp;
          last_published_timestamp_ms_ = now_ms;
          this->timestamp_sensor_->publish_state(timestamp);
        }
    }
}

void DaikinEkhheComponent::update_number_cache(const std::string &number_name, float value) {
  uint32_t now = millis();
  last_published_number_values_[number_name] = value;
  last_published_number_ms_[number_name] = now;
}

void DaikinEkhheComponent::update_select_cache(const std::string &select_name, const std::string &value) {
  uint32_t now = millis();
  last_published_select_values_[select_name] = value;
  last_published_select_ms_[select_name] = now;
}

#if defined(USE_SWITCH)
void DaikinEkhheComponent::update_switch_cache(const std::string &switch_name, bool value) {
  uint32_t now = millis();
  last_published_switch_values_[switch_name] = value;
  last_published_switch_ms_[switch_name] = now;
}

bool DaikinEkhheComponent::current_operational_mode_(uint8_t &mode) const {
  if (last_d2_packet_.size() > D2_PACKET_MODE_IDX) {
    mode = last_d2_packet_[D2_PACKET_MODE_IDX];
    return true;
  }
  if (last_cc_packet_.size() > CC_PACKET_MODE_IDX) {
    mode = last_cc_packet_[CC_PACKET_MODE_IDX];
    return true;
  }
  return false;
}

bool DaikinEkhheComponent::silent_mode_allowed_mode_(uint8_t mode) const {
  return mode == kOperationalModeAuto || mode == kOperationalModeEco || mode == kOperationalModeBoost;
}

void DaikinEkhheComponent::publish_silent_mode_from_latest_packet_() {
  bool enabled = false;
  if (last_d2_packet_.size() > D2_PACKET_MASK2_IDX) {
    enabled = extract_field_value(last_d2_packet_, D2_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1) != 0;
  } else if (last_cc_packet_.size() > CC_PACKET_MASK2_IDX) {
    enabled = extract_field_value(last_cc_packet_, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1) != 0;
  }
  set_switch_value(SILENT_MODE, enabled);
}

bool DaikinEkhheComponent::set_silent_mode(bool enabled) {
  if (enabled) {
    uint8_t mode = 0;
    if (!current_operational_mode_(mode)) {
      DAIKIN_WARN(TAG, "Silent mode cannot be enabled before an operational mode has been received.");
      publish_silent_mode_from_latest_packet_();
      return false;
    }
    if (!silent_mode_allowed_mode_(mode)) {
      DAIKIN_WARN(TAG, "Silent mode can only be enabled in Auto, Eco, or Boost mode; current mode=%u", mode);
      publish_silent_mode_from_latest_packet_();
      return false;
    }
  }

  if (!send_uart_cc_command(CC_PACKET_MASK2_IDX, enabled ? 1 : 0, SILENT_MODE_BIT_POSITION, 1)) {
    publish_silent_mode_from_latest_packet_();
    return false;
  }
  return true;
}
#endif

bool DaikinEkhheComponent::stage_time_band_number(const std::string &number_name, float value) {
  const uint8_t staged_value = value <= 0.0f ? 0 : static_cast<uint8_t>(value);
  if (number_name == TIME_BAND_START_HOUR) {
    time_band_state_.start_hour = staged_value;
  } else if (number_name == TIME_BAND_START_MINUTE) {
    time_band_state_.start_minute = staged_value;
  } else if (number_name == TIME_BAND_END_HOUR) {
    time_band_state_.end_hour = staged_value;
  } else if (number_name == TIME_BAND_END_MINUTE) {
    time_band_state_.end_minute = staged_value;
  } else {
    return false;
  }

  time_band_state_.initialized = true;
  time_band_state_.staged_dirty = true;
  DAIKIN_DBG(TAG, "Time-band staged: %s=%u", number_name.c_str(), staged_value);
  return true;
}

bool DaikinEkhheComponent::stage_time_band_mode(uint8_t mode) {
  time_band_state_.mode = mode;
  time_band_state_.initialized = true;
  time_band_state_.staged_dirty = true;
  DAIKIN_DBG(TAG, "Time-band staged: mode=%u", mode);
  return true;
}

void DaikinEkhheComponent::apply_time_band() {
  if (!time_band_state_.initialized && !last_cc_packet_.empty()) {
    update_time_band_state_from_bus_(last_cc_packet_, false, true);
  }
  if (!time_band_state_.initialized) {
    DAIKIN_WARN(TAG, "Time-band apply requested before time-band state was initialized from the bus.");
    return;
  }

  request_time_band_tx_(kTimeBandApplyFlag, time_band_state_.start_hour,
                        time_band_state_.start_minute, time_band_state_.end_hour,
                        time_band_state_.end_minute, time_band_state_.mode);
}

void DaikinEkhheComponent::clear_time_band() {
  if (!time_band_state_.initialized && !last_cc_packet_.empty()) {
    update_time_band_state_from_bus_(last_cc_packet_, false, true);
  }
  if (!time_band_state_.initialized) {
    DAIKIN_WARN(TAG, "Time-band clear requested before time-band state was initialized from the bus.");
    return;
  }

  request_time_band_tx_(kTimeBandClearFlag, 0, 0, 0, 0, time_band_state_.mode);
}

#if defined(USE_SWITCH)
void DaikinEkhheSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    DAIKIN_WARN(TAG, "Parent component is null, cannot send Switch command.");
    return;
  }

  auto name = this->internal_id_;
  if (name == SILENT_MODE) {
    if (this->parent_->set_silent_mode(state)) {
      this->parent_->update_switch_cache(name, state);
      this->publish_state(state);
    }
    return;
  }

  DAIKIN_WARN(TAG, "No matching UART command for Switch: %s", name.c_str());
}
#endif

#if defined(USE_BUTTON)
void DaikinEkhheActionButton::press_action() {
  if (this->parent_ == nullptr) {
    return;
  }
  if (this->action_ == Action::RESTORE_DEFAULT_SETTINGS) {
    this->parent_->restore_default_settings();
  } else if (this->action_ == Action::SAVE_KNOWN_GOOD_PROFILE) {
    this->parent_->save_known_good_profile();
  } else if (this->action_ == Action::RESTORE_KNOWN_GOOD_PROFILE) {
    this->parent_->restore_known_good_profile();
  } else if (this->action_ == Action::RESTORE_AUTO_SNAPSHOT) {
    this->parent_->restore_auto_snapshot();
  } else if (this->action_ == Action::APPLY_TIME_BAND) {
    this->parent_->apply_time_band();
  } else if (this->action_ == Action::CLEAR_TIME_BAND) {
    this->parent_->clear_time_band();
  }
}
#endif

void DaikinEkhheNumber::control(float value) {

    if (this->parent_ == nullptr) {
        DAIKIN_WARN(TAG, "Parent component is null, cannot send Number command.");
        return;
    }

    // Use get_name() to determine which UART command to send
    auto name = this->internal_id_;

    if (name == TX_SEND_CALIBRATION) {
        uint32_t delay_ms = value <= 0.0f ? 0 : static_cast<uint32_t>(value);
        this->parent_->set_tx_delay_after_d2_ms(delay_ms);
        delay_ms = this->parent_->get_tx_delay_after_d2_ms();
        ESP_LOGI(TAG, "TX send calibration set to %u ms", delay_ms);
        return;
    }

    if (this->parent_->stage_time_band_number(name, value)) {
        this->parent_->update_number_cache(name, value);
        this->publish_state(value);
        return;
    }

    auto field_it = NUMBER_FIELD_SPECS.find(name);
    if (field_it == NUMBER_FIELD_SPECS.end()) {
        DAIKIN_WARN(TAG, "No matching UART command for Number: %s", name.c_str());
        return;
    }

    const NumberFieldSpec &field = field_it->second;
    const uint8_t uart_value = encode_number_field(value, field);
    const bool sent = field_is_extended(field.family)
                          ? this->parent_->send_uart_c2_command(field.index, uart_value, BIT_POSITION_NO_BITMASK)
                          : this->parent_->send_uart_cc_command(field.index, uart_value, BIT_POSITION_NO_BITMASK);
    if (sent) {
        this->parent_->update_number_cache(name, value);
        this->publish_state(value);
    }
}

void DaikinEkhheSelect::control(const std::string &value) {
    if (this->parent_ == nullptr) {
        DAIKIN_WARN(TAG, "Parent component is null, cannot send Select.");
        return;
    }

    // Use get_name() to determine which UART command to send
    auto name = this->internal_id_;

    // Find the correct value index from the mapping
    auto mappings = this->get_select_mappings();
    if (mappings.empty()) {
        DAIKIN_WARN(TAG, "No select mappings found for %s!", name.c_str());
        return;
    }

    auto it = mappings.find(value);
    if (it == mappings.end()) {
        DAIKIN_WARN(TAG, "Invalid select option %s for entity %s!", value.c_str(), name.c_str());
        return;
    }
    uint8_t uart_value = static_cast<uint8_t>(it->second);

    if (name == TIME_BAND_MODE) {
        if (this->parent_->stage_time_band_mode(uart_value)) {
            this->parent_->update_select_cache(name, value);
            this->publish_state(value);
        }
        return;
    }

    auto field_it = SELECT_FIELD_SPECS.find(name);
    if (field_it == SELECT_FIELD_SPECS.end()) {
        DAIKIN_WARN(TAG, "No matching UART command for Select: %s", name.c_str());
        return;
    }

    const SelectFieldSpec &field = field_it->second;
    // Update value in ESPHome
    bool sent = field_is_extended(field.family)
                    ? this->parent_->send_uart_c2_command(field.index, uart_value, field.bit_position, field.bit_width)
                    : this->parent_->send_uart_cc_command(field.index, uart_value, field.bit_position, field.bit_width);
    if (sent) {
        this->parent_->update_select_cache(name, value);
        this->publish_state(value);
    }
}

}  // namespace daikin_ekkhe
}  // namespace esphome
