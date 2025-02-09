#include "daikin_ekhhe.h"
#include "daikin_ekhhe_const.h"
#include "esphome/core/log.h"


#include <cinttypes>
#include <numeric>
#include <ctime>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

static const uint8_t DD_PACKET_START_BYTE = 0xDD;
static const uint8_t D2_PACKET_START_BYTE = 0xD2;
static const uint8_t D4_PACKET_START_BYTE = 0xD4;
static const uint8_t C1_PACKET_START_BYTE = 0xC1;
static const uint8_t CC_PACKET_START_BYTE = 0xCC;
static const uint8_t CD_PACKET_START_BYTE = 0xCD;


// Packet definitions
static const std::map<uint8_t, uint8_t> PACKET_SIZES = {
    {DD_PACKET_START_BYTE, DaikinEkhheComponent::DD_PACKET_SIZE},
    {D2_PACKET_START_BYTE, DaikinEkhheComponent::D2_PACKET_SIZE},
    {D4_PACKET_START_BYTE, DaikinEkhheComponent::D4_PACKET_SIZE},
    {C1_PACKET_START_BYTE, DaikinEkhheComponent::C1_PACKET_SIZE},
    {CC_PACKET_START_BYTE, DaikinEkhheComponent::CC_PACKET_SIZE},
    {CD_PACKET_START_BYTE, DaikinEkhheComponent::CD_PACKET_SIZE},
};


void DaikinEkhheComponent::setup() {
    ESP_LOGI(TAG, "Setting up Daikin EKHHE component...");
    start_uart_cycle();
}

void DaikinEkhheComponent::loop() {
    // only enable UART every PROCESS_INTERVAL_MS
    unsigned long now = millis();
    if (!uart_active_) {
        if (now - last_process_time_ >= update_interval_ || last_process_time_ == 0) {
            start_uart_cycle();  
        }
        return;
    }


  while (this->available()) {
    uint8_t byte = this->read();

    if (!receiving_) {
      // Check if this byte is a valid packet start byte
      if (PACKET_SIZES.find(byte) != PACKET_SIZES.end()) {
        ESP_LOGI(TAG, "Detected packet start byte: 0x%X", byte);
        buffer_.clear();
        buffer_.push_back(byte);
        expected_length_ = PACKET_SIZES.at(byte);
        receiving_ = true;
      }
    } else {
      // Receiving a packet
      buffer_.push_back(byte);

      if (buffer_.size() >= expected_length_) {
        // Packet fully received
        receiving_ = false;
        process_uart_buffer();
      }
    }
  }
}

void DaikinEkhheComponent::start_uart_cycle() {
    ESP_LOGI(TAG, "Starting UART read cycle...");
    uart_active_ = true;
    last_process_time_ = millis();
}

DaikinEkhheComponent::EkhheError DaikinEkhheComponent::process_uart_buffer() {

  // Checksum check
  if (buffer_.empty()) {
    return EKHHE_ERROR_BUFFER_EMPTY;
  }

  uint8_t checksum = ekhhe_checksum(buffer_);

  if (checksum != buffer_.back()) {
    return EKHHE_ERROR_CHECKSUM;
  }

  // Extract packet type
  uint8_t packet_type = buffer_[0];

  switch (packet_type) {
    case DD_PACKET_START_BYTE:
      last_dd_packet_ = buffer_;
      break;
    case D2_PACKET_START_BYTE:
      last_d2_packet_ = buffer_;
      break;
    case D4_PACKET_START_BYTE:
      last_d4_packet_ = buffer_;
      break;
    case C1_PACKET_START_BYTE:
      last_c1_packet_ =  buffer_;
      break;
    case CC_PACKET_START_BYTE:
      last_cc_packet_ = buffer_;
      break;
  }

  ESP_LOGV(TAG, "Stored packet type: 0x%X", packet_type);

  if (!last_d2_packet_.empty() && !last_dd_packet_.empty() && !last_cc_packet_.empty() &&
      !last_c1_packet_.empty() && !last_d4_packet_.empty()) {
      
      process_packet_set();
      uart_active_ = false;  // Disable UART reading until next cycle
  }

  return EKHHE_ERROR_NONE;
}

void DaikinEkhheComponent::process_packet_set() {
  /* 
  if (!last_d2_packet_.empty()) {
        ESP_LOGD(TAG, "Processing D2 Packet");
        parse_d2_packet(last_d2_packet_);
  }
  */
  if (!last_dd_packet_.empty()) {
        ESP_LOGD(TAG, "Processing DD Packet");
        parse_dd_packet(last_dd_packet_);
  }
  if (!last_cc_packet_.empty()) {
        ESP_LOGD(TAG, "Processing CC Packet");
        parse_cc_packet(last_cc_packet_);
  }
  /* 
  if (!last_c1_packet_.empty()) {
        ESP_LOGD(TAG, "Processing C1 Packet");
        parse_c1_packet(last_c1_packet_);
  }
  if (!last_d4_packet_.empty()) {
        ESP_LOGD(TAG, "Processing D4 Packet");
        parse_d4_packet(last_d4_packet_);
  }
  */

  // Clear stored packets after processing (except for cc)
  last_d2_packet_.clear();
  last_dd_packet_.clear();
  last_c1_packet_.clear();
  last_d4_packet_.clear();
}

void DaikinEkhheComponent::print_buffer() {
  std::string hex_output;
  for (size_t i = 0; i < buffer_.size(); i++) {
    char hex_byte[6];  // Enough space for "0xXX "
    snprintf(hex_byte, sizeof(hex_byte), "0x%02X ", buffer_[i]);
    hex_output += hex_byte;

    // Print in groups of 8 for readability
    if ((i + 1) % 8 == 0) {
      hex_output += "\n";
    }
  }

  ESP_LOGV("daikin_ekhhe", "Buffer Contents:\n%s", hex_output.c_str());
}

void DaikinEkhheComponent::parse_dd_packet(std::vector<uint8_t> buffer) {

  // update sensors
  std::map<std::string, float> sensor_values = {
      {A_LOW_WAT_T_PROBE,      (int8_t)buffer[DD_PACKET_A_IDX]},
      {B_UP_WAT_T_PROBE,       (int8_t)buffer[DD_PACKET_B_IDX]},
      {C_DEFROST_T_PROBE,      (int8_t)buffer[DD_PACKET_C_IDX]},
      {D_SUPPLY_AIR_T_PROBE,   (int8_t)buffer[DD_PACKET_D_IDX]},
      {E_EVA_INLET_T_PROBE,    (int8_t)buffer[DD_PACKET_E_IDX]},
      {F_EVA_OUTLET_T_PROBE,   (int8_t)buffer[DD_PACKET_F_IDX]},
      {G_COMP_GAS_T_PROBE,     buffer[DD_PACKET_H_IDX]},
      {H_SOLAR_T_PROBE,        buffer[DD_PACKET_H_IDX]},
      {I_EEV_STEP,             buffer[DD_PACKET_I_IDX]},
  };

  for (const auto &entry : sensor_values) {
    set_sensor_value(entry.first, entry.second);
  }

  // update binary_sensors
  std::map<std::string, bool> binary_sensor_values = {
      {DIG1_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x01)},
      {DIG2_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x02)},
      {DIG3_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x04)},
  };

  for (const auto &entry : binary_sensor_values) {
    DaikinEkhheComponent::set_binary_sensor_value(entry.first, entry.second);
  }

  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_d2_packet(std::vector<uint8_t> buffer) {

  // update numbers
  std::map<std::string, float> number_values = {
      {P1_LOW_WAT_PROBE_HYST,     buffer[D2_PACKET_P1_IDX]},
      {P2_HEAT_ON_DELAY,          buffer[D2_PACKET_P2_IDX]},
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
      // Some variables are bitmasks - clean these up later by parameterizing
      // Mask 1
      {POWER_STATUS,           buffer[D2_PACKET_MASK1_IDX] & 0x01},
      {P39_EEV_MODE,          (buffer[D2_PACKET_MASK1_IDX] & 0x04) >> 2},
      {P13_HW_CIRC_PUMP_MODE, (buffer[D2_PACKET_MASK1_IDX] & 0x10) >> 4},
      // Mask 2
      {P11_DISP_WAT_T_PROBE,   buffer[D2_PACKET_MASK2_IDX] & 0x01},
      {P15_SAFETY_SW_TYPE,    (buffer[D2_PACKET_MASK2_IDX] & 0x02) >> 1},
      {P5_DEFROST_MODE,       (buffer[D2_PACKET_MASK2_IDX] & 0x04) >> 2},
      {P6_EHEATER_DEFROSTING, (buffer[D2_PACKET_MASK2_IDX] & 0x08) >> 3},
      {P33_EEV_CONTROL,       (buffer[D2_PACKET_MASK2_IDX] & 0x10) >> 4},
      // The rest
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

  update_timestamp(buffer[D2_PACKET_HOUR_IDX], buffer[D2_PACKET_MIN_IDX]);

  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_d4_packet(std::vector<uint8_t> buffer) {
  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_c1_packet(std::vector<uint8_t> buffer) {
  print_buffer();
  return;
}

void DaikinEkhheComponent::parse_cc_packet(std::vector<uint8_t> buffer) {

    // update numbers
  std::map<std::string, float> number_values = {
      {P1_LOW_WAT_PROBE_HYST,     buffer[CC_PACKET_P1_IDX]},
      {P2_HEAT_ON_DELAY,          buffer[CC_PACKET_P2_IDX]},
      {P3_ANTL_SET_T,             buffer[CC_PACKET_P3_IDX]},
      {P4_ANTL_DURATION,          buffer[CC_PACKET_P4_IDX]},
      {P7_DEFROST_CYCLE_DELAY,    buffer[CC_PACKET_P7_IDX]},
      {P8_DEFR_START_THRES,       (int8_t)buffer[CC_PACKET_P8_IDX]},
      {P9_DEFR_STOP_THRES,        buffer[CC_PACKET_P9_IDX]},
      {P10_DEFR_MAX_DURATION,     buffer[CC_PACKET_P10_IDX]},
      {P17_HP_START_DELAY_DIG1,   buffer[CC_PACKET_P17_IDX]},
      {P18_LOW_WAT_T_DIG1,        buffer[CC_PACKET_P18_IDX]},
      {P19_LOW_WAT_T_HYST,        buffer[CC_PACKET_P19_IDX]},
      {P20_SOL_DRAIN_THRES,       buffer[CC_PACKET_P20_IDX]},
      {P21_LOW_WAT_T_HP_STOP,     buffer[CC_PACKET_P21_IDX]},
      {P22_UP_WAT_T_EH_STOP,      buffer[CC_PACKET_P22_IDX]},
      {P25_UP_WAT_T_OFFSET,       (int8_t)buffer[CC_PACKET_P25_IDX]},
      {P26_LOW_WAT_T_OFFSET,      (int8_t)buffer[CC_PACKET_P26_IDX]},
      {P27_INLET_T_OFFSET,        (int8_t)buffer[CC_PACKET_P27_IDX]},
      {P28_DEFR_T_OFFSET,         (int8_t)buffer[CC_PACKET_P28_IDX]},
      {P29_ANTL_START_HR,         buffer[CC_PACKET_P29_IDX]},
      {P30_UP_WAT_T_EH_HYST,      buffer[CC_PACKET_P30_IDX]},
      {P31_HP_PERIOD_AUTO,        buffer[CC_PACKET_P31_IDX]},
      {P32_EH_AUTO_TRES,          buffer[CC_PACKET_P32_IDX]},
      {P34_EEV_SH_PERIOD,         buffer[CC_PACKET_P34_IDX]},
      {P35_EEV_SH_SETPOINT,       (int8_t)buffer[CC_PACKET_P35_IDX]},
      {P36_EEV_DSH_SETPOINT,      buffer[CC_PACKET_P36_IDX]},
      {P37_EEV_STEP_DEFR,         buffer[CC_PACKET_P37_IDX]},      
      {P38_EEV_MIN_STEP_AUTO,     buffer[CC_PACKET_P38_IDX]},
      {P40_EEV_INIT_STEP,         buffer[CC_PACKET_P40_IDX]},
      {P41_AKP1_THRES,            (int8_t)buffer[CC_PACKET_P41_IDX]},
      {P42_AKP2_THRES,            (int8_t)buffer[CC_PACKET_P42_IDX]},
      {P43_AKP3_THRES,            (int8_t)buffer[CC_PACKET_P43_IDX]},
      {P44_EEV_KP1_GAIN,          (int8_t)buffer[CC_PACKET_P44_IDX]},
      {P45_EEV_KP2_GAIN,          (int8_t)buffer[CC_PACKET_P45_IDX]},
      {P46_EEV_KP3_GAIN,          (int8_t)buffer[CC_PACKET_P46_IDX]},
      {P47_MAX_INLET_T_HP,        buffer[CC_PACKET_P47_IDX]},
      {P48_MIN_INLET_T_HP,        (int8_t)buffer[CC_PACKET_P48_IDX]},
      {P49_EVA_INLET_THRES,       buffer[CC_PACKET_P49_IDX]},
      {P50_ANTIFREEZE_SET,        buffer[CC_PACKET_P50_IDX]},
      {P51_EVA_HIGH_SET,          buffer[CC_PACKET_P51_IDX]},
      {P52_EVA_LOW_SET,           buffer[CC_PACKET_P52_IDX]},
      // TODO: Add P54

      {ECO_T_TEMPERATURE,         buffer[CC_PACKET_ECO_TTARGET_IDX]},
      {AUTO_T_TEMPERATURE,        buffer[CC_PACKET_AUTO_TTARGET_IDX]},
      {BOOST_T_TEMPERATURE,       buffer[CC_PACKET_BOOST_TTGARGET_IDX]},
      {ELECTRIC_T_TEMPERATURE,    buffer[CC_PACKET_ELECTRIC_TTARGET_IDX]},
  };


  for (const auto &entry : number_values) {
    set_number_value(entry.first, entry.second);
  }

  // update selects
  std::map<std::string, float> select_values = {
      // Some variables are bitmasks - clean these up later by parameterizing
      // Mask 1
      {POWER_STATUS,           buffer[CC_PACKET_MASK1_IDX] & 0x01},
      {P39_EEV_MODE,          (buffer[CC_PACKET_MASK1_IDX] & 0x04) >> 2},
      {P13_HW_CIRC_PUMP_MODE, (buffer[CC_PACKET_MASK1_IDX] & 0x10) >> 4},
      // Mask 2
      {P11_DISP_WAT_T_PROBE,   buffer[CC_PACKET_MASK2_IDX] & 0x01},
      {P15_SAFETY_SW_TYPE,    (buffer[CC_PACKET_MASK2_IDX] & 0x02) >> 1},
      {P5_DEFROST_MODE,       (buffer[CC_PACKET_MASK2_IDX] & 0x04) >> 2},
      {P6_EHEATER_DEFROSTING, (buffer[CC_PACKET_MASK2_IDX] & 0x08) >> 3},
      {P33_EEV_CONTROL,       (buffer[CC_PACKET_MASK2_IDX] & 0x10) >> 4},
      // The rest
      {OPERATIONAL_MODE,       buffer[CC_PACKET_MODE_IDX]},
      {P12_EXT_PUMP_MODE,      buffer[CC_PACKET_P12_IDX]},
      {P14_EVA_BLOWER_TYPE,    buffer[CC_PACKET_P14_IDX]},
      {P16_SOLAR_MODE_INT,     buffer[CC_PACKET_P16_IDX]},
      {P23_PV_MODE_INT,        buffer[CC_PACKET_P23_IDX]},
      {P24_OFF_PEAK_MODE,      buffer[CC_PACKET_P24_IDX]},
  };

  for (const auto &entry : select_values) {
    set_select_value(entry.first, entry.second);
  }

  update_timestamp(buffer[CC_PACKET_HOUR_IDX], buffer[CC_PACKET_MIN_IDX]);

  print_buffer();
  return;
}

void DaikinEkhheComponent::register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor) {
  if (sensor != nullptr) {
    sensors_[sensor_name] = sensor;
    ESP_LOGI(TAG, "Registered Sensor: %s", sensor_name.c_str());
  }
}


void DaikinEkhheComponent::register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor) {
  if (binary_sensor != nullptr) {
    binary_sensors_[sensor_name] = binary_sensor;
    ESP_LOGI(TAG, "Registered Binary Sensor: %s", sensor_name.c_str());
  }
}

void DaikinEkhheComponent::register_number(const std::string &number_name, esphome::number::Number *number) {
  if (number != nullptr) {
    numbers_[number_name] = number;
    ESP_LOGI(TAG, "Registered Number: %s", number_name.c_str());
  }
}

void DaikinEkhheComponent::register_select(const std::string &select_name, select::Select *select) {
  if (select != nullptr) {
      //selects_[select_name] = select;
      selects_[select_name] = static_cast<DaikinEkhheSelect *>(select);
      ESP_LOGI(TAG, "Registered Select: %s", select_name.c_str());
  }
}

void DaikinEkhheComponent::register_timestamp_sensor(text_sensor::TextSensor *sensor) {
    this->timestamp_sensor_ = sensor;
    ESP_LOGI(TAG, "Registered timestamp sensor");
}

void DaikinEkhheComponent::set_sensor_value(const std::string &sensor_name, float value) {
  if (sensors_.find(sensor_name) != sensors_.end()) {
    sensors_[sensor_name]->publish_state(value);
  }
}


void DaikinEkhheComponent::set_binary_sensor_value(const std::string &sensor_name, bool value) {
  if (binary_sensors_.find(sensor_name) != binary_sensors_.end()) {
    binary_sensors_[sensor_name]->publish_state(value);
  }
}


void DaikinEkhheComponent::set_number_value(const std::string &number_name, float value) {
  // This sets a number value that's been gotten from the UART stream, not something that's 
  // set through the API or UI
  if (numbers_.find(number_name) != numbers_.end()) {
    numbers_[number_name]->publish_state(value);
  }
}

void DaikinEkhheComponent::set_select_value(const std::string &select_name, int value) {
  // This sets a select value that's been gotten from the UART stream, not something that's 
  // set through the API or UI
  if (selects_.count(select_name)) {
    DaikinEkhheSelect *select = selects_[select_name];

        // Find the corresponding string option for the numeric value
        for (const auto &entry : select->get_select_mappings()) {
            if (entry.second == value) {                
                // Update ESPHome with the new selected value
                select->publish_state(entry.first);
                return;
            }
        }
  }
}

void DaikinEkhheComponent::update_timestamp(uint8_t hour, uint8_t minute) {
    if (this->timestamp_sensor_ != nullptr) {
        ESPTime now = (*clock).now();

        if (!now.is_valid()) {
            ESP_LOGW(TAG, "Time not available yet. Skipping timestamp update.");
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
        this->timestamp_sensor_->publish_state(timestamp);
        ESP_LOGV("daikin_ekhhe", "Updated timestamp (UTC): %s", timestamp);
    }
}


uint8_t DaikinEkhheComponent::ekhhe_checksum(const std::vector<uint8_t>& data_bytes) {
  // Compute the checksum as (sum of data bytes) mod 256 + 170
  uint16_t sum = std::accumulate(data_bytes.begin(), data_bytes.end()-1, 0);
  return (sum % 256 + 170) & 0xFF;
}

void DaikinEkhheComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Daikin EKHHE:");

    // Log all enabled sensors
    ESP_LOGCONFIG(TAG, "Enabled Sensors:");
    for (const auto &entry : sensors_) {
        if (entry.second != nullptr) {
            ESP_LOGCONFIG(TAG, "  - %s: Last value: %.2f", entry.first.c_str(), entry.second->get_state());
        }
    }

    // needs to be 9600/N/1
    this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);
}

void DaikinEkhheComponent::on_shutdown() {
    ESP_LOGI(TAG, "Shutdown was called");
}

void DaikinEkhheComponent::update() {
    ESP_LOGI(TAG, "Update was called");
}

void DaikinEkhheComponent::set_update_interval(int interval_ms) {
    this->update_interval_ = interval_ms;
    ESP_LOGI(TAG, "Update interval set to %d ms", interval_ms);
}


void DaikinEkhheNumber::control(float value) {

    // Ensure parent component exists
    if (this->parent_ != nullptr) {
        // Use get_name() to determine which UART command to send
        auto name = this->internal_id_;
        ESP_LOGI(TAG, "Changing number value: %s -> %.2f", this->internal_id_.c_str(), value);

        /*
        if (this->get_name() == "low_water_temp_hysteresis") {
            ESP_LOGD(TAG, "")
            this->parent_->send_uart_command(0x01, static_cast<int>(value));  // Example: Command ID 0x01
        } else if (this->get_name() == "heat_on_delay") {
            this->parent_->send_uart_command(0x02, static_cast<int>(value));  // Example: Command ID 0x02
        } else {
            ESP_LOGW(TAG, "No matching UART command for Number: %s", this->get_name().c_str());
        }
        */
    }

    if (false) {
          // Publish new state to ESPHome
          this->publish_state(value);

    }

    //ESP_LOGI(TAG, "Number control called: %.2f", value);

    // Update value in ESPHome
    //this->publish_state(value);

    // Send command via UART (example)
    // Will need to later implement this to control numbers over UART
    //send_uart_command(value);
}

void DaikinEkhheSelect::control(const std::string &value) {
    ESP_LOGI(TAG, "Select control called: %s", value.c_str());

    // Update value in ESPHome
    this->publish_state(value);

    // Send selection over UART
    // send_uart_command(value);  // Uncomment if needed
}


void DaikinEkhheComponent::send_uart_command(const std::string &parameter, int value) {
  /*
    if (last_cc_packet_.empty()) {
        ESP_LOGW("daikin_ekhhe", "No CC packet received yet. Cannot send command.");
        return;
    }

    ESP_LOGI("daikin_ekhhe", "Updating CC packet for %s -> %d", parameter.c_str(), value);

    // ✅ Update the relevant field in the CC packet
    update_cc_packet(parameter, value);

    // ✅ Update timestamp in the CC packet
    time_t now = time(nullptr);
    struct tm *timeinfo = gmtime(&now);
    last_cc_packet_[CC_HOUR_INDEX] = static_cast<uint8_t>(timeinfo->tm_hour);
    last_cc_packet_[CC_MINUTE_INDEX] = static_cast<uint8_t>(timeinfo->tm_min);

    // ✅ Recalculate checksum
    last_cc_packet_.back() = calculate_checksum(last_cc_packet_);

    // ✅ Send the updated CC packet
    this->write_array(last_cc_packet_);
    this->flush();

    ESP_LOGI("daikin_ekhhe", "CC packet sent.");
    */
}


}  // namespace daikin_ekkhe
}  // namespace esphome
