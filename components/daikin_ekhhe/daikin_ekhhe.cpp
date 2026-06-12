#include "daikin_ekhhe.h"
#include "daikin_ekhhe_metadata.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"

#include <algorithm>
#include <numeric>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

void DaikinEkhheComponent::setup() {
  this->known_good_profile_pref_ =
      global_preferences->make_preference<StoredProfileBlob>(KNOWN_GOOD_PROFILE_PREF_KEY, true);
  this->auto_snapshot_pref_ =
      global_preferences->make_preference<StoredProfileBlob>(AUTO_SNAPSHOT_PREF_KEY, true);
  this->load_persistent_profiles_();
  start_uart_cycle();
}

void DaikinEkhheComponent::loop() {
  if (processing_updates_ || uart_tx_active_) {
    return;
  }

  const unsigned long now = millis();
  if (!uart_active_) {
    if (now - last_process_time_ >= update_interval_ || last_process_time_ == 0) {
      start_uart_cycle();
    }
    return;
  }

  while (this->available()) {
    uint8_t byte = read_rx_byte_();
    consume_uart_byte_(byte, millis());
    if (packet_set_complete()) {
      break;
    }
  }

  const unsigned long after_rx = millis();
  check_rx_frame_timeout_(after_rx);
  if (uart_active_ && cycle_synced_ && !packet_set_complete()) {
    if (last_rx_time_ > 0 && (after_rx - last_rx_time_) > kCycleTimeoutMs && !cycle_timeout_logged_) {
      cycle_timeouts_++;
      cycle_timeout_logged_ = true;
      uint8_t missing_mask = kRequiredPacketMask & ~cycle_packet_types_seen_;
      std::string missing = packet_mask_to_string_(missing_mask);
      DAIKIN_WARN(TAG, "Cycle partial: missing=%s bytes=%u packets=%u crc=%u frame=%u",
                  missing.c_str(), cycle_bytes_read_, cycle_packets_seen_, cycle_checksum_errors_,
                  cycle_framing_errors_);
      start_uart_cycle();
      return;
    }
  }

  if (packet_set_complete()) {
    process_packet_set();
  }
}

uint8_t DaikinEkhheComponent::ekhhe_checksum(const std::vector<uint8_t>& data_bytes) {
  uint16_t sum = std::accumulate(data_bytes.begin(), data_bytes.end()-1, 0);
  return (sum % 256 + 170) & 0xFF;
}

void DaikinEkhheComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin EKHHE:");
  ESP_LOGCONFIG(TAG, "  Update interval: %lu ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Continuous RX: %s", YESNO(this->continuous_rx_));
  ESP_LOGCONFIG(TAG, "  TX send calibration: %u ms", this->tx_delay_after_d2_ms_);

  ESP_LOGCONFIG(TAG, "Enabled Sensors:");
  for (const auto &entry : sensors_) {
    if (entry.second != nullptr) {
      ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
    }
  }

  ESP_LOGCONFIG(TAG, "Enabled Binary Sensors:");
  for (const auto &entry : binary_sensors_) {
    if (entry.second != nullptr) {
      ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
    }
  }

  ESP_LOGCONFIG(TAG, "Enabled Numbers:");
  for (const auto &entry : numbers_) {
    if (entry.second != nullptr) {
      ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
    }
  }

  ESP_LOGCONFIG(TAG, "Enabled Selects:");
  for (const auto &entry : selects_) {
    if (entry.second != nullptr) {
      ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
    }
  }

  this->check_uart_settings(9600, 1, esphome::uart::UART_CONFIG_PARITY_NONE, 8);
}

void DaikinEkhheComponent::on_shutdown() {
}

void DaikinEkhheComponent::update() {
}

void DaikinEkhheComponent::set_update_interval(int interval_ms) {
  this->update_interval_ = interval_ms;
}

void DaikinEkhheComponent::set_continuous_rx(bool enabled) {
  this->continuous_rx_ = enabled;
}

void DaikinEkhheComponent::set_tx_delay_after_d2_ms(uint32_t delay_ms) {
  this->tx_delay_after_d2_ms_ = std::min(delay_ms, kMaxTxDelayAfterD2Ms);
  auto it = numbers_.find(TX_SEND_CALIBRATION);
  if (it != numbers_.end() && it->second != nullptr) {
    it->second->publish_state(this->tx_delay_after_d2_ms_);
  }
}


}  // namespace daikin_ekkhe
}  // namespace esphome
