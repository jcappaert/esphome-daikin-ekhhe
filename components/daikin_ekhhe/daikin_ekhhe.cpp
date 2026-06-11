#include "daikin_ekhhe.h"
#include "daikin_ekhhe_metadata.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"


#include <cinttypes>
#include <cstdio>
#include <algorithm>
#include <numeric>
#include <ctime>
#include <cmath>
#include <cstring>

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
    // Don't process RX if we're processing sensor updates or if UART TX is active
    if (processing_updates_ || uart_tx_active_) {
      return;
    }

    // only enable UART every PROCESS_INTERVAL_MS
    unsigned long now = millis();
    if (!uart_active_) {
        if (now - last_process_time_ >= update_interval_ || last_process_time_ == 0) {
            start_uart_cycle();  
        }
        return;
    }

    // Receive bytes
    while (this->available()) {
      uint8_t byte = read_rx_byte_();
      consume_uart_byte_(byte, millis());
      if (packet_set_complete()) {
        break;
      }
    }

    now = millis();
    check_rx_frame_timeout_(now);
    if (uart_active_ && cycle_synced_ && !packet_set_complete()) {
      if (last_rx_time_ > 0 && (now - last_rx_time_) > kCycleTimeoutMs && !cycle_timeout_logged_) {
        cycle_timeouts_++;
        cycle_timeout_logged_ = true;
        uint8_t missing_mask = kRequiredPacketMask & ~cycle_packet_types_seen_;
        std::string missing = packet_mask_to_string_(missing_mask);
        DAIKIN_WARN(TAG, "Cycle partial: missing=%s bytes=%u packets=%u crc=%u frame=%u",
                   missing.c_str(), cycle_bytes_read_, cycle_packets_seen_, cycle_checksum_errors_,
                   cycle_framing_errors_);
      }
    }

    // Check if all packets have been stored and process them
    if (packet_set_complete()) {
      process_packet_set();
    }
}

void DaikinEkhheComponent::handle_complete_packet_(uint8_t packet_type, const uint8_t *data, size_t length) {
  std::vector<uint8_t> packet(data, data + length);
  uint8_t packet_mask = packet_mask_for_start_(packet_type);
  uint8_t flags = 0;
  bool crc_ok = true;
  if ((packet_mask & kChecksumPacketMask) != 0) {
    crc_ok = (ekhhe_checksum(packet) == packet.back());
    if (!crc_ok) {
      flags |= RAW_FRAME_CRC_ERROR;
    }
  }

  store_raw_frame_(packet_type, packet.data(), length, flags);

  if (!crc_ok) {
    if (cycle_synced_) {
      if (latest_packets_.count(packet_type) == 0) {
        cycle_checksum_errors_++;
        cycle_checksum_error_mask_ |= packet_mask;
      }
    }
    return;
  }

  const uint32_t now_ms = millis();
  if (tx_operation_active_() && tx_waiting_for_first_rx_) {
    DAIKIN_DBG(TAG, "TX timing: first_rx_after_tx type=%s dt=%u",
               packet_type_to_string_(packet_type).c_str(), now_ms - tx_sent_ms_);
    tx_waiting_for_first_rx_ = false;
  }

  const uint8_t tx_readback_type = active_tx_readback_packet_type_();
  if (tx_operation_active_() && tx_waiting_for_first_cc_ && packet_type == tx_readback_type) {
    DAIKIN_DBG(TAG, "TX timing: first_readback_after_tx type=%s dt=%u",
               packet_type_to_string_(packet_type).c_str(), now_ms - tx_sent_ms_);
    tx_waiting_for_first_cc_ = false;
  }

  last_frame_profile_ms_ = now_ms;
  last_frame_profile_type_ = packet_type;
  if (packet_type == CC_PACKET_START_BYTE) {
    last_cc_profile_ms_ = now_ms;
  }
  if (packet_type == C2_PACKET_START_BYTE) {
    last_c2_packet_ = packet;
  }
  if (restore_tx_active_() &&
      packet_type == tx_packet_family_spec_(restore_tx_().family).readback_packet_type) {
    check_pending_restore_(packet);
  }
  if (profile_restore_tx_active_() &&
      packet_type == tx_packet_family_spec_(profile_restore_tx_().family).readback_packet_type) {
    check_pending_profile_restore_(packet);
  }
  if (packet_type == D2_PACKET_START_BYTE && time_band_tx_active_()) {
    check_pending_time_band_(packet);
  }
  if (single_field_tx_active_() &&
      packet_type == tx_packet_family_spec_(single_field_tx_().family).readback_packet_type) {
    check_pending_tx_(packet);
  }
  if (packet_type == D2_PACKET_START_BYTE && tx_operation_active_()) {
    size_t d2_index = 0;
    const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
    if (d2_entry != nullptr) {
      if (restore_tx_active_()) {
        schedule_queued_restore_from_d2_(*d2_entry);
      } else if (profile_restore_tx_active_()) {
        schedule_queued_profile_restore_from_d2_(*d2_entry);
      } else if (time_band_tx_active_()) {
        schedule_queued_time_band_from_d2_(*d2_entry);
      } else if (single_field_tx_active_()) {
        schedule_queued_tx_from_d2_(*d2_entry);
      }
    }
  }

  if (!cycle_synced_) {
    cycle_synced_ = true;
    latest_packets_.clear();
    cycle_packets_seen_ = 0;
    cycle_packet_types_seen_ = 0;
    cycle_checksum_errors_ = 0;
    cycle_checksum_error_mask_ = 0;
    cycle_framing_errors_ = 0;
    cycle_framing_error_start_ = 0;
    cycle_bytes_read_ = length;
  }

  if (cycle_synced_ && latest_packets_.count(packet_type) > 0) {
    return;
  }

  // Store only the latest version of each valid packet
  latest_packets_[packet_type] = packet;
  cycle_packets_seen_++;
  cycle_packet_types_seen_ |= packet_mask;
}

uint8_t DaikinEkhheComponent::read_rx_byte_() {
  cycle_bytes_read_++;
  return this->read();
}

void DaikinEkhheComponent::reset_rx_frame_() {
  rx_frame_active_ = false;
  rx_frame_start_ms_ = 0;
  rx_frame_type_ = 0;
  rx_frame_expected_len_ = 0;
  rx_frame_offset_ = 0;
}

void DaikinEkhheComponent::consume_uart_byte_(uint8_t byte, uint32_t now_ms) {
  last_rx_time_ = now_ms;

  if (!rx_frame_active_) {
    auto size_it = PACKET_SIZES.find(byte);
    if (size_it == PACKET_SIZES.end()) {
      return;
    }

    rx_frame_active_ = true;
    rx_frame_start_ms_ = now_ms;
    rx_frame_type_ = byte;
    rx_frame_expected_len_ = size_it->second;
    rx_frame_offset_ = 1;
    rx_frame_buffer_[0] = byte;

    if (rx_frame_expected_len_ == rx_frame_offset_) {
      handle_complete_packet_(rx_frame_type_, rx_frame_buffer_, rx_frame_expected_len_);
      reset_rx_frame_();
    }
    return;
  }

  if (rx_frame_offset_ < kRawFrameMaxLen) {
    rx_frame_buffer_[rx_frame_offset_] = byte;
  }
  rx_frame_offset_++;

  if (rx_frame_offset_ >= rx_frame_expected_len_) {
    handle_complete_packet_(rx_frame_type_, rx_frame_buffer_, rx_frame_expected_len_);
    reset_rx_frame_();
  }
}

void DaikinEkhheComponent::check_rx_frame_timeout_(uint32_t now_ms) {
  if (!rx_frame_active_ || (now_ms - rx_frame_start_ms_) < kFrameReadTimeoutMs) {
    return;
  }

  if (cycle_synced_) {
    cycle_framing_errors_++;
    cycle_framing_error_start_ = rx_frame_type_;
  }
  reset_rx_frame_();
}

void DaikinEkhheComponent::store_raw_frame_(uint8_t packet_type, const uint8_t *data, size_t length, uint8_t flags) {
  if (length > kRawFrameMaxLen) {
    length = kRawFrameMaxLen;
    flags |= RAW_FRAME_TRUNCATED;
  }

  if (raw_frame_count_ == kRawFrameBufferSize) {
    raw_frame_head_ = (raw_frame_head_ + 1) % kRawFrameBufferSize;
    raw_frame_count_--;
  }

  size_t index = (raw_frame_head_ + raw_frame_count_) % kRawFrameBufferSize;
  RawFrameEntry &entry = raw_frames_[index];
  raw_frame_seq_++;
  entry.seq = raw_frame_seq_;
  entry.timestamp_ms = millis();
  entry.packet_type = packet_type;
  entry.length = static_cast<uint8_t>(length);
  entry.flags = flags;
  std::memcpy(entry.data, data, length);
  raw_frame_count_++;
}

void DaikinEkhheComponent::reset_cycle_stats_() {
  cycle_start_ms_ = millis();
  last_rx_time_ = cycle_start_ms_;
  cycle_bytes_read_ = 0;
  cycle_packets_seen_ = 0;
  cycle_timeouts_ = 0;
  cycle_checksum_errors_ = 0;
  cycle_checksum_error_mask_ = 0;
  cycle_framing_errors_ = 0;
  cycle_framing_error_start_ = 0;
  cycle_packet_types_seen_ = 0;
  cycle_timeout_logged_ = false;
  cycle_publish_allowed_ = true;
  cycle_synced_ = false;
}

uint8_t DaikinEkhheComponent::packet_mask_for_start_(uint8_t start_byte) const {
  switch (start_byte) {
    case DD_PACKET_START_BYTE:
      return kPacketMaskDD;
    case D2_PACKET_START_BYTE:
      return kPacketMaskD2;
    case D4_PACKET_START_BYTE:
      return kPacketMaskD4;
    case C1_PACKET_START_BYTE:
      return kPacketMaskC1;
    case C2_PACKET_START_BYTE:
      return kPacketMaskC2;
    case CC_PACKET_START_BYTE:
      return kPacketMaskCC;
    default:
      return 0;
  }
}

std::string DaikinEkhheComponent::packet_mask_to_string_(uint8_t mask) const {
  std::string out;
  if (mask & kPacketMaskDD) out += "DD,";
  if (mask & kPacketMaskD2) out += "D2,";
  if (mask & kPacketMaskD4) out += "D4,";
  if (mask & kPacketMaskC1) out += "C1,";
  if (mask & kPacketMaskC2) out += "C2,";
  if (mask & kPacketMaskCC) out += "CC,";
  if (out.empty()) {
    return "none";
  }
  out.pop_back();
  return out;
}

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

std::string DaikinEkhheComponent::packet_type_to_string_(uint8_t packet_type) const {
  switch (packet_type) {
    case DD_PACKET_START_BYTE:
      return "DD";
    case D2_PACKET_START_BYTE:
      return "D2";
    case D4_PACKET_START_BYTE:
      return "D4";
    case C1_PACKET_START_BYTE:
      return "C1";
    case C2_PACKET_START_BYTE:
      return "C2";
    case CC_PACKET_START_BYTE:
      return "CC";
    case CD_PACKET_START_BYTE:
      return "CD";
    default:
      return "latest";
  }
}

const DaikinEkhheComponent::RawFrameEntry *DaikinEkhheComponent::find_latest_frame_by_type_(uint8_t packet_type,
                                                                                            size_t &index,
                                                                                            bool require_ok) const {
  if (raw_frame_count_ == 0) {
    return nullptr;
  }
  if (packet_type == 0) {
    if (!require_ok) {
      index = (raw_frame_head_ + raw_frame_count_ - 1) % kRawFrameBufferSize;
      return &raw_frames_[index];
    }
    for (size_t i = 0; i < raw_frame_count_; ++i) {
      size_t idx = (raw_frame_head_ + raw_frame_count_ - 1 - i) % kRawFrameBufferSize;
      const RawFrameEntry &entry = raw_frames_[idx];
      if (is_frame_ok_(entry)) {
        index = idx;
        return &entry;
      }
    }
    return nullptr;
  }
  for (size_t i = 0; i < raw_frame_count_; ++i) {
    size_t idx = (raw_frame_head_ + raw_frame_count_ - 1 - i) % kRawFrameBufferSize;
    const RawFrameEntry &entry = raw_frames_[idx];
    if (entry.packet_type == packet_type) {
      if (require_ok && !is_frame_ok_(entry)) {
        continue;
      }
      index = idx;
      return &entry;
    }
  }
  return nullptr;
}

const DaikinEkhheComponent::RawFrameEntry *DaikinEkhheComponent::find_latest_dd_frame_(size_t &index) const {
  if (raw_frame_count_ == 0) {
    return nullptr;
  }
  for (size_t i = 0; i < raw_frame_count_; ++i) {
    size_t idx = (raw_frame_head_ + raw_frame_count_ - 1 - i) % kRawFrameBufferSize;
    const RawFrameEntry &entry = raw_frames_[idx];
    if (entry.packet_type != DD_PACKET_START_BYTE) {
      continue;
    }
    if (entry.length < 2) {
      continue;
    }
    if ((entry.flags & RAW_FRAME_TRUNCATED) != 0) {
      continue;
    }
    index = idx;
    return &entry;
  }
  return nullptr;
}

bool DaikinEkhheComponent::is_frame_ok_(const RawFrameEntry &entry) const {
  return entry.flags == 0;
}

bool DaikinEkhheComponent::field_matches_target_(const std::vector<uint8_t> &buffer, uint8_t index, uint8_t value,
                                                 uint8_t bit_position, uint8_t bit_width) const {
  if (index >= buffer.size()) {
    return false;
  }
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return buffer[index] == value;
  }
  const uint8_t mask = field_value_mask(index, bit_position, bit_width);
  const uint8_t field_value = extract_field_value(buffer, index, bit_position, bit_width);
  return field_value == (value & mask);
}

bool DaikinEkhheComponent::time_band_matches_packet_(const std::vector<uint8_t> &buffer, bool d2_packet,
                                                     uint8_t flag, uint8_t start_hour, uint8_t start_minute,
                                                     uint8_t end_hour, uint8_t end_minute,
                                                     uint8_t mode) const {
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

  return buffer.size() > mode_idx &&
         buffer[flag_idx] == flag &&
         buffer[start_hour_idx] == start_hour &&
         buffer[start_minute_idx] == start_minute &&
         buffer[end_hour_idx] == end_hour &&
         buffer[end_minute_idx] == end_minute &&
         buffer[mode_idx] == mode;
}

bool DaikinEkhheComponent::validate_time_band_request_(uint8_t flag, uint8_t start_hour,
                                                       uint8_t start_minute, uint8_t end_hour,
                                                       uint8_t end_minute, uint8_t mode,
                                                       std::string &reason) const {
  if (flag != kTimeBandApplyFlag && flag != kTimeBandClearFlag) {
    char msg[64];
    snprintf(msg, sizeof(msg), "unsupported flag 0x%02X", flag);
    reason = msg;
    return false;
  }
  if (start_hour > 23 || end_hour > 23) {
    char msg[96];
    snprintf(msg, sizeof(msg), "hour out of range start=%u end=%u", start_hour, end_hour);
    reason = msg;
    return false;
  }
  if (start_minute > 59 || end_minute > 59) {
    char msg[96];
    snprintf(msg, sizeof(msg), "minute out of range start=%u end=%u", start_minute, end_minute);
    reason = msg;
    return false;
  }
  if (mode > kTimeBandMaxMode) {
    char msg[64];
    snprintf(msg, sizeof(msg), "mode %u out of range 0..%u", mode, kTimeBandMaxMode);
    reason = msg;
    return false;
  }

  if (flag == kTimeBandApplyFlag) {
    const uint16_t start_total = static_cast<uint16_t>(start_hour) * 60 + start_minute;
    const uint16_t end_total = static_cast<uint16_t>(end_hour) * 60 + end_minute;
    if (end_total == start_total) {
      reason = "end time must be after start time; use clear for 00:00 -> 00:00";
      return false;
    }
    if (end_total < start_total) {
      reason = "overnight time bands crossing midnight are not supported";
      return false;
    }
  } else if (start_hour != 0 || start_minute != 0 || end_hour != 0 || end_minute != 0) {
    reason = "clear command must use 00:00 -> 00:00";
    return false;
  }

  return true;
}

uint8_t DaikinEkhheComponent::tx_readback_index_(TxPacketFamily family, uint8_t write_index,
                                                 uint8_t bit_position) const {
  if (family == TxPacketFamily::EXTENDED) {
    return write_index;
  }

  const ManagedFieldSpec *field = find_managed_field_by_cc(write_index, bit_position);
  return field != nullptr ? field->readback.index : write_index;
}

uint8_t DaikinEkhheComponent::tx_readback_bit_position_(TxPacketFamily family, uint8_t write_index,
                                                        uint8_t bit_position) const {
  if (family == TxPacketFamily::EXTENDED || bit_position == BIT_POSITION_NO_BITMASK) {
    return bit_position;
  }

  const ManagedFieldSpec *field = find_managed_field_by_cc(write_index, bit_position);
  return field != nullptr ? field->readback.bit_position : bit_position;
}

uint8_t DaikinEkhheComponent::tx_readback_bit_width_(TxPacketFamily family, uint8_t write_index,
                                                     uint8_t bit_position, uint8_t bit_width) const {
  if (family == TxPacketFamily::EXTENDED || bit_position == BIT_POSITION_NO_BITMASK) {
    return bit_width;
  }

  const ManagedFieldSpec *field = find_managed_field_by_cc(write_index, bit_position);
  return field != nullptr ? field->readback.bit_width : bit_width;
}

DaikinEkhheComponent::TxOperationKind DaikinEkhheComponent::active_tx_operation_kind_() const {
  if (single_field_tx_active_()) {
    return TxOperationKind::SINGLE_FIELD;
  }
  if (restore_tx_active_()) {
    return TxOperationKind::RESTORE_DEFAULTS;
  }
  if (profile_restore_tx_active_()) {
    return TxOperationKind::PROFILE_RESTORE;
  }
  if (time_band_tx_active_()) {
    return TxOperationKind::TIME_BAND;
  }
  return TxOperationKind::NONE;
}

const char *DaikinEkhheComponent::tx_operation_kind_label_(TxOperationKind kind) const {
  switch (kind) {
    case TxOperationKind::SINGLE_FIELD:
      return "single_field";
    case TxOperationKind::RESTORE_DEFAULTS:
      return "restore_defaults";
    case TxOperationKind::PROFILE_RESTORE:
      return "profile_restore";
    case TxOperationKind::TIME_BAND:
      return "time_band";
    case TxOperationKind::NONE:
    default:
      return "none";
  }
}

DaikinEkhheComponent::TxPacketFamily DaikinEkhheComponent::active_tx_operation_family_() const {
  switch (active_tx_operation_kind_()) {
    case TxOperationKind::SINGLE_FIELD:
      return single_field_tx_().family;
    case TxOperationKind::RESTORE_DEFAULTS:
      return restore_tx_().family;
    case TxOperationKind::PROFILE_RESTORE:
      return profile_restore_tx_().family;
    case TxOperationKind::TIME_BAND:
    case TxOperationKind::NONE:
    default:
      return TxPacketFamily::MAIN;
  }
}

uint8_t DaikinEkhheComponent::active_tx_readback_packet_type_() const {
  TxOperationKind kind = active_tx_operation_kind_();
  if (kind == TxOperationKind::NONE || kind == TxOperationKind::TIME_BAND) {
    return D2_PACKET_START_BYTE;
  }
  return tx_packet_family_spec_(active_tx_operation_family_()).readback_packet_type;
}

bool DaikinEkhheComponent::tx_operation_active_() const {
  return active_tx_operation_kind_() != TxOperationKind::NONE;
}

bool DaikinEkhheComponent::single_field_tx_active_() const {
  return tx_operation_.kind == TxOperationKind::SINGLE_FIELD;
}

const DaikinEkhheComponent::SingleFieldTxPayload &DaikinEkhheComponent::single_field_tx_() const {
  return tx_operation_.single_field;
}

bool DaikinEkhheComponent::single_field_tx_matches_(TxPacketFamily family, uint8_t index,
                                                    uint8_t bit_position) const {
  const auto &target = single_field_tx_();
  return single_field_tx_active_() && target.family == family && target.index == index &&
         target.bit_position == bit_position;
}

bool DaikinEkhheComponent::restore_tx_active_() const {
  return tx_operation_.kind == TxOperationKind::RESTORE_DEFAULTS;
}

DaikinEkhheComponent::RestoreTxPayload &DaikinEkhheComponent::restore_tx_() {
  return tx_operation_.restore;
}

const DaikinEkhheComponent::RestoreTxPayload &DaikinEkhheComponent::restore_tx_() const {
  return tx_operation_.restore;
}

bool DaikinEkhheComponent::profile_restore_tx_active_() const {
  return tx_operation_.kind == TxOperationKind::PROFILE_RESTORE;
}

DaikinEkhheComponent::ProfileRestoreTxPayload &DaikinEkhheComponent::profile_restore_tx_() {
  return tx_operation_.profile_restore;
}

const DaikinEkhheComponent::ProfileRestoreTxPayload &DaikinEkhheComponent::profile_restore_tx_() const {
  return tx_operation_.profile_restore;
}

bool DaikinEkhheComponent::time_band_tx_active_() const {
  return tx_operation_.kind == TxOperationKind::TIME_BAND;
}

DaikinEkhheComponent::TimeBandTxPayload &DaikinEkhheComponent::time_band_tx_() {
  return tx_operation_.time_band;
}

const DaikinEkhheComponent::TimeBandTxPayload &DaikinEkhheComponent::time_band_tx_() const {
  return tx_operation_.time_band;
}

bool DaikinEkhheComponent::single_field_tx_busy_() const {
  return single_field_tx_active_() || queued_tx_.active || queued_tx_.scheduled ||
         tx_ui_sync_active_(TxOperationKind::SINGLE_FIELD);
}

bool DaikinEkhheComponent::restore_tx_busy_() const {
  return restore_tx_active_() || tx_ui_sync_active_(TxOperationKind::RESTORE_DEFAULTS);
}

bool DaikinEkhheComponent::profile_restore_tx_busy_() const {
  return profile_restore_tx_active_() || tx_ui_sync_active_(TxOperationKind::PROFILE_RESTORE);
}

bool DaikinEkhheComponent::time_band_tx_sending_() const {
  return time_band_tx_active_() || queued_time_band_tx_.active || queued_time_band_tx_.scheduled;
}

bool DaikinEkhheComponent::time_band_tx_busy_() const {
  return time_band_tx_sending_() || tx_ui_sync_active_(TxOperationKind::TIME_BAND);
}

bool DaikinEkhheComponent::any_write_busy_() const {
  return single_field_tx_busy_() || restore_tx_busy_() || profile_restore_tx_busy_() ||
         time_band_tx_busy_();
}

bool DaikinEkhheComponent::any_tx_or_ui_sync_active_() const {
  return tx_operation_active_() || tx_ui_sync_active_();
}

bool DaikinEkhheComponent::tx_ui_sync_active_() const {
  return tx_ui_sync_.kind != TxOperationKind::NONE;
}

bool DaikinEkhheComponent::tx_ui_sync_active_(TxOperationKind kind) const {
  return tx_ui_sync_.kind == kind;
}

void DaikinEkhheComponent::reset_tx_ui_sync_() {
  tx_ui_sync_ = TxUiSyncState{};
}

void DaikinEkhheComponent::start_single_field_ui_sync_(const SingleFieldTxPayload &target) {
  reset_tx_ui_sync_();
  tx_ui_sync_.kind = TxOperationKind::SINGLE_FIELD;
  tx_ui_sync_.single_field = target;
}

void DaikinEkhheComponent::start_restore_ui_sync_() {
  reset_tx_ui_sync_();
  tx_ui_sync_.kind = TxOperationKind::RESTORE_DEFAULTS;
}

void DaikinEkhheComponent::start_profile_restore_ui_sync_(bool known_good) {
  reset_tx_ui_sync_();
  tx_ui_sync_.kind = TxOperationKind::PROFILE_RESTORE;
  tx_ui_sync_.known_good = known_good;
}

void DaikinEkhheComponent::start_time_band_ui_sync_(const TimeBandTxPayload &target) {
  reset_tx_ui_sync_();
  tx_ui_sync_.kind = TxOperationKind::TIME_BAND;
  tx_ui_sync_.time_band = target;
}

void DaikinEkhheComponent::clear_tx_wait_markers_() {
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
}

void DaikinEkhheComponent::reset_tx_operation_() {
  tx_operation_.kind = TxOperationKind::NONE;
  tx_operation_.single_field = SingleFieldTxPayload{};
  tx_operation_.restore = RestoreTxPayload{};
  tx_operation_.profile_restore = ProfileRestoreTxPayload{};
  tx_operation_.time_band = TimeBandTxPayload{};
  tx_operation_.attempts_sent = 0;
  tx_operation_.last_attempt_d2_seq = 0;
}

void DaikinEkhheComponent::reset_single_field_tx_lifecycle_(bool clear_ui_sync) {
  reset_tx_operation_();
  queued_tx_.active = false;
  queued_tx_.scheduled = false;
  if (clear_ui_sync && tx_ui_sync_active_(TxOperationKind::SINGLE_FIELD)) {
    reset_tx_ui_sync_();
  }
}

void DaikinEkhheComponent::reset_restore_tx_lifecycle_(bool clear_ui_sync) {
  reset_pending_restore_();
  reset_queued_restore_();
  if (clear_ui_sync && tx_ui_sync_active_(TxOperationKind::RESTORE_DEFAULTS)) {
    reset_tx_ui_sync_();
  }
}

void DaikinEkhheComponent::reset_profile_restore_tx_lifecycle_(bool clear_ui_sync) {
  reset_pending_profile_restore_();
  reset_queued_profile_restore_();
  if (clear_ui_sync && tx_ui_sync_active_(TxOperationKind::PROFILE_RESTORE)) {
    reset_tx_ui_sync_();
  }
}

void DaikinEkhheComponent::reset_time_band_tx_lifecycle_(bool clear_ui_sync) {
  reset_pending_time_band_();
  reset_queued_time_band_();
  if (clear_ui_sync && tx_ui_sync_active_(TxOperationKind::TIME_BAND)) {
    reset_tx_ui_sync_();
  }
}

void DaikinEkhheComponent::reset_tx_lifecycle_(TxOperationKind kind, bool clear_ui_sync) {
  switch (kind) {
    case TxOperationKind::SINGLE_FIELD:
      reset_single_field_tx_lifecycle_(clear_ui_sync);
      break;
    case TxOperationKind::RESTORE_DEFAULTS:
      reset_restore_tx_lifecycle_(clear_ui_sync);
      break;
    case TxOperationKind::PROFILE_RESTORE:
      reset_profile_restore_tx_lifecycle_(clear_ui_sync);
      break;
    case TxOperationKind::TIME_BAND:
      reset_time_band_tx_lifecycle_(clear_ui_sync);
      break;
    case TxOperationKind::NONE:
    default:
      break;
  }
}

void DaikinEkhheComponent::reset_pending_restore_() {
  reset_tx_operation_();
}

void DaikinEkhheComponent::reset_queued_restore_() {
  queued_restore_.active = false;
  queued_restore_.scheduled = false;
  queued_restore_.family = TxPacketFamily::MAIN;
  queued_restore_.anchor_ms = 0;
  queued_restore_.anchor_seq = 0;
}

void DaikinEkhheComponent::reset_pending_profile_restore_() {
  reset_tx_operation_();
}

void DaikinEkhheComponent::reset_queued_profile_restore_() {
  queued_profile_restore_.active = false;
  queued_profile_restore_.scheduled = false;
  queued_profile_restore_.known_good = false;
  queued_profile_restore_.family = TxPacketFamily::MAIN;
  queued_profile_restore_.anchor_ms = 0;
  queued_profile_restore_.anchor_seq = 0;
}

void DaikinEkhheComponent::reset_pending_time_band_() {
  reset_tx_operation_();
}

void DaikinEkhheComponent::reset_queued_time_band_() {
  queued_time_band_tx_.active = false;
  queued_time_band_tx_.scheduled = false;
  queued_time_band_tx_.anchor_ms = 0;
  queued_time_band_tx_.anchor_seq = 0;
}

bool DaikinEkhheComponent::defer_single_field_tx_(TxPacketFamily family, uint8_t index, uint8_t value,
                                                  uint8_t bit_position, uint8_t bit_width) {
  const auto &spec = tx_packet_family_spec_(family);
  for (auto &deferred : deferred_user_txs_) {
    if (deferred.family == family && deferred.index == index && deferred.bit_position == bit_position) {
      deferred.value = value;
      deferred.bit_width = bit_width;
      const uint8_t log_width = field_log_width(index, bit_position, bit_width);
      ESP_LOGI(TAG, "TX deferred write updated: family=%s index=%u value=0x%02X bit=%u width=%u queued=%u",
               spec.label, index, value, bit_position, log_width,
               static_cast<unsigned>(deferred_user_txs_.size()));
      return true;
    }
  }

  if (deferred_user_txs_.size() >= kDeferredTxMax) {
    const uint8_t log_width = field_log_width(index, bit_position, bit_width);
    DAIKIN_WARN(TAG, "TX deferred write queue full, dropping write: family=%s index=%u value=0x%02X bit=%u width=%u",
                spec.label, index, value, bit_position, log_width);
    return false;
  }

  DeferredTx deferred;
  deferred.family = family;
  deferred.index = index;
  deferred.value = value;
  deferred.bit_position = bit_position;
  deferred.bit_width = bit_width;
  deferred_user_txs_.push_back(deferred);

  const uint8_t log_width = field_log_width(index, bit_position, bit_width);
  ESP_LOGI(TAG, "TX deferred write queued: family=%s index=%u value=0x%02X bit=%u width=%u queued=%u",
           spec.label, index, value, bit_position, log_width, static_cast<unsigned>(deferred_user_txs_.size()));

  if (!uart_active_ && !uart_tx_active_) {
    start_uart_cycle();
  }

  return true;
}

bool DaikinEkhheComponent::has_deferred_user_tx_(TxPacketFamily family, uint8_t index, uint8_t bit_position) const {
  for (const auto &deferred : deferred_user_txs_) {
    if (deferred.family == family && deferred.index == index && deferred.bit_position == bit_position) {
      return true;
    }
  }
  return false;
}

void DaikinEkhheComponent::flush_deferred_user_tx_() {
  if (deferred_user_txs_.empty()) {
    return;
  }
  if (any_write_busy_()) {
    return;
  }

  DeferredTx deferred = deferred_user_txs_.front();
  deferred_user_txs_.erase(deferred_user_txs_.begin());

  const auto &spec = tx_packet_family_spec_(deferred.family);
  const uint8_t log_width = field_log_width(deferred.index, deferred.bit_position, deferred.bit_width);
  ESP_LOGI(TAG, "TX deferred write starting: family=%s index=%u value=0x%02X bit=%u width=%u remaining=%u",
           spec.label, deferred.index, deferred.value, deferred.bit_position, log_width,
           static_cast<unsigned>(deferred_user_txs_.size()));

  send_uart_command_(deferred.family, deferred.index, deferred.value, deferred.bit_position, deferred.bit_width);
}

bool DaikinEkhheComponent::schedule_tx_after_d2_(TxOperationKind kind, const RawFrameEntry &d2_entry) {
  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms =
      d2_age_ms >= tx_delay_after_d2_ms_ ? 0 : (tx_delay_after_d2_ms_ - d2_age_ms);
  uint32_t generation = 0;

  switch (kind) {
    case TxOperationKind::SINGLE_FIELD: {
      if (!single_field_tx_active_() || queued_tx_.scheduled) {
        return false;
      }
      const auto &target = single_field_tx_();
      generation = queued_tx_.generation;
      queued_tx_.active = true;
      queued_tx_.scheduled = true;
      queued_tx_.family = target.family;
      queued_tx_.index = target.index;
      queued_tx_.value = target.value;
      queued_tx_.bit_position = target.bit_position;
      queued_tx_.bit_width = target.bit_width;
      queued_tx_.anchor_ms = d2_entry.timestamp_ms;
      queued_tx_.anchor_seq = d2_entry.seq;

      const auto &spec = tx_packet_family_spec_(target.family);
      DAIKIN_DBG(TAG, "TX scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
                 spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_tx_.request_ms);
      break;
    }
    case TxOperationKind::RESTORE_DEFAULTS: {
      if (!restore_tx_active_() || queued_restore_.scheduled) {
        return false;
      }
      generation = queued_restore_.generation;
      const TxPacketFamily family = restore_tx_().family;
      queued_restore_.active = true;
      queued_restore_.scheduled = true;
      queued_restore_.family = family;
      queued_restore_.anchor_ms = d2_entry.timestamp_ms;
      queued_restore_.anchor_seq = d2_entry.seq;

      const auto &spec = tx_packet_family_spec_(family);
      DAIKIN_DBG(TAG, "Restore defaults scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
                 spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_restore_.request_ms);
      break;
    }
    case TxOperationKind::PROFILE_RESTORE: {
      if (!profile_restore_tx_active_() || queued_profile_restore_.scheduled) {
        return false;
      }
      const auto &target = profile_restore_tx_();
      generation = queued_profile_restore_.generation;
      const TxPacketFamily family = target.family;
      queued_profile_restore_.active = true;
      queued_profile_restore_.scheduled = true;
      queued_profile_restore_.known_good = target.known_good;
      queued_profile_restore_.family = family;
      queued_profile_restore_.anchor_ms = d2_entry.timestamp_ms;
      queued_profile_restore_.anchor_seq = d2_entry.seq;

      const auto &spec = tx_packet_family_spec_(family);
      DAIKIN_DBG(TAG, "%s restore scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
                 queued_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot",
                 spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_profile_restore_.request_ms);
      break;
    }
    case TxOperationKind::TIME_BAND: {
      if (!time_band_tx_active_() || queued_time_band_tx_.scheduled) {
        return false;
      }
      generation = queued_time_band_tx_.generation;
      queued_time_band_tx_.active = true;
      queued_time_band_tx_.scheduled = true;
      queued_time_band_tx_.anchor_ms = d2_entry.timestamp_ms;
      queued_time_band_tx_.anchor_seq = d2_entry.seq;

      DAIKIN_DBG(TAG, "Time-band scheduling: d2_seq=%u d2_age=%u delay=%u request_age=%u",
                 d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_time_band_tx_.request_ms);
      break;
    }
    case TxOperationKind::NONE:
    default:
      return false;
  }

  set_timeout(delay_ms, [this, kind, generation]() { this->run_tx_after_d2_(kind, generation); });
  return true;
}

bool DaikinEkhheComponent::tx_d2_schedule_current_(TxOperationKind kind, uint32_t generation) const {
  switch (kind) {
    case TxOperationKind::SINGLE_FIELD:
      return queued_tx_.active && queued_tx_.generation == generation;
    case TxOperationKind::RESTORE_DEFAULTS:
      return queued_restore_.active && queued_restore_.generation == generation;
    case TxOperationKind::PROFILE_RESTORE:
      return queued_profile_restore_.active && queued_profile_restore_.generation == generation;
    case TxOperationKind::TIME_BAND:
      return queued_time_band_tx_.active && queued_time_band_tx_.generation == generation;
    case TxOperationKind::NONE:
    default:
      return false;
  }
}

void DaikinEkhheComponent::run_tx_after_d2_(TxOperationKind kind, uint32_t generation) {
  if (!tx_d2_schedule_current_(kind, generation)) {
    return;
  }

  switch (kind) {
    case TxOperationKind::SINGLE_FIELD: {
      TxPacketFamily family = queued_tx_.family;
      const auto &spec = tx_packet_family_spec_(family);
      uint8_t index = queued_tx_.index;
      uint8_t value = queued_tx_.value;
      uint8_t bit_position = queued_tx_.bit_position;
      uint8_t bit_width = queued_tx_.bit_width;
      uint32_t anchor_seq = queued_tx_.anchor_seq;
      uint32_t anchor_ms = queued_tx_.anchor_ms;

      queued_tx_.active = false;
      queued_tx_.scheduled = false;

      std::vector<uint8_t> base_packet = family == TxPacketFamily::EXTENDED ? last_c1_packet_ : last_cc_packet_;
      auto latest_base = latest_packets_.find(spec.base_packet_type);
      if (latest_base != latest_packets_.end()) {
        base_packet = latest_base->second;
      }

      tx_operation_.attempts_sent++;
      tx_operation_.last_attempt_d2_seq = anchor_seq;

      DAIKIN_DBG(TAG,
                 "TX scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
                 spec.label, anchor_seq, millis() - anchor_ms, tx_operation_.attempts_sent, kTxMaxRepeats,
                 latest_base != latest_packets_.end());
      send_uart_tx_packet_(family, base_packet, true, index, value, bit_position, bit_width);
      return;
    }
    case TxOperationKind::RESTORE_DEFAULTS: {
      const TxPacketFamily family = queued_restore_.family;
      const auto &spec = tx_packet_family_spec_(family);
      const bool extended = family == TxPacketFamily::EXTENDED;
      const uint32_t anchor_seq = queued_restore_.anchor_seq;
      const uint32_t anchor_ms = queued_restore_.anchor_ms;
      queued_restore_.active = false;
      queued_restore_.scheduled = false;

      std::vector<uint8_t> base_packet = extended ? last_c1_packet_ : last_cc_packet_;
      auto latest_base = latest_packets_.find(spec.base_packet_type);
      if (latest_base != latest_packets_.end()) {
        base_packet = latest_base->second;
      }
      if (base_packet.empty()) {
        DAIKIN_WARN(TAG, "Restore defaults aborted: no %s base packet is available.", spec.label);
        reset_tx_lifecycle_(TxOperationKind::RESTORE_DEFAULTS, false);
        return;
      }

      std::vector<uint8_t> packet = base_packet;
      apply_restore_defaults_to_packet(packet, extended);
      auto &target = restore_tx_();
      tx_operation_.attempts_sent++;
      if (extended) {
        target.extended_write_sent = true;
      } else {
        target.main_write_sent = true;
      }
      tx_operation_.last_attempt_d2_seq = anchor_seq;

      DAIKIN_DBG(TAG,
                 "Restore defaults scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
                 spec.label, anchor_seq, millis() - anchor_ms, tx_operation_.attempts_sent, kTxMaxRepeats,
                 latest_base != latest_packets_.end());
      send_restore_defaults_packet_(family, base_packet, packet);
      return;
    }
    case TxOperationKind::PROFILE_RESTORE: {
      const bool known_good = queued_profile_restore_.known_good;
      const TxPacketFamily family = queued_profile_restore_.family;
      const auto &spec = tx_packet_family_spec_(family);
      const bool extended = family == TxPacketFamily::EXTENDED;
      const ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
      const uint32_t anchor_seq = queued_profile_restore_.anchor_seq;
      const uint32_t anchor_ms = queued_profile_restore_.anchor_ms;
      queued_profile_restore_.active = false;
      queued_profile_restore_.scheduled = false;

      const uint8_t profile_length = extended ? profile.extended_length : profile.main_length;
      const uint8_t *profile_data = extended ? profile.extended_data : profile.main_data;
      if (!profile.valid || profile_length == 0) {
        DAIKIN_WARN(TAG, "%s restore aborted: stored profile became unavailable.",
                    known_good ? "Known-good profile" : "Auto snapshot");
        reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, false);
        return;
      }

      std::vector<uint8_t> base_packet = extended ? last_c1_packet_ : last_cc_packet_;
      auto latest_base = latest_packets_.find(spec.base_packet_type);
      if (latest_base != latest_packets_.end()) {
        base_packet = latest_base->second;
      }
      if (base_packet.empty()) {
        DAIKIN_WARN(TAG, "%s restore aborted: no %s base packet is available.",
                    known_good ? "Known-good profile" : "Auto snapshot", spec.label);
        reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, false);
        return;
      }

      std::vector<uint8_t> packet = base_packet;
      merge_profile_managed_fields(extended, packet, profile_data, profile_length);
      auto &target = profile_restore_tx_();
      tx_operation_.attempts_sent++;
      if (extended) {
        target.extended_write_sent = true;
      } else {
        target.main_write_sent = true;
      }
      tx_operation_.last_attempt_d2_seq = anchor_seq;

      DAIKIN_DBG(TAG,
                 "%s restore scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
                 known_good ? "Known-good profile" : "Auto snapshot", spec.label,
                 anchor_seq, millis() - anchor_ms, tx_operation_.attempts_sent, kTxMaxRepeats,
                 latest_base != latest_packets_.end());
      send_profile_restore_packet_(family, base_packet, packet, known_good);
      return;
    }
    case TxOperationKind::TIME_BAND: {
      const uint32_t anchor_seq = queued_time_band_tx_.anchor_seq;
      const uint32_t anchor_ms = queued_time_band_tx_.anchor_ms;
      queued_time_band_tx_.active = false;
      queued_time_band_tx_.scheduled = false;

      std::vector<uint8_t> base_packet = last_cc_packet_;
      auto latest_cc = latest_packets_.find(CC_PACKET_START_BYTE);
      if (latest_cc != latest_packets_.end()) {
        base_packet = latest_cc->second;
      }

      tx_operation_.attempts_sent++;
      tx_operation_.last_attempt_d2_seq = anchor_seq;

      DAIKIN_DBG(TAG,
                 "Time-band scheduling: sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_cc=%u",
                 anchor_seq, millis() - anchor_ms, tx_operation_.attempts_sent, kTxMaxRepeats,
                 latest_cc != latest_packets_.end());
      send_time_band_packet_(base_packet);
      return;
    }
    case TxOperationKind::NONE:
    default:
      return;
  }
}

void DaikinEkhheComponent::schedule_queued_tx_from_d2_(const RawFrameEntry &d2_entry) {
  schedule_tx_after_d2_(TxOperationKind::SINGLE_FIELD, d2_entry);
}

void DaikinEkhheComponent::schedule_queued_restore_from_d2_(const RawFrameEntry &d2_entry) {
  schedule_tx_after_d2_(TxOperationKind::RESTORE_DEFAULTS, d2_entry);
}

void DaikinEkhheComponent::schedule_queued_profile_restore_from_d2_(const RawFrameEntry &d2_entry) {
  schedule_tx_after_d2_(TxOperationKind::PROFILE_RESTORE, d2_entry);
}

void DaikinEkhheComponent::schedule_queued_time_band_from_d2_(const RawFrameEntry &d2_entry) {
  schedule_tx_after_d2_(TxOperationKind::TIME_BAND, d2_entry);
}

const DaikinEkhheComponent::TxPacketFamilySpec &DaikinEkhheComponent::tx_packet_family_spec_(
    TxPacketFamily family) const {
  static const TxPacketFamilySpec main_family = {
      TxPacketFamily::MAIN,
      CC_PACKET_START_BYTE,
      CD_PACKET_START_BYTE,
      D2_PACKET_START_BYTE,
      CD_PACKET_SIZE,
      "main",
  };
  static const TxPacketFamilySpec extended_family = {
      TxPacketFamily::EXTENDED,
      C1_PACKET_START_BYTE,
      C2_PACKET_START_BYTE,
      D4_PACKET_START_BYTE,
      C2_PACKET_SIZE,
      "extended",
  };

  return family == TxPacketFamily::EXTENDED ? extended_family : main_family;
}

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

void DaikinEkhheComponent::publish_profile_statuses_() {
  publish_profile_status_(true);
  publish_profile_status_(false);
}

void DaikinEkhheComponent::publish_profile_status_(bool known_good) {
  text_sensor::TextSensor *sensor =
      known_good ? known_good_profile_status_sensor_ : auto_snapshot_status_sensor_;
  if (sensor == nullptr) {
    return;
  }
  sensor->publish_state(format_profile_status_(known_good));
}

std::string DaikinEkhheComponent::format_profile_status_(bool known_good) const {
  const ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
  if (!profile.valid || profile.main_length == 0 || profile.extended_length == 0) {
    return "EMPTY";
  }

  char ts_buf[32];
  if (profile.saved_at_epoch > 0) {
    time_t saved = static_cast<time_t>(profile.saved_at_epoch);
    struct tm *utc = gmtime(&saved);
    if (utc != nullptr && strftime(ts_buf, sizeof(ts_buf), "%Y-%m-%dT%H:%M:%SZ", utc) > 0) {
      char msg[128];
      snprintf(msg, sizeof(msg), "VALID main=%u extended=%u saved=%s",
               static_cast<unsigned>(profile.main_length),
               static_cast<unsigned>(profile.extended_length), ts_buf);
      return msg;
    }
  }

  char msg[96];
  snprintf(msg, sizeof(msg), "VALID main=%u extended=%u",
           static_cast<unsigned>(profile.main_length),
           static_cast<unsigned>(profile.extended_length));
  return msg;
}

void DaikinEkhheComponent::load_persistent_profiles_() {
  auto load_slot = [this](bool known_good) {
    ESPPreferenceObject &pref = known_good ? known_good_profile_pref_ : auto_snapshot_pref_;
    ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
    StoredProfileBlob blob{};
    profile = ProfileState{};

    if (!pref.load(&blob)) {
      return;
    }
    if (blob.magic != PROFILE_MAGIC || blob.version != PROFILE_VERSION || blob.main_length == 0 ||
        blob.main_length > kRawFrameMaxLen || blob.extended_length == 0 ||
        blob.extended_length > kRawFrameMaxLen) {
      return;
    }
    if (blob.main_data[0] != CC_PACKET_START_BYTE || blob.extended_data[0] != C1_PACKET_START_BYTE) {
      return;
    }
    std::vector<uint8_t> main_packet(blob.main_data, blob.main_data + blob.main_length);
    std::vector<uint8_t> extended_packet(blob.extended_data, blob.extended_data + blob.extended_length);
    if (main_packet.empty() || ekhhe_checksum(main_packet) != main_packet.back() ||
        extended_packet.empty() || ekhhe_checksum(extended_packet) != extended_packet.back()) {
      return;
    }
    if (profile_data_hash(blob.main_data, blob.main_length) != blob.main_data_hash ||
        profile_data_hash(blob.extended_data, blob.extended_length) != blob.extended_data_hash) {
      return;
    }

    profile.valid = true;
    profile.main_length = blob.main_length;
    profile.extended_length = blob.extended_length;
    profile.saved_at_epoch = blob.saved_at_epoch;
    profile.main_data_hash = blob.main_data_hash;
    profile.extended_data_hash = blob.extended_data_hash;
    std::memcpy(profile.main_data, blob.main_data, blob.main_length);
    std::memcpy(profile.extended_data, blob.extended_data, blob.extended_length);
  };

  load_slot(true);
  load_slot(false);
  publish_profile_statuses_();
}

bool DaikinEkhheComponent::capture_current_packet_(uint8_t packet_type,
                                                   const std::vector<uint8_t> &cache,
                                                   std::vector<uint8_t> &packet) const {
  if (!cache.empty()) {
    packet = cache;
    return true;
  }
  size_t index = 0;
  const RawFrameEntry *entry = find_latest_frame_by_type_(packet_type, index, true);
  if (entry == nullptr || entry->length == 0) {
    packet.clear();
    return false;
  }
  packet.assign(entry->data, entry->data + entry->length);
  return true;
}

bool DaikinEkhheComponent::capture_current_profile_packets_(std::vector<uint8_t> &main_packet,
                                                            std::vector<uint8_t> &extended_packet) const {
  if (!capture_current_packet_(CC_PACKET_START_BYTE, last_cc_packet_, main_packet)) {
    return false;
  }
  if (!capture_current_packet_(C1_PACKET_START_BYTE, last_c1_packet_, extended_packet)) {
    main_packet.clear();
    return false;
  }
  return true;
}

bool DaikinEkhheComponent::save_profile_(bool known_good, const std::vector<uint8_t> &main_packet,
                                         const std::vector<uint8_t> &extended_packet) {
  if (main_packet.empty() || main_packet.size() > kRawFrameMaxLen ||
      main_packet[0] != CC_PACKET_START_BYTE) {
    DAIKIN_WARN(TAG, "%s save aborted: invalid CC packet",
                known_good ? "Known-good profile" : "Auto snapshot");
    return false;
  }
  if (extended_packet.empty() || extended_packet.size() > kRawFrameMaxLen ||
      extended_packet[0] != C1_PACKET_START_BYTE) {
    DAIKIN_WARN(TAG, "%s save aborted: invalid C1 packet",
                known_good ? "Known-good profile" : "Auto snapshot");
    return false;
  }

  StoredProfileBlob blob{};
  blob.magic = PROFILE_MAGIC;
  blob.version = PROFILE_VERSION;
  blob.main_length = static_cast<uint8_t>(main_packet.size());
  blob.extended_length = static_cast<uint8_t>(extended_packet.size());
  if (this->clock != nullptr) {
    ESPTime now = this->clock->utcnow();
    if (now.is_valid() && now.timestamp > 0) {
      blob.saved_at_epoch = static_cast<uint32_t>(now.timestamp);
    }
  }
  std::memcpy(blob.main_data, main_packet.data(), main_packet.size());
  std::memcpy(blob.extended_data, extended_packet.data(), extended_packet.size());
  blob.main_data_hash = profile_data_hash(blob.main_data, blob.main_length);
  blob.extended_data_hash = profile_data_hash(blob.extended_data, blob.extended_length);

  ESPPreferenceObject &pref = known_good ? known_good_profile_pref_ : auto_snapshot_pref_;
  if (!pref.save(&blob) || !global_preferences->sync()) {
    DAIKIN_WARN(TAG, "%s save failed", known_good ? "Known-good profile" : "Auto snapshot");
    return false;
  }

  ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
  profile.valid = true;
  profile.main_length = blob.main_length;
  profile.extended_length = blob.extended_length;
  profile.saved_at_epoch = blob.saved_at_epoch;
  profile.main_data_hash = blob.main_data_hash;
  profile.extended_data_hash = blob.extended_data_hash;
  std::memcpy(profile.main_data, blob.main_data, blob.main_length);
  std::memcpy(profile.extended_data, blob.extended_data, blob.extended_length);
  if (!known_good) {
    auto_snapshot_last_write_ms_ = millis();
  }
  publish_profile_statuses_();

  DAIKIN_WARN(TAG, "%s saved: main=%u extended=%u",
              known_good ? "Known-good profile" : "Auto snapshot",
              static_cast<unsigned>(profile.main_length),
              static_cast<unsigned>(profile.extended_length));
  return true;
}

bool DaikinEkhheComponent::auto_save_snapshot_if_needed_() {
  std::vector<uint8_t> main_packet;
  std::vector<uint8_t> extended_packet;
  if (!capture_current_profile_packets_(main_packet, extended_packet)) {
    DAIKIN_WARN(TAG, "Auto snapshot skipped: no valid CC/C1 packet pair available.");
    return false;
  }
  if (auto_snapshot_.valid &&
      managed_fields_equal_between_packets(false, auto_snapshot_.main_data, auto_snapshot_.main_length,
                                           main_packet.data(), main_packet.size()) &&
      managed_fields_equal_between_packets(true, auto_snapshot_.extended_data, auto_snapshot_.extended_length,
                                           extended_packet.data(), extended_packet.size())) {
    DAIKIN_DBG(TAG, "Auto snapshot skipped: managed fields unchanged.");
    return false;
  }
  const uint32_t now_ms = millis();
  if (auto_snapshot_.valid && auto_snapshot_last_write_ms_ != 0 &&
      (now_ms - auto_snapshot_last_write_ms_) < kAutoSnapshotCooldownMs) {
    DAIKIN_DBG(TAG, "Auto snapshot skipped: cooldown=%u remaining=%u",
               kAutoSnapshotCooldownMs,
               kAutoSnapshotCooldownMs - (now_ms - auto_snapshot_last_write_ms_));
    return false;
  }
  return save_profile_(false, main_packet, extended_packet);
}

void DaikinEkhheComponent::restore_profile_(bool known_good) {
  const ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
  if (!profile.valid || profile.main_length == 0 || profile.extended_length == 0) {
    DAIKIN_WARN(TAG, "%s restore requested but no profile is stored.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }
  if (last_cc_packet_.empty()) {
    DAIKIN_WARN(TAG, "%s restore requested before any CC packet was captured.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }
  if (last_c1_packet_.empty()) {
    DAIKIN_WARN(TAG, "%s restore requested before any C1 packet was captured.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }
  if (profile_restore_tx_busy_()) {
    DAIKIN_WARN(TAG, "%s restore already in progress.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }
  if (restore_tx_busy_() || single_field_tx_busy_() || time_band_tx_busy_()) {
    DAIKIN_WARN(TAG, "%s restore blocked: another write is currently active.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }

  tx_request_ms_ = millis();
  reset_tx_operation_();
  tx_operation_.kind = TxOperationKind::PROFILE_RESTORE;
  auto &target = profile_restore_tx_();
  target.known_good = known_good;
  target.family = TxPacketFamily::MAIN;
  tx_operation_.attempts_sent = 0;
  tx_operation_.last_attempt_d2_seq = 0;
  reset_tx_ui_sync_();

  reset_queued_profile_restore_();
  queued_profile_restore_.active = true;
  queued_profile_restore_.scheduled = false;
  queued_profile_restore_.known_good = known_good;
  queued_profile_restore_.family = TxPacketFamily::MAIN;
  queued_profile_restore_.generation++;
  queued_profile_restore_.request_ms = tx_request_ms_;

  clear_tx_wait_markers_();

  DAIKIN_WARN(TAG, "%s restore requested: managed_fields=%u main=%u extended=%u",
              known_good ? "Known-good profile" : "Auto snapshot",
              static_cast<unsigned>(PROFILE_MANAGED_FIELD_COUNT),
              static_cast<unsigned>(PROFILE_MANAGED_MAIN_FIELD_COUNT),
              static_cast<unsigned>(PROFILE_MANAGED_EXTENDED_FIELD_COUNT));

  if (!uart_active_ && !uart_tx_active_) {
    start_uart_cycle();
  }

  size_t d2_index = 0;
  const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
  if (d2_entry != nullptr) {
    uint32_t d2_age_ms = millis() - d2_entry->timestamp_ms;
    if (d2_age_ms <= tx_delay_after_d2_ms_) {
      schedule_queued_profile_restore_from_d2_(*d2_entry);
      return;
    }
  }

  DAIKIN_DBG(TAG, "%s restore scheduling: waiting_for_next_d2",
             known_good ? "Known-good profile" : "Auto snapshot");
}

bool DaikinEkhheComponent::packet_set_complete() {
    return latest_packets_.count(DD_PACKET_START_BYTE) &&
           latest_packets_.count(D2_PACKET_START_BYTE) &&
           latest_packets_.count(D4_PACKET_START_BYTE) &&
           latest_packets_.count(C1_PACKET_START_BYTE) &&
           latest_packets_.count(CC_PACKET_START_BYTE);
}

void DaikinEkhheComponent::start_uart_cycle() {
    uart_active_ = true;
    latest_packets_.clear();
    reset_rx_frame_();
    reset_cycle_stats_();
}

void DaikinEkhheComponent::process_packet_set() {
  processing_updates_ = true;

  bool has_required = (cycle_packet_types_seen_ & kRequiredPacketMask) == kRequiredPacketMask;
  cycle_publish_allowed_ = has_required;

  // Assign stored packets to last known packet values
  last_dd_packet_ = latest_packets_[DD_PACKET_START_BYTE];
  last_d2_packet_ = latest_packets_[D2_PACKET_START_BYTE];
  last_d4_packet_ = latest_packets_[D4_PACKET_START_BYTE];
  last_c1_packet_ = latest_packets_[C1_PACKET_START_BYTE];
  last_cc_packet_ = latest_packets_[CC_PACKET_START_BYTE];

  parse_dd_packet(last_dd_packet_);
  //parse_d2_packet(last_d2_packet);  // Don't process D2 since all info is in CC as well
  parse_d4_packet(last_d4_packet_);
  parse_c1_packet(last_c1_packet_);
  parse_cc_packet(last_cc_packet_);

  if (!has_required) {
    uint8_t missing_mask = kRequiredPacketMask & ~cycle_packet_types_seen_;
    std::string missing = packet_mask_to_string_(missing_mask);
    DAIKIN_WARN(TAG, "Cycle partial: missing=%s bytes=%u packets=%u crc=%u frame=%u",
               missing.c_str(), cycle_bytes_read_, cycle_packets_seen_, cycle_checksum_errors_,
               cycle_framing_errors_);
  }
  if (cycle_framing_errors_ > 0 && !cycle_timeout_logged_) {
    if (has_required) {
      DAIKIN_DBG(TAG, "Cycle resync: framing=%u last_start=0x%02X bytes=%u packets=%u",
                 cycle_framing_errors_, cycle_framing_error_start_, cycle_bytes_read_,
                 cycle_packets_seen_);
    } else {
      DAIKIN_WARN(TAG, "Cycle warn: framing=%u last_start=0x%02X bytes=%u packets=%u",
                  cycle_framing_errors_, cycle_framing_error_start_, cycle_bytes_read_,
                  cycle_packets_seen_);
    }
  }
  if (!has_required && cycle_checksum_errors_ > 0) {
    std::string types = packet_mask_to_string_(cycle_packet_types_seen_);
    std::string checksum_types = packet_mask_to_string_(cycle_checksum_error_mask_);
    DAIKIN_ERROR(TAG, "Cycle error: checksum=%u checksum_types=%s types=%s bytes=%u packets=%u",
                 cycle_checksum_errors_, checksum_types.c_str(), types.c_str(),
                 cycle_bytes_read_, cycle_packets_seen_);
  }

  update_dd_b1_bit_sensors_();
  flush_deferred_user_tx_();

  // Reset UART cycle
  processing_updates_ = false;
  last_process_time_ = millis();
  if (any_tx_or_ui_sync_active_() || continuous_rx_) {
    start_uart_cycle();
  } else {
    uart_active_ = false;
  }
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

  return;
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

uint8_t DaikinEkhheComponent::ekhhe_checksum(const std::vector<uint8_t>& data_bytes) {
  // Compute the checksum as (sum of data bytes) mod 256 + 170
  uint16_t sum = std::accumulate(data_bytes.begin(), data_bytes.end()-1, 0);
  return (sum % 256 + 170) & 0xFF;
}

void DaikinEkhheComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Daikin EKHHE:");
    ESP_LOGCONFIG(TAG, "  Update interval: %lu ms", this->update_interval_);
    ESP_LOGCONFIG(TAG, "  Continuous RX: %s", YESNO(this->continuous_rx_));
    ESP_LOGCONFIG(TAG, "  TX send calibration: %u ms", this->tx_delay_after_d2_ms_);

    // Log all enabled sensors
    ESP_LOGCONFIG(TAG, "Enabled Sensors:");
    for (const auto &entry : sensors_) {
        if (entry.second != nullptr) {
            ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
        }
    }

    ESP_LOGCONFIG(TAG, "Enabled Binary Sensors:");
    // binary sensors
    for (const auto &entry : binary_sensors_) {
        if (entry.second != nullptr) {
              ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
        }
    }

    // numbers
    ESP_LOGCONFIG(TAG, "Enabled Numbers:");
    // binary sensors
    for (const auto &entry : numbers_) {
        if (entry.second != nullptr) {
              ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
        }
    }

    // selects
       // numbers
       ESP_LOGCONFIG(TAG, "Enabled Selects:");
       // binary sensors
       for (const auto &entry : selects_) {
           if (entry.second != nullptr) {
                 ESP_LOGCONFIG(TAG, "  - %s", entry.first.c_str());
           }
       }

    // needs to be 9600/N/1
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

void DaikinEkhheComponent::save_known_good_profile() {
  std::vector<uint8_t> main_packet;
  std::vector<uint8_t> extended_packet;
  if (!capture_current_profile_packets_(main_packet, extended_packet)) {
    DAIKIN_WARN(TAG, "Known-good profile save requested before any valid CC/C1 packet pair was captured.");
    return;
  }
  save_profile_(true, main_packet, extended_packet);
}

void DaikinEkhheComponent::restore_known_good_profile() {
  restore_profile_(true);
}

void DaikinEkhheComponent::restore_auto_snapshot() {
  restore_profile_(false);
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

void DaikinEkhheComponent::request_time_band_tx_(uint8_t flag, uint8_t start_hour, uint8_t start_minute,
                                                 uint8_t end_hour, uint8_t end_minute, uint8_t mode) {
  if (last_cc_packet_.empty()) {
    DAIKIN_WARN(TAG, "Time-band command requested before any CC packet was captured.");
    return;
  }
  std::string validation_error;
  if (!validate_time_band_request_(flag, start_hour, start_minute, end_hour, end_minute, mode,
                                   validation_error)) {
    DAIKIN_WARN(TAG, "Time-band command rejected: %s", validation_error.c_str());
    return;
  }
  DAIKIN_DBG(TAG, "Time-band command validated: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
             flag, start_hour, start_minute, end_hour, end_minute, mode);
  if (time_band_tx_sending_()) {
    DAIKIN_WARN(TAG, "Time-band command already in progress.");
    return;
  }
  if (single_field_tx_busy_() || restore_tx_busy_() || profile_restore_tx_busy_() ||
      tx_ui_sync_active_(TxOperationKind::TIME_BAND)) {
    DAIKIN_WARN(TAG, "Time-band command blocked: another write is currently active.");
    return;
  }

  tx_request_ms_ = millis();
  reset_tx_operation_();
  tx_operation_.kind = TxOperationKind::TIME_BAND;
  auto &target = time_band_tx_();
  target.flag = flag;
  target.start_hour = start_hour;
  target.start_minute = start_minute;
  target.end_hour = end_hour;
  target.end_minute = end_minute;
  target.mode = mode;
  reset_tx_ui_sync_();

  reset_queued_time_band_();
  queued_time_band_tx_.active = true;
  queued_time_band_tx_.scheduled = false;
  queued_time_band_tx_.generation++;
  queued_time_band_tx_.request_ms = tx_request_ms_;

  clear_tx_wait_markers_();

  ESP_LOGI(TAG, "Time-band command requested: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
           flag, start_hour, start_minute, end_hour, end_minute, mode);

  if (!uart_active_ && !uart_tx_active_) {
    start_uart_cycle();
  }

  size_t d2_index = 0;
  const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
  if (d2_entry != nullptr) {
    uint32_t d2_age_ms = millis() - d2_entry->timestamp_ms;
    if (d2_age_ms <= tx_delay_after_d2_ms_) {
      schedule_queued_time_band_from_d2_(*d2_entry);
      return;
    }
  }

  DAIKIN_DBG(TAG, "Time-band scheduling: waiting_for_next_d2 flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
             flag, start_hour, start_minute, end_hour, end_minute, mode);
}

void DaikinEkhheComponent::restore_default_settings() {
  if (last_cc_packet_.empty()) {
    DAIKIN_WARN(TAG, "Restore defaults requested before any CC packet was captured.");
    return;
  }
  if (last_c1_packet_.empty()) {
    DAIKIN_WARN(TAG, "Restore defaults requested before any C1 packet was captured.");
    return;
  }
  if (profile_restore_tx_busy_()) {
    DAIKIN_WARN(TAG, "Restore defaults blocked: a profile restore is currently active.");
    return;
  }
  if (restore_tx_busy_()) {
    DAIKIN_WARN(TAG, "Restore defaults already in progress.");
    return;
  }
  if (single_field_tx_busy_() || time_band_tx_busy_()) {
    DAIKIN_WARN(TAG, "Restore defaults blocked: another write is currently active.");
    return;
  }

  tx_request_ms_ = millis();
  reset_tx_operation_();
  tx_operation_.kind = TxOperationKind::RESTORE_DEFAULTS;
  restore_tx_().family = TxPacketFamily::MAIN;
  tx_operation_.attempts_sent = 0;
  tx_operation_.last_attempt_d2_seq = 0;
  reset_tx_ui_sync_();

  reset_queued_restore_();
  queued_restore_.active = true;
  queued_restore_.scheduled = false;
  queued_restore_.family = TxPacketFamily::MAIN;
  queued_restore_.generation++;
  queued_restore_.request_ms = tx_request_ms_;

  clear_tx_wait_markers_();

  DAIKIN_WARN(TAG, "Restore defaults requested: fields=%u main=%u extended=%u",
              static_cast<unsigned>(RESTORE_DEFAULT_FIELD_COUNT),
              static_cast<unsigned>(RESTORE_DEFAULT_MAIN_FIELD_COUNT),
              static_cast<unsigned>(RESTORE_DEFAULT_EXTENDED_FIELD_COUNT));

  if (!uart_active_ && !uart_tx_active_) {
    start_uart_cycle();
  }

  size_t d2_index = 0;
  const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
  if (d2_entry != nullptr) {
    uint32_t d2_age_ms = millis() - d2_entry->timestamp_ms;
    if (d2_age_ms <= tx_delay_after_d2_ms_) {
      schedule_queued_restore_from_d2_(*d2_entry);
      return;
    }
  }

DAIKIN_DBG(TAG, "Restore defaults scheduling: waiting_for_next_d2");
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

void DaikinEkhheComponent::send_prebuilt_cd_packet_(TxPacketFamily family, const std::vector<uint8_t> &command,
                                                    TxPacketKind kind,
                                                    uint8_t index, uint8_t value, uint8_t bit_position,
                                                    uint8_t bit_width,
                                                    uint8_t attempts_sent) {
  const auto &spec = tx_packet_family_spec_(family);
  const std::string tx_type = packet_type_to_string_(spec.write_packet_type);
  if (command.empty()) {
    DAIKIN_WARN(TAG, "%s packet empty, cannot send.", tx_type.c_str());
    return;
  }
  if (command.size() != spec.packet_size) {
    DAIKIN_WARN(TAG, "%s packet length %u invalid, cannot send.",
                tx_type.c_str(), static_cast<unsigned>(command.size()));
    return;
  }

  const unsigned long time_since_last_rx = millis() - last_rx_time_;

  uart_active_ = false;
  uart_tx_active_ = true;

  size_t base_index = 0;
  const RawFrameEntry *base_entry = find_latest_frame_by_type_(spec.base_packet_type, base_index, true);
  uint32_t base_age_ms = base_entry != nullptr ? (millis() - base_entry->timestamp_ms) : 0;
  uint32_t tx_start_ms = millis();

  size_t dd_index = 0;
  size_t c1_index = 0;
  size_t d2_index = 0;
  const RawFrameEntry *latest_dd_entry = find_latest_frame_by_type_(DD_PACKET_START_BYTE, dd_index, true);
  const RawFrameEntry *latest_c1_entry = find_latest_frame_by_type_(C1_PACKET_START_BYTE, c1_index, true);
  const RawFrameEntry *latest_d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);

  if (kind != TxPacketKind::SNAPSHOT) {
    auto age_or_zero = [tx_start_ms](const RawFrameEntry *entry) -> uint32_t {
      return entry != nullptr ? (tx_start_ms - entry->timestamp_ms) : 0;
    };

    DAIKIN_DBG(TAG, "TX start: family=%s tx=%s base=%s readback=%s since_dd=%u since_c1=%u since_d2=%u since_base=%u base_age=%u len=%u",
               spec.label, tx_type.c_str(), packet_type_to_string_(spec.base_packet_type).c_str(),
               packet_type_to_string_(spec.readback_packet_type).c_str(),
               age_or_zero(latest_dd_entry), age_or_zero(latest_c1_entry), age_or_zero(latest_d2_entry),
               age_or_zero(base_entry), base_age_ms, static_cast<unsigned>(command.size()));
  }

  this->write_array(command);
  this->flush();
  tx_sent_ms_ = millis();
  const bool track_tx = kind != TxPacketKind::SNAPSHOT;
  tx_waiting_for_first_rx_ = track_tx;
  tx_waiting_for_first_cc_ = track_tx;

  if (kind == TxPacketKind::SINGLE_FIELD) {
    const uint8_t log_width = field_log_width(index, bit_position, bit_width);
    ESP_LOGI(TAG, "TX %s sent: family=%s base=%s readback=%s index=%u value=0x%02X bit=%u width=%u len=%u",
             tx_type.c_str(), spec.label, packet_type_to_string_(spec.base_packet_type).c_str(),
             packet_type_to_string_(spec.readback_packet_type).c_str(), index, value, bit_position, log_width,
             static_cast<unsigned>(command.size()));
    DAIKIN_DBG(TAG, "TX timing: request_to_send=%u since_last_rx=%u base_age=%u",
               tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_age_ms);
  } else if (kind == TxPacketKind::PROFILE_RESTORE) {
    const unsigned field_count = family == TxPacketFamily::EXTENDED
                                     ? static_cast<unsigned>(PROFILE_MANAGED_EXTENDED_FIELD_COUNT)
                                     : static_cast<unsigned>(PROFILE_MANAGED_MAIN_FIELD_COUNT);
    ESP_LOGI(TAG, "TX profile restore sent: family=%s fields=%u len=%u attempt=%u/%u",
             spec.label, field_count, static_cast<unsigned>(command.size()), attempts_sent, kTxMaxRepeats);
    DAIKIN_DBG(TAG, "TX timing: profile_restore_request_to_send=%u since_last_rx=%u base_age=%u",
               tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_age_ms);
  } else if (kind == TxPacketKind::RESTORE_DEFAULTS) {
    const unsigned field_count = family == TxPacketFamily::EXTENDED
                                     ? static_cast<unsigned>(RESTORE_DEFAULT_EXTENDED_FIELD_COUNT)
                                     : static_cast<unsigned>(RESTORE_DEFAULT_MAIN_FIELD_COUNT);
    ESP_LOGI(TAG, "TX restore defaults sent: family=%s fields=%u attempt=%u/%u len=%u",
             spec.label, field_count, attempts_sent, kTxMaxRepeats,
             static_cast<unsigned>(command.size()));
    DAIKIN_DBG(TAG, "TX timing: restore_request_to_send=%u since_last_rx=%u base_age=%u",
               tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_age_ms);
  } else if (kind == TxPacketKind::TIME_BAND) {
    const auto &target = time_band_tx_();
    ESP_LOGI(TAG, "TX time band sent: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u attempt=%u/%u len=%u",
             target.flag, target.start_hour, target.start_minute, target.end_hour, target.end_minute, target.mode,
             attempts_sent, kTxMaxRepeats, static_cast<unsigned>(command.size()));
    DAIKIN_DBG(TAG, "TX timing: time_band_request_to_send=%u since_last_rx=%u base_age=%u",
               tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_age_ms);
  } else {
    ESP_LOGI(TAG, "TX %s sent: snapshot len=%u", tx_type.c_str(), static_cast<unsigned>(command.size()));
    DAIKIN_DBG(TAG, "TX timing: snapshot_send since_last_rx=%u base_age=%u",
               time_since_last_rx, base_age_ms);
  }

  set_timeout(10, [this]() {
    uart_tx_active_ = false;
    start_uart_cycle();
  });
}

bool DaikinEkhheComponent::validate_outbound_cd_packet_(TxPacketFamily family,
                                                        const std::vector<uint8_t> &base_packet,
                                                        const std::vector<uint8_t> &command,
                                                        TxPacketKind kind, uint8_t index,
                                                        uint8_t value, uint8_t bit_position,
                                                        uint8_t bit_width,
                                                        std::string &reason) {
  const auto &spec = tx_packet_family_spec_(family);
  if (command.empty() || base_packet.empty()) {
    reason = "base or command packet empty";
    return false;
  }
  if (command.size() != base_packet.size()) {
    char msg[96];
    snprintf(msg, sizeof(msg), "size mismatch base=%u command=%u",
             static_cast<unsigned>(base_packet.size()), static_cast<unsigned>(command.size()));
    reason = msg;
    return false;
  }
  if (command.size() != spec.packet_size) {
    char msg[96];
    snprintf(msg, sizeof(msg), "packet size %u is not expected %u",
             static_cast<unsigned>(command.size()), static_cast<unsigned>(spec.packet_size));
    reason = msg;
    return false;
  }
  if (base_packet[0] != spec.base_packet_type) {
    char msg[64];
    snprintf(msg, sizeof(msg), "base start byte 0x%02X is not %s",
             base_packet[0], packet_type_to_string_(spec.base_packet_type).c_str());
    reason = msg;
    return false;
  }
  if (command[0] != spec.write_packet_type) {
    char msg[64];
    snprintf(msg, sizeof(msg), "command start byte 0x%02X is not %s",
             command[0], packet_type_to_string_(spec.write_packet_type).c_str());
    reason = msg;
    return false;
  }

  const uint8_t expected_checksum = ekhhe_checksum(command);
  if (command.back() != expected_checksum) {
    char msg[96];
    snprintf(msg, sizeof(msg), "checksum mismatch expected=0x%02X actual=0x%02X",
             expected_checksum, command.back());
    reason = msg;
    return false;
  }

  if (kind == TxPacketKind::SNAPSHOT) {
    return true;
  }

  std::vector<uint8_t> expected = base_packet;
  expected[0] = spec.write_packet_type;

  if (kind == TxPacketKind::SINGLE_FIELD) {
    if (index >= expected.size()) {
      char msg[64];
      snprintf(msg, sizeof(msg), "target index %u out of range", index);
      reason = msg;
      return false;
    }
    if (bit_position != BIT_POSITION_NO_BITMASK) {
      apply_field_value(expected, index, bit_position, effective_bit_width(bit_position, bit_width), value);
    } else {
      expected[index] = value;
    }
  } else if (kind == TxPacketKind::RESTORE_DEFAULTS) {
    apply_restore_defaults_to_packet(expected, family == TxPacketFamily::EXTENDED);
  } else if (kind == TxPacketKind::TIME_BAND) {
    if (family != TxPacketFamily::MAIN) {
      reason = "time-band commands are only valid for the main packet family";
      return false;
    }
    auto is_allowed_time_band_diff = [&command](size_t offset) -> bool {
      return offset == CC_PACKET_TIME_BAND_FLAG_IDX ||
             offset == CC_PACKET_TIME_BAND_START_HOUR_IDX ||
             offset == CC_PACKET_TIME_BAND_START_MINUTE_IDX ||
             offset == CC_PACKET_TIME_BAND_END_HOUR_IDX ||
             offset == CC_PACKET_TIME_BAND_END_MINUTE_IDX ||
             offset == CC_PACKET_TIME_BAND_MODE_IDX ||
             offset == command.size() - 1;
    };

    for (size_t i = 0; i < expected.size(); ++i) {
      if (expected[i] == command[i]) {
        continue;
      }
      if (!is_allowed_time_band_diff(i)) {
        char msg[128];
        snprintf(msg, sizeof(msg), "time-band command changed unexpected byte %u expected=0x%02X actual=0x%02X",
                 static_cast<unsigned>(i), expected[i], command[i]);
        reason = msg;
        return false;
      }
    }
    return true;
  }

  expected.back() = ekhhe_checksum(expected);

  if (expected == command) {
    return true;
  }

  size_t diff_count = 0;
  size_t first_diff = 0;
  for (size_t i = 0; i < expected.size(); ++i) {
    if (expected[i] == command[i]) {
      continue;
    }
    if (diff_count == 0) {
      first_diff = i;
    }
    diff_count++;
  }

  char msg[192];
  snprintf(msg, sizeof(msg),
           "unexpected packet diff count=%u first_byte=%u expected=0x%02X actual=0x%02X",
           static_cast<unsigned>(diff_count), static_cast<unsigned>(first_diff),
           expected[first_diff], command[first_diff]);
  reason = msg;
  return false;
}

void DaikinEkhheComponent::send_uart_tx_packet_(TxPacketFamily family, const std::vector<uint8_t> &base_packet,
                                                bool apply_change, uint8_t index, uint8_t value,
                                                uint8_t bit_position, uint8_t bit_width) {
  const auto &spec = tx_packet_family_spec_(family);
  const std::string base_type = packet_type_to_string_(spec.base_packet_type);
  if (base_packet.empty()) {
    DAIKIN_WARN(TAG, "Base %s packet empty, cannot send.", base_type.c_str());
    return;
  }
  if (base_packet.size() != spec.packet_size) {
    DAIKIN_WARN(TAG, "Base %s packet length %u invalid, cannot send.",
                base_type.c_str(), static_cast<unsigned>(base_packet.size()));
    return;
  }

  std::vector<uint8_t> command = base_packet;
  command[0] = spec.write_packet_type;

  if (apply_change) {
    if (index >= command.size()) {
      DAIKIN_WARN(TAG, "TX blocked: target index %u out of range for %s packet len=%u",
                  index, base_type.c_str(), static_cast<unsigned>(command.size()));
      reset_tx_lifecycle_(TxOperationKind::SINGLE_FIELD, true);
      return;
    }
    if (bit_position != BIT_POSITION_NO_BITMASK) {
      apply_field_value(command, index, bit_position, effective_bit_width(bit_position, bit_width), value);
    } else {
      command[index] = value;
    }
  }

  command.back() = ekhhe_checksum(command);

  std::string validation_error;
  if (!validate_outbound_cd_packet_(family, base_packet, command,
                                    apply_change ? TxPacketKind::SINGLE_FIELD : TxPacketKind::SNAPSHOT,
                                    index, value, bit_position, bit_width, validation_error)) {
    DAIKIN_WARN(TAG, "TX blocked by packet diff guard: %s", validation_error.c_str());
    clear_tx_wait_markers_();
    if (apply_change) {
      reset_tx_lifecycle_(TxOperationKind::SINGLE_FIELD, true);
      if (!uart_active_ && !uart_tx_active_) {
        start_uart_cycle();
      }
    }
    return;
  }

  const uint8_t attempts_sent =
      apply_change && single_field_tx_active_() ? tx_operation_.attempts_sent : 0;
  send_prebuilt_cd_packet_(family, command, apply_change ? TxPacketKind::SINGLE_FIELD : TxPacketKind::SNAPSHOT,
                           index, value, bit_position, bit_width, attempts_sent);
}

void DaikinEkhheComponent::send_uart_cc_packet_(const std::vector<uint8_t> &base_packet, bool apply_change,
                                                uint8_t index, uint8_t value, uint8_t bit_position,
                                                uint8_t bit_width) {
  send_uart_tx_packet_(TxPacketFamily::MAIN, base_packet, apply_change, index, value, bit_position, bit_width);
}

void DaikinEkhheComponent::send_restore_defaults_packet_(TxPacketFamily family,
                                                         const std::vector<uint8_t> &base_packet,
                                                         const std::vector<uint8_t> &packet) {
  const auto &spec = tx_packet_family_spec_(family);
  std::vector<uint8_t> command = packet;
  if (!command.empty()) {
    command[0] = spec.write_packet_type;
    command.back() = ekhhe_checksum(command);
  }

  std::string validation_error;
  if (!validate_outbound_cd_packet_(family, base_packet, command, TxPacketKind::RESTORE_DEFAULTS,
                                    0, 0, BIT_POSITION_NO_BITMASK, 0, validation_error)) {
    DAIKIN_WARN(TAG, "Restore defaults TX blocked by packet diff guard: family=%s %s",
                spec.label, validation_error.c_str());
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::RESTORE_DEFAULTS, true);
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(family, command, TxPacketKind::RESTORE_DEFAULTS, 0, 0, BIT_POSITION_NO_BITMASK,
                           0, tx_operation_.attempts_sent);
}

void DaikinEkhheComponent::send_profile_restore_packet_(TxPacketFamily family,
                                                        const std::vector<uint8_t> &base_packet,
                                                        const std::vector<uint8_t> &packet,
                                                        bool known_good) {
  const auto &spec = tx_packet_family_spec_(family);
  const bool extended = family == TxPacketFamily::EXTENDED;
  const ProfileState &profile = known_good ? known_good_profile_ : auto_snapshot_;
  const uint8_t profile_length = extended ? profile.extended_length : profile.main_length;
  const uint8_t *profile_data = extended ? profile.extended_data : profile.main_data;
  std::vector<uint8_t> command = packet;
  if (!command.empty()) {
    command[0] = spec.write_packet_type;
    command.back() = ekhhe_checksum(command);
  }

  std::vector<uint8_t> expected = base_packet;
  if (!expected.empty()) {
    expected[0] = spec.write_packet_type;
    merge_profile_managed_fields(extended, expected, profile_data, profile_length);
    expected.back() = ekhhe_checksum(expected);
  }
  if (!profile.valid || profile_length == 0 || command != expected) {
    DAIKIN_WARN(TAG, "%s TX blocked by packet diff guard: family=%s prebuilt command mismatch",
                known_good ? "Known-good profile restore" : "Auto snapshot restore",
                spec.label);
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, true);
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  if (base_packet.size() != packet.size() || command.size() != spec.packet_size ||
      base_packet.empty() || base_packet[0] != spec.base_packet_type ||
      command.empty() || command[0] != spec.write_packet_type) {
    DAIKIN_WARN(TAG, "%s TX blocked: family=%s invalid base/packet base=%u packet=%u",
                known_good ? "Known-good profile restore" : "Auto snapshot restore",
                spec.label,
                static_cast<unsigned>(base_packet.size()), static_cast<unsigned>(packet.size()));
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, true);
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(family, command, TxPacketKind::PROFILE_RESTORE, 0, 0, BIT_POSITION_NO_BITMASK,
                           0, tx_operation_.attempts_sent);
}

void DaikinEkhheComponent::send_time_band_packet_(const std::vector<uint8_t> &base_packet) {
  if (!time_band_tx_active_()) {
    return;
  }
  const auto &target = time_band_tx_();
  if (base_packet.empty()) {
    DAIKIN_WARN(TAG, "Time-band TX blocked: base CC packet empty.");
    reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
    return;
  }
  if (base_packet.size() != CC_PACKET_SIZE) {
    DAIKIN_WARN(TAG, "Time-band TX blocked: base CC packet length %u invalid.",
                static_cast<unsigned>(base_packet.size()));
    reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
    return;
  }

  std::vector<uint8_t> command = base_packet;
  command[0] = CD_PACKET_START_BYTE;
  command[CC_PACKET_TIME_BAND_FLAG_IDX] = target.flag;
  command[CC_PACKET_TIME_BAND_START_HOUR_IDX] = target.start_hour;
  command[CC_PACKET_TIME_BAND_START_MINUTE_IDX] = target.start_minute;
  command[CC_PACKET_TIME_BAND_END_HOUR_IDX] = target.end_hour;
  command[CC_PACKET_TIME_BAND_END_MINUTE_IDX] = target.end_minute;
  command[CC_PACKET_TIME_BAND_MODE_IDX] = target.mode;
  command.back() = ekhhe_checksum(command);

  std::string validation_error;
  if (!validate_outbound_cd_packet_(TxPacketFamily::MAIN, base_packet, command, TxPacketKind::TIME_BAND,
                                    0, 0, BIT_POSITION_NO_BITMASK, 0, validation_error)) {
    DAIKIN_WARN(TAG, "Time-band TX blocked by packet diff guard: %s", validation_error.c_str());
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(TxPacketFamily::MAIN, command, TxPacketKind::TIME_BAND, 0, 0,
                           BIT_POSITION_NO_BITMASK, 0, tx_operation_.attempts_sent);
}

void DaikinEkhheComponent::check_pending_tx_(const std::vector<uint8_t> &buffer) {
  if (!single_field_tx_active_()) {
    return;
  }
  const auto &target = single_field_tx_();
  const auto &spec = tx_packet_family_spec_(target.family);
  const uint8_t readback_index =
      tx_readback_index_(target.family, target.index, target.bit_position);
  const uint8_t readback_bit_position =
      tx_readback_bit_position_(target.family, target.index, target.bit_position);
  const uint8_t readback_bit_width =
      tx_readback_bit_width_(target.family, target.index, target.bit_position, target.bit_width);
  if (readback_index >= buffer.size()) {
    reset_tx_lifecycle_(TxOperationKind::SINGLE_FIELD, false);
    flush_deferred_user_tx_();
    return;
  }

  bool matched = field_matches_target_(buffer, readback_index, target.value,
                                       readback_bit_position, readback_bit_width);

  if (matched) {
    if (tx_operation_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "TX already current: family=%s index=%u readback_index=%u value=0x%02X bit=%u",
                 spec.label, target.index, readback_index, target.value, target.bit_position);
    } else {
      bool retried = tx_operation_.attempts_sent > 1;
      if (target.bit_position == BIT_POSITION_NO_BITMASK) {
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: family=%s readback=%s index=%u readback_index=%u value=0x%02X attempts=%u",
                      spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                      target.index, readback_index, target.value, tx_operation_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: family=%s readback=%s index=%u readback_index=%u value=0x%02X",
                   spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                   target.index, readback_index, target.value);
        }
      } else {
        uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                                  effective_bit_width(readback_bit_position, readback_bit_width));
        const uint8_t log_width = field_log_width(readback_index, readback_bit_position, readback_bit_width);
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u value=%u attempts=%u",
                      spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                      target.index, readback_index, target.bit_position, readback_bit_position,
                      log_width, field_value, tx_operation_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u value=%u",
                   spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                   target.index, readback_index, target.bit_position, readback_bit_position,
                   log_width, field_value);
        }
      }
      DAIKIN_DBG(TAG, "TX timing: applied_after=%u attempts=%u", millis() - tx_sent_ms_,
                 tx_operation_.attempts_sent);
      start_single_field_ui_sync_(target);
    }
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::SINGLE_FIELD, false);
    flush_deferred_user_tx_();
    return;
  }

  if (tx_operation_.attempts_sent == 0) {
    return;
  }

  if (tx_operation_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "TX retry pending: family=%s readback=%s index=%u readback_index=%u attempt=%u/%u",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               target.index, readback_index, tx_operation_.attempts_sent, kTxMaxRepeats);
    return;
  }

  if (target.bit_position == BIT_POSITION_NO_BITMASK) {
    DAIKIN_WARN(TAG, "TX not applied: family=%s readback=%s index=%u readback_index=%u expected=0x%02X current=0x%02X",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                target.index, readback_index, target.value, buffer[readback_index]);
  } else {
    uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                              effective_bit_width(readback_bit_position, readback_bit_width));
    const uint8_t mask = field_value_mask(readback_index, readback_bit_position, readback_bit_width);
    const uint8_t log_width = field_log_width(readback_index, readback_bit_position, readback_bit_width);
    DAIKIN_WARN(TAG, "TX not applied: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u expected=%u current=%u",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                target.index, readback_index, target.bit_position, readback_bit_position,
                log_width, target.value & mask, field_value);
  }
  DAIKIN_DBG(TAG, "TX timing: failed_after=%u attempts=%u", millis() - tx_sent_ms_,
             tx_operation_.attempts_sent);
  uint8_t index = target.index;
  if (target.bit_position == BIT_POSITION_NO_BITMASK) {
    const bool extended = target.family == TxPacketFamily::EXTENDED;
    for (const auto &entry : NUMBER_FIELD_SPECS) {
      const NumberFieldSpec &field = entry.second;
      if (field_is_extended(field.family) == extended && field.index == index) {
        set_number_value(entry.first, decode_number_field_value(buffer[readback_index], field));
      }
    }
    for (const auto &entry : SELECT_FIELD_SPECS) {
      const SelectFieldSpec &field = entry.second;
      if (field_is_extended(field.family) == extended && field.index == index &&
          field.bit_position == BIT_POSITION_NO_BITMASK) {
        set_select_value(entry.first, buffer[readback_index]);
      }
    }
  } else {
    uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                              effective_bit_width(readback_bit_position, readback_bit_width));
    const bool extended = target.family == TxPacketFamily::EXTENDED;
    for (const auto &entry : SELECT_FIELD_SPECS) {
      const SelectFieldSpec &field = entry.second;
      if (field_is_extended(field.family) == extended && field.index == index &&
          field.bit_position == target.bit_position) {
        set_select_value(entry.first, field_value);
      }
    }
#if defined(USE_SWITCH)
    if (target.family == TxPacketFamily::MAIN && index == CC_PACKET_MASK2_IDX &&
        target.bit_position == SILENT_MODE_BIT_POSITION) {
      set_switch_value(SILENT_MODE, field_value != 0);
    }
#endif
  }
  clear_tx_wait_markers_();
  reset_tx_lifecycle_(TxOperationKind::SINGLE_FIELD, true);
  flush_deferred_user_tx_();
}

void DaikinEkhheComponent::check_pending_restore_(const std::vector<uint8_t> &buffer) {
  if (!restore_tx_active_()) {
    return;
  }

  auto &target = restore_tx_();
  const TxPacketFamily family = target.family;
  const auto &spec = tx_packet_family_spec_(family);
  const bool extended = family == TxPacketFamily::EXTENDED;
  const bool matched = restore_defaults_match_packet(buffer, true, extended);
  if (matched) {
    if (family == TxPacketFamily::MAIN) {
      target.main_applied = true;
      if (tx_operation_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "Restore defaults main block applied after retries: attempts=%u",
                    tx_operation_.attempts_sent);
      } else if (tx_operation_.attempts_sent > 0) {
        ESP_LOGI(TAG, "Restore defaults main block applied.");
      } else {
        DAIKIN_DBG(TAG, "Restore defaults main block already current.");
      }
      if (tx_operation_.attempts_sent > 0) {
        DAIKIN_DBG(TAG, "TX timing: restore_main_applied_after=%u attempts=%u",
                   millis() - tx_sent_ms_, tx_operation_.attempts_sent);
      }
      target.family = TxPacketFamily::EXTENDED;
      tx_operation_.attempts_sent = 0;
      tx_operation_.last_attempt_d2_seq = 0;
      reset_queued_restore_();
      queued_restore_.active = true;
      queued_restore_.scheduled = false;
      queued_restore_.family = TxPacketFamily::EXTENDED;
      queued_restore_.generation++;
      queued_restore_.request_ms = tx_request_ms_;
      clear_tx_wait_markers_();
      DAIKIN_DBG(TAG, "Restore defaults scheduling: main complete, waiting_for_next_d2 for extended");
      return;
    }

    target.extended_applied = true;
    const bool wrote_any = target.main_write_sent || target.extended_write_sent;
    if (tx_operation_.attempts_sent == 0) {
      if (wrote_any) {
        ESP_LOGI(TAG, "Restore defaults applied.");
      } else {
        DAIKIN_DBG(TAG, "Restore defaults already current.");
      }
    } else {
      if (tx_operation_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "Restore defaults extended block applied after retries: attempts=%u",
                    tx_operation_.attempts_sent);
      } else {
        ESP_LOGI(TAG, "Restore defaults applied.");
      }
      DAIKIN_DBG(TAG, "TX timing: restore_extended_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    }
    if (wrote_any) {
      start_restore_ui_sync_();
    }
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::RESTORE_DEFAULTS, false);
    return;
  }

  if (tx_operation_.attempts_sent == 0) {
    return;
  }

  if (tx_operation_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "Restore defaults retry pending: family=%s readback=%s attempt=%u/%u",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               tx_operation_.attempts_sent, kTxMaxRepeats);
    return;
  }

  uint8_t current_value = 0;
  const RestoreFieldSpec *field = first_restore_mismatch(buffer, true, current_value, extended);
  if (field != nullptr) {
    DAIKIN_WARN(TAG,
                "Restore defaults not applied: family=%s readback=%s first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u partial_main=%u partial_extended=%u",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                field->name, field->value, current_value, tx_operation_.attempts_sent,
                target.main_applied, target.extended_applied);
  } else {
    DAIKIN_WARN(TAG, "Restore defaults not applied: family=%s attempts=%u partial_main=%u partial_extended=%u",
                spec.label, tx_operation_.attempts_sent,
                target.main_applied, target.extended_applied);
  }
  DAIKIN_DBG(TAG, "TX timing: restore_failed_after=%u family=%s attempts=%u",
             millis() - tx_sent_ms_, spec.label, tx_operation_.attempts_sent);
  clear_tx_wait_markers_();
  reset_tx_lifecycle_(TxOperationKind::RESTORE_DEFAULTS, false);
}

void DaikinEkhheComponent::check_pending_profile_restore_(const std::vector<uint8_t> &buffer) {
  if (!profile_restore_tx_active_()) {
    return;
  }

  auto &target = profile_restore_tx_();
  const ProfileState &profile =
      target.known_good ? known_good_profile_ : auto_snapshot_;
  const TxPacketFamily family = target.family;
  const auto &spec = tx_packet_family_spec_(family);
  const bool extended = family == TxPacketFamily::EXTENDED;
  const uint8_t profile_length = extended ? profile.extended_length : profile.main_length;
  const uint8_t *profile_data = extended ? profile.extended_data : profile.main_data;
  if (!profile.valid || profile_length == 0) {
    DAIKIN_WARN(TAG, "%s restore aborted: stored profile missing during confirmation.",
                target.known_good ? "Known-good profile" : "Auto snapshot");
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, false);
    return;
  }

  const bool matched = profile_matches_packet(extended, profile_data, profile_length, buffer, true);
  if (matched) {
    if (family == TxPacketFamily::MAIN) {
      target.main_applied = true;
      if (tx_operation_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "%s main block applied after retries: attempts=%u",
                    target.known_good ? "Known-good profile restore" : "Auto snapshot restore",
                    tx_operation_.attempts_sent);
      } else if (tx_operation_.attempts_sent > 0) {
        ESP_LOGI(TAG, "%s main block applied.",
                 target.known_good ? "Known-good profile restore" : "Auto snapshot restore");
      } else {
        DAIKIN_DBG(TAG, "%s main block already current.",
                   target.known_good ? "Known-good profile" : "Auto snapshot");
      }
      if (tx_operation_.attempts_sent > 0) {
        DAIKIN_DBG(TAG, "TX timing: profile_restore_main_applied_after=%u attempts=%u",
                   millis() - tx_sent_ms_, tx_operation_.attempts_sent);
      }
      target.family = TxPacketFamily::EXTENDED;
      tx_operation_.attempts_sent = 0;
      tx_operation_.last_attempt_d2_seq = 0;
      reset_queued_profile_restore_();
      queued_profile_restore_.active = true;
      queued_profile_restore_.scheduled = false;
      queued_profile_restore_.known_good = target.known_good;
      queued_profile_restore_.family = TxPacketFamily::EXTENDED;
      queued_profile_restore_.generation++;
      queued_profile_restore_.request_ms = tx_request_ms_;
      clear_tx_wait_markers_();
      DAIKIN_DBG(TAG, "%s restore scheduling: main complete, waiting_for_next_d2 for extended",
                 target.known_good ? "Known-good profile" : "Auto snapshot");
      return;
    }

    target.extended_applied = true;
    const bool wrote_any = target.main_write_sent || target.extended_write_sent;
    if (tx_operation_.attempts_sent == 0) {
      if (wrote_any) {
        ESP_LOGI(TAG, "%s applied.",
                 target.known_good ? "Known-good profile restore" : "Auto snapshot restore");
      } else {
        DAIKIN_DBG(TAG, "%s already current.",
                   target.known_good ? "Known-good profile" : "Auto snapshot");
      }
    } else {
      if (tx_operation_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "%s extended block applied after retries: attempts=%u",
                    target.known_good ? "Known-good profile restore" : "Auto snapshot restore",
                    tx_operation_.attempts_sent);
      } else {
        ESP_LOGI(TAG, "%s applied.",
                 target.known_good ? "Known-good profile restore" : "Auto snapshot restore");
      }
      DAIKIN_DBG(TAG, "TX timing: profile_restore_extended_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    }
    if (wrote_any) {
      start_profile_restore_ui_sync_(target.known_good);
    }
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, false);
    return;
  }

  if (tx_operation_.attempts_sent == 0) {
    return;
  }

  if (tx_operation_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "%s retry pending: family=%s readback=%s attempt=%u/%u",
               target.known_good ? "Known-good profile restore" : "Auto snapshot restore",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               tx_operation_.attempts_sent, kTxMaxRepeats);
    return;
  }

  uint8_t expected_value = 0;
  uint8_t current_value = 0;
  const ManagedFieldSpec *field = first_profile_mismatch(extended, profile_data, profile_length, buffer, true,
                                                         expected_value, current_value);
  if (field != nullptr) {
    DAIKIN_WARN(TAG,
                "%s not applied: family=%s readback=%s first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u partial_main=%u partial_extended=%u",
                target.known_good ? "Known-good profile restore" : "Auto snapshot restore",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                field->name, expected_value, current_value, tx_operation_.attempts_sent,
                target.main_applied, target.extended_applied);
  } else {
    DAIKIN_WARN(TAG, "%s not applied: family=%s attempts=%u partial_main=%u partial_extended=%u",
                target.known_good ? "Known-good profile restore" : "Auto snapshot restore",
                spec.label, tx_operation_.attempts_sent,
                target.main_applied, target.extended_applied);
  }
  DAIKIN_DBG(TAG, "TX timing: profile_restore_failed_after=%u family=%s attempts=%u",
             millis() - tx_sent_ms_, spec.label, tx_operation_.attempts_sent);
  clear_tx_wait_markers_();
  reset_tx_lifecycle_(TxOperationKind::PROFILE_RESTORE, false);
}

void DaikinEkhheComponent::check_pending_time_band_(const std::vector<uint8_t> &buffer) {
  if (!time_band_tx_active_()) {
    return;
  }
  const auto &target = time_band_tx_();
  if (buffer.size() <= D2_PACKET_TIME_BAND_MODE_IDX) {
    DAIKIN_WARN(TAG, "Time-band readback packet too short: len=%u", static_cast<unsigned>(buffer.size()));
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
    flush_deferred_user_tx_();
    return;
  }

  struct FieldCheck {
    const char *name;
    uint8_t index;
    uint8_t expected;
  };
  const FieldCheck fields[] = {
      {"flag", D2_PACKET_TIME_BAND_FLAG_IDX, target.flag},
      {"start_hour", D2_PACKET_TIME_BAND_START_HOUR_IDX, target.start_hour},
      {"start_minute", D2_PACKET_TIME_BAND_START_MINUTE_IDX, target.start_minute},
      {"end_hour", D2_PACKET_TIME_BAND_END_HOUR_IDX, target.end_hour},
      {"end_minute", D2_PACKET_TIME_BAND_END_MINUTE_IDX, target.end_minute},
      {"mode", D2_PACKET_TIME_BAND_MODE_IDX, target.mode},
  };

  const FieldCheck *first_mismatch = nullptr;
  for (const auto &field : fields) {
    if (buffer[field.index] != field.expected) {
      first_mismatch = &field;
      break;
    }
  }

  if (first_mismatch == nullptr) {
    if (tx_operation_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "Time-band already current: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
                 target.flag, target.start_hour, target.start_minute, target.end_hour, target.end_minute,
                 target.mode);
    } else if (tx_operation_.attempts_sent > 1) {
      DAIKIN_WARN(TAG,
                  "Time-band applied after retries: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u attempts=%u",
                  target.flag, target.start_hour, target.start_minute, target.end_hour, target.end_minute,
                  target.mode, tx_operation_.attempts_sent);
      DAIKIN_DBG(TAG, "TX timing: time_band_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    } else {
      ESP_LOGI(TAG, "Time-band applied: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
               target.flag, target.start_hour, target.start_minute, target.end_hour, target.end_minute,
               target.mode);
      DAIKIN_DBG(TAG, "TX timing: time_band_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    }

    update_time_band_state_from_bus_(buffer, true, true);
    if (tx_operation_.attempts_sent > 0) {
      start_time_band_ui_sync_(target);
    }
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
    flush_deferred_user_tx_();
    return;
  }

  if (tx_operation_.attempts_sent == 0) {
    return;
  }

  if (tx_operation_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG,
               "Time-band retry pending: first_mismatch=%s expected=0x%02X current=0x%02X attempt=%u/%u",
               first_mismatch->name, first_mismatch->expected, buffer[first_mismatch->index],
               tx_operation_.attempts_sent, kTxMaxRepeats);
    return;
  }

  DAIKIN_WARN(TAG,
              "Time-band not applied: first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u",
              first_mismatch->name, first_mismatch->expected, buffer[first_mismatch->index],
              tx_operation_.attempts_sent);
  DAIKIN_DBG(TAG, "TX timing: time_band_failed_after=%u attempts=%u",
             millis() - tx_sent_ms_, tx_operation_.attempts_sent);

  update_time_band_state_from_bus_(buffer, true, true);
  clear_tx_wait_markers_();
  reset_tx_lifecycle_(TxOperationKind::TIME_BAND, false);
  flush_deferred_user_tx_();
}

bool DaikinEkhheComponent::send_uart_cc_command(uint8_t index, uint8_t value,
                                                uint8_t bit_position, uint8_t bit_width) {
    return send_uart_command_(TxPacketFamily::MAIN, index, value, bit_position, bit_width);
}

bool DaikinEkhheComponent::send_uart_c2_command(uint8_t index, uint8_t value,
                                                uint8_t bit_position, uint8_t bit_width) {
    return send_uart_command_(TxPacketFamily::EXTENDED, index, value, bit_position, bit_width);
}

bool DaikinEkhheComponent::send_uart_command_(TxPacketFamily family, uint8_t index, uint8_t value,
                                              uint8_t bit_position, uint8_t bit_width) {
    const auto &spec = tx_packet_family_spec_(family);
    const std::vector<uint8_t> &base_cache =
        family == TxPacketFamily::EXTENDED ? last_c1_packet_ : last_cc_packet_;
    if (base_cache.empty()) {
        DAIKIN_WARN(TAG, "No %s packet received yet. Cannot send command.",
                    packet_type_to_string_(spec.base_packet_type).c_str());
        return false;
    }
    if (restore_tx_busy_()) {
        DAIKIN_WARN(TAG, "Restore defaults in progress, ignoring single-parameter write.");
        return false;
    }
    if (profile_restore_tx_busy_()) {
        DAIKIN_WARN(TAG, "Profile restore in progress, ignoring single-parameter write.");
        return false;
    }
    if (time_band_tx_busy_()) {
        return defer_single_field_tx_(family, index, value, bit_position, bit_width);
    }
    if (single_field_tx_busy_()) {
        return defer_single_field_tx_(family, index, value, bit_position, bit_width);
    }

    auto_save_snapshot_if_needed_();

    tx_request_ms_ = millis();
    {
      const uint32_t now_ms = tx_request_ms_;
      const uint32_t since_last_frame = last_frame_profile_ms_ == 0 ? 0 : (now_ms - last_frame_profile_ms_);
      const uint32_t since_last_cc = last_cc_profile_ms_ == 0 ? 0 : (now_ms - last_cc_profile_ms_);
      const std::string last_type =
          last_frame_profile_ms_ == 0 ? "none" : packet_type_to_string_(last_frame_profile_type_);
      size_t d2_phase_index = 0;
      const RawFrameEntry *d2_phase_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_phase_index, true);
      const uint32_t since_last_d2 = d2_phase_entry != nullptr ? (now_ms - d2_phase_entry->timestamp_ms) : 0;
      size_t base_phase_index = 0;
      const RawFrameEntry *base_phase_entry = find_latest_frame_by_type_(spec.base_packet_type, base_phase_index, true);
      const uint32_t since_last_base =
          base_phase_entry != nullptr ? (now_ms - base_phase_entry->timestamp_ms) : 0;
      std::string cycle_types = packet_mask_to_string_(cycle_packet_types_seen_);
      const uint8_t log_width = field_log_width(index, bit_position, bit_width);
      DAIKIN_DBG(TAG,
                 "TX request phase: family=%s index=%u value=0x%02X bit=%u width=%u last=%s since_last=%u since_d2=%u since_cc=%u since_base=%u cycle_ms=%u cycle_types=%s uart_active=%u",
                 spec.label, index, value, bit_position, log_width, last_type.c_str(), since_last_frame,
                 since_last_d2, since_last_cc, since_last_base, now_ms - cycle_start_ms_, cycle_types.c_str(),
                 uart_active_);
    }
    reset_tx_operation_();
    tx_operation_.kind = TxOperationKind::SINGLE_FIELD;
    tx_operation_.single_field.family = family;
    tx_operation_.single_field.index = index;
    tx_operation_.single_field.value = value;
    tx_operation_.single_field.bit_position = bit_position;
    tx_operation_.single_field.bit_width = bit_width;
    tx_operation_.attempts_sent = 0;
    tx_operation_.last_attempt_d2_seq = 0;
    reset_tx_ui_sync_();

    queued_tx_.active = true;
    queued_tx_.scheduled = false;
    queued_tx_.family = family;
    queued_tx_.index = index;
    queued_tx_.value = value;
    queued_tx_.bit_position = bit_position;
    queued_tx_.bit_width = bit_width;
    queued_tx_.generation++;
    queued_tx_.request_ms = tx_request_ms_;
    queued_tx_.anchor_ms = 0;
    queued_tx_.anchor_seq = 0;
    clear_tx_wait_markers_();

    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }

    size_t d2_index = 0;
    const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
    if (d2_entry != nullptr) {
      uint32_t d2_age_ms = millis() - d2_entry->timestamp_ms;
      if (d2_age_ms <= tx_delay_after_d2_ms_) {
        schedule_queued_tx_from_d2_(*d2_entry);
        return true;
      }
    }

    const uint8_t log_width = field_log_width(index, bit_position, bit_width);
    DAIKIN_DBG(TAG, "TX scheduling: family=%s waiting_for_next_d2 index=%u value=0x%02X bit=%u width=%u",
               spec.label, index, value, bit_position, log_width);
    return true;
}


}  // namespace daikin_ekkhe
}  // namespace esphome
