#include "daikin_ekhhe.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"


#include <cinttypes>
#include <cstdio>
#include <numeric>
#include <ctime>
#include <cmath>
#include <cstring>

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
      uint8_t byte = this->read();
      cycle_bytes_read_++;
      store_latest_packet(byte);
      if (packet_set_complete()) {
        break;
      }
    }

    now = millis();
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

void DaikinEkhheComponent::store_latest_packet(uint8_t byte) {
  last_rx_time_ = millis();

  // Wait for a start byte
  if (PACKET_SIZES.find(byte) == PACKET_SIZES.end()) {
    return;
  }

  // Read the expected packet size
  uint8_t expected_length = PACKET_SIZES.at(byte);
  std::vector<uint8_t> packet(expected_length);
  packet[0] = byte;

  // Read the rest of the packet
  if (!read_packet_bytes_(packet.data() + 1, expected_length - 1, kFrameReadTimeoutMs)) {
    raw_frames_error_++;
    raw_framing_errors_total_++;
    if (cycle_synced_) {
      cycle_framing_errors_++;
      cycle_framing_error_start_ = byte;
    }
    return;
  }

  uint8_t packet_mask = packet_mask_for_start_(byte);
  uint8_t flags = 0;
  bool crc_ok = true;
  if ((packet_mask & kChecksumPacketMask) != 0) {
    crc_ok = (ekhhe_checksum(packet) == packet.back());
    if (!crc_ok) {
      flags |= RAW_FRAME_CRC_ERROR;
    }
  }

  store_raw_frame_(byte, packet.data(), expected_length, flags);

  if (!crc_ok) {
    if (cycle_synced_) {
      if (latest_packets_.count(byte) == 0) {
        cycle_checksum_errors_++;
        cycle_checksum_error_mask_ |= packet_mask;
      }
    }
    return;
  }

  const uint32_t now_ms = millis();
  if (debug_mode_) {
    const uint32_t dt_prev = last_frame_profile_ms_ == 0 ? 0 : (now_ms - last_frame_profile_ms_);
    const uint32_t dt_prev_cc = last_cc_profile_ms_ == 0 ? 0 : (now_ms - last_cc_profile_ms_);
    const std::string prev_type =
        last_frame_profile_ms_ == 0 ? "none" : packet_type_to_string_(last_frame_profile_type_);
    const std::string type = packet_type_to_string_(byte);

    DAIKIN_DBG(TAG,
               "RX timing: seq=%u type=%s len=%u dt_prev=%u prev=%s dt_prev_cc=%u cycle_ms=%u pending=%u tx_active=%u",
               raw_frame_seq_, type.c_str(), expected_length, dt_prev, prev_type.c_str(), dt_prev_cc,
               now_ms - cycle_start_ms_, pending_tx_.active, uart_tx_active_);

    if (byte == CD_PACKET_START_BYTE) {
      DAIKIN_DBG(TAG, "RX observed CD on bus: seq=%u dt_prev=%u dt_prev_cc=%u", raw_frame_seq_, dt_prev, dt_prev_cc);
      const uint32_t cd_seq = raw_frame_seq_;
      set_timeout(kCdContextLogDelayMs, [this, cd_seq]() {
        log_frame_context_(cd_seq, kCdContextFramesBefore, kCdContextFramesAfter);
      });
    }

    if (pending_tx_.active && tx_waiting_for_first_rx_) {
      DAIKIN_DBG(TAG, "TX timing: first_rx_after_tx type=%s dt=%u", type.c_str(), now_ms - tx_sent_ms_);
      tx_waiting_for_first_rx_ = false;
    }

    if (pending_tx_.active && tx_waiting_for_first_cc_ && byte == CC_PACKET_START_BYTE) {
      DAIKIN_DBG(TAG, "TX timing: first_cc_after_tx dt=%u", now_ms - tx_sent_ms_);
      tx_waiting_for_first_cc_ = false;
    }
  }

  last_frame_profile_ms_ = now_ms;
  last_frame_profile_type_ = byte;
  if (byte == CC_PACKET_START_BYTE) {
    last_cc_profile_ms_ = now_ms;
  }
  if (byte == D2_PACKET_START_BYTE && pending_tx_.active) {
    check_pending_tx_(packet);
  }
  if (byte == D2_PACKET_START_BYTE && pending_tx_.active) {
    size_t d2_index = 0;
    const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
    if (d2_entry != nullptr) {
      schedule_queued_tx_from_d2_(*d2_entry);
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
    cycle_bytes_read_ = expected_length;
  }

  if (cycle_synced_ && latest_packets_.count(byte) > 0) {
    return;
  }

  // Store only the latest version of each valid packet
  latest_packets_[byte] = packet;
  cycle_packets_seen_++;
  cycle_packet_types_seen_ |= packet_mask;
}

bool DaikinEkhheComponent::read_packet_bytes_(uint8_t *dest, size_t length, uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  size_t offset = 0;

  while (offset < length) {
    if (this->available()) {
      dest[offset++] = this->read();
      cycle_bytes_read_++;
    } else if (millis() - start_ms >= timeout_ms) {
      return false;
    }
  }

  return true;
}

void DaikinEkhheComponent::store_raw_frame_(uint8_t packet_type, const uint8_t *data, size_t length, uint8_t flags) {
  if (length > kRawFrameMaxLen) {
    raw_frames_truncated_++;
    raw_frames_error_++;
    length = kRawFrameMaxLen;
    flags |= RAW_FRAME_TRUNCATED;
  }

  if (raw_frame_count_ == kRawFrameBufferSize) {
    raw_frames_dropped_++;
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
  raw_frames_captured_++;
  raw_bytes_captured_ += length;
  if ((flags & RAW_FRAME_CRC_ERROR) != 0) {
    raw_crc_errors_total_++;
  }
  if ((flags & (RAW_FRAME_CRC_ERROR | RAW_FRAME_TIMEOUT | RAW_FRAME_TRUNCATED | RAW_FRAME_UNKNOWN_TYPE)) != 0) {
    raw_frames_error_++;
  }
}

void DaikinEkhheComponent::reset_cycle_stats_() {
  cycle_start_ms_ = millis();
  last_rx_time_ = cycle_start_ms_;
  cycle_bytes_read_ = 0;
  cycle_packets_seen_ = 0;
  cycle_packets_parsed_ = 0;
  cycle_parse_ms_ = 0;
  cycle_total_ms_ = 0;
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

bool DaikinEkhheComponent::should_publish_debug_text_(const std::string &key, const std::string &value,
                                                      uint32_t min_interval_ms) {
  uint32_t now = millis();
  auto it = debug_last_published_text_.find(key);
  if (it == debug_last_published_text_.end()) {
    debug_last_published_text_[key] = value;
    debug_last_published_text_ms_[key] = now;
    return true;
  }

  uint32_t last_ms = debug_last_published_text_ms_[key];
  if (now - last_ms < min_interval_ms) {
    return false;
  }

  if (it->second != value) {
    it->second = value;
    debug_last_published_text_ms_[key] = now;
    return true;
  }

  return false;
}

uint8_t DaikinEkhheComponent::packet_type_from_string_(const std::string &value) const {
  if (value == "DD") return DD_PACKET_START_BYTE;
  if (value == "D2") return D2_PACKET_START_BYTE;
  if (value == "D4") return D4_PACKET_START_BYTE;
  if (value == "C1") return C1_PACKET_START_BYTE;
  if (value == "CC") return CC_PACKET_START_BYTE;
  return 0;
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
    case CC_PACKET_START_BYTE:
      return "CC";
    case CD_PACKET_START_BYTE:
      return "CD";
    default:
      return "latest";
  }
}

const DaikinEkhheComponent::RawFrameEntry *DaikinEkhheComponent::find_raw_frame_by_seq_(uint32_t seq,
                                                                                        size_t &index) const {
  if (raw_frame_count_ == 0) {
    return nullptr;
  }
  for (size_t i = 0; i < raw_frame_count_; ++i) {
    size_t idx = (raw_frame_head_ + i) % kRawFrameBufferSize;
    const RawFrameEntry &entry = raw_frames_[idx];
    if (entry.seq == seq) {
      index = idx;
      return &entry;
    }
  }
  return nullptr;
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

const DaikinEkhheComponent::RawFrameEntry *DaikinEkhheComponent::find_previous_frame_by_type_(
    uint8_t packet_type, uint32_t seq, size_t &index, bool require_ok) const {
  if (raw_frame_count_ == 0 || packet_type == 0) {
    return nullptr;
  }
  bool past_current = false;
  for (size_t i = 0; i < raw_frame_count_; ++i) {
    size_t idx = (raw_frame_head_ + raw_frame_count_ - 1 - i) % kRawFrameBufferSize;
    const RawFrameEntry &entry = raw_frames_[idx];
    if (entry.packet_type != packet_type) {
      continue;
    }
    if (!past_current) {
      if (entry.seq == seq) {
        past_current = true;
      }
      continue;
    }
    if (require_ok && !is_frame_ok_(entry)) {
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
                                                 uint8_t bit_position) const {
  if (index >= buffer.size()) {
    return false;
  }
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return buffer[index] == value;
  }
  uint8_t bit = (buffer[index] >> bit_position) & 0x01;
  return bit == (value & 0x01);
}

void DaikinEkhheComponent::schedule_queued_tx_from_d2_(const RawFrameEntry &d2_entry) {
  if (!pending_tx_.active || queued_tx_.scheduled) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms = d2_age_ms >= kTxDelayAfterD2Ms ? 0 : (kTxDelayAfterD2Ms - d2_age_ms);
  const uint32_t generation = queued_tx_.generation;

  queued_tx_.active = true;
  queued_tx_.scheduled = true;
  queued_tx_.index = pending_tx_.index;
  queued_tx_.value = pending_tx_.value;
  queued_tx_.bit_position = pending_tx_.bit_position;
  queued_tx_.anchor_ms = d2_entry.timestamp_ms;
  queued_tx_.anchor_seq = d2_entry.seq;

  DAIKIN_DBG(TAG, "TX scheduling: d2_seq=%u d2_age=%u delay=%u request_age=%u",
             d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_tx_.request_ms);

  set_timeout(delay_ms, [this, generation]() {
    if (!queued_tx_.active || queued_tx_.generation != generation) {
      return;
    }

    uint8_t index = queued_tx_.index;
    uint8_t value = queued_tx_.value;
    uint8_t bit_position = queued_tx_.bit_position;
    uint32_t anchor_seq = queued_tx_.anchor_seq;
    uint32_t anchor_ms = queued_tx_.anchor_ms;

    queued_tx_.active = false;
    queued_tx_.scheduled = false;

    std::vector<uint8_t> base_packet = last_cc_packet_;
    auto latest_cc = latest_packets_.find(CC_PACKET_START_BYTE);
    if (latest_cc != latest_packets_.end()) {
      base_packet = latest_cc->second;
    }

    pending_tx_.attempts_sent++;
    pending_tx_.last_attempt_d2_seq = anchor_seq;

    DAIKIN_DBG(TAG, "TX scheduling: sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_cc=%u",
               anchor_seq, millis() - anchor_ms, pending_tx_.attempts_sent, kTxMaxRepeats,
               latest_cc != latest_packets_.end());
    send_uart_cc_packet_(base_packet, true, index, value, bit_position);
  });
}

const DaikinEkhheComponent::RawFrameEntry *DaikinEkhheComponent::select_raw_frame_(size_t &index, size_t &back) {
  if (raw_frame_count_ == 0) {
    return nullptr;
  }

  if (debug_freeze_ && debug_frozen_seq_ != 0) {
    const RawFrameEntry *entry = find_raw_frame_by_seq_(debug_frozen_seq_, index);
    if (entry != nullptr) {
      size_t newest_index = (raw_frame_head_ + raw_frame_count_ - 1) % kRawFrameBufferSize;
      back = (newest_index + kRawFrameBufferSize - index) % kRawFrameBufferSize;
      if (back >= raw_frame_count_) {
        back = raw_frame_count_ - 1;
      }
      return entry;
    }

    debug_freeze_ = false;
    debug_frozen_seq_ = 0;
#if DAIKIN_EKHHE_DEBUG && defined(USE_SWITCH)
    if (debug_freeze_switch_ != nullptr) {
      debug_freeze_switch_->publish_state(false);
    }
#endif
  }

  const RawFrameEntry *entry = find_latest_frame_by_type_(debug_packet_type_, index, true);
  if (entry == nullptr) {
    entry = find_latest_frame_by_type_(debug_packet_type_, index, false);
  }
  if (entry == nullptr) {
    return nullptr;
  }
  size_t newest_index = (raw_frame_head_ + raw_frame_count_ - 1) % kRawFrameBufferSize;
  back = (newest_index + kRawFrameBufferSize - index) % kRawFrameBufferSize;
  if (back >= raw_frame_count_) {
    back = raw_frame_count_ - 1;
  }
  return entry;
}

std::string DaikinEkhheComponent::raw_frame_flags_to_string_(uint8_t flags) const {
  if (flags == 0) {
    return "none";
  }
  std::string out;
  if (flags & RAW_FRAME_CRC_ERROR) out += "crc,";
  if (flags & RAW_FRAME_TIMEOUT) out += "timeout,";
  if (flags & RAW_FRAME_TRUNCATED) out += "truncated,";
  if (flags & RAW_FRAME_UNKNOWN_TYPE) out += "unknown,";
  if (!out.empty()) {
    out.pop_back();
  }
  return out;
}

std::string DaikinEkhheComponent::format_raw_frame_hex_(const RawFrameEntry &entry) const {
  static const char hex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(entry.length * 4);
  for (size_t i = 0; i < entry.length; ++i) {
    if (i % 16 == 0) {
      char header[8];
      snprintf(header, sizeof(header), "%02X:", static_cast<unsigned>(i));
      if (!out.empty()) {
        out.push_back('\n');
      }
      out.append(header);
    }
    out.push_back(' ');
    uint8_t byte = entry.data[i];
    out.push_back(hex[(byte >> 4) & 0x0F]);
    out.push_back(hex[byte & 0x0F]);
  }
  return out;
}

std::string DaikinEkhheComponent::format_raw_frame_hex_data_(const uint8_t *data, size_t length) const {
  static const char hex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(length * 4);
  for (size_t i = 0; i < length; ++i) {
    if (i % 16 == 0) {
      char header[8];
      snprintf(header, sizeof(header), "%02X:", static_cast<unsigned>(i));
      if (!out.empty()) {
        out.push_back('\n');
      }
      out.append(header);
    }
    out.push_back(' ');
    uint8_t byte = data[i];
    out.push_back(hex[(byte >> 4) & 0x0F]);
    out.push_back(hex[byte & 0x0F]);
  }
  return out;
}

std::string DaikinEkhheComponent::format_raw_frame_meta_(const RawFrameEntry &entry, size_t index, size_t back,
                                                         uint32_t now_ms) const {
  std::string flags = raw_frame_flags_to_string_(entry.flags);
  char buffer[160];
  uint32_t age_ms = now_ms - entry.timestamp_ms;
  snprintf(buffer, sizeof(buffer), "type=0x%02X len=%u flags=%s ts_ms=%u pos=%u back=%u age_ms=%u seq=%u",
           entry.packet_type, entry.length, flags.c_str(), entry.timestamp_ms, static_cast<unsigned>(index),
           static_cast<unsigned>(back), age_ms, entry.seq);
  return std::string(buffer);
}

void DaikinEkhheComponent::log_frame_context_(uint32_t center_seq, uint8_t before, uint8_t after) const {
  size_t center_index = 0;
  const RawFrameEntry *center = find_raw_frame_by_seq_(center_seq, center_index);
  if (center == nullptr) {
    DAIKIN_DBG(TAG, "CD context: seq=%u not found count=%u dropped=%u", center_seq,
               static_cast<unsigned>(raw_frame_count_), raw_frames_dropped_);
    return;
  }

  size_t center_pos = (center_index + kRawFrameBufferSize - raw_frame_head_) % kRawFrameBufferSize;
  if (center_pos >= raw_frame_count_) {
    DAIKIN_DBG(TAG, "CD context: seq=%u index=%u outside active buffer count=%u", center_seq,
               static_cast<unsigned>(center_index), static_cast<unsigned>(raw_frame_count_));
    return;
  }

  size_t start_pos = center_pos > before ? center_pos - before : 0;
  size_t end_pos = center_pos + static_cast<size_t>(after);
  if (end_pos >= raw_frame_count_) {
    end_pos = raw_frame_count_ - 1;
  }

  DAIKIN_DBG(TAG, "CD context: center_seq=%u center_pos=%u window=%u..%u count=%u dropped=%u",
             center_seq, static_cast<unsigned>(center_pos), static_cast<unsigned>(start_pos),
             static_cast<unsigned>(end_pos), static_cast<unsigned>(raw_frame_count_), raw_frames_dropped_);

  uint32_t prev_ts = 0;
  for (size_t pos = start_pos; pos <= end_pos; ++pos) {
    size_t idx = (raw_frame_head_ + pos) % kRawFrameBufferSize;
    const RawFrameEntry &entry = raw_frames_[idx];
    uint32_t dt_prev = prev_ts == 0 ? 0 : (entry.timestamp_ms - prev_ts);
    int rel = static_cast<int>(pos) - static_cast<int>(center_pos);
    std::string type = packet_type_to_string_(entry.packet_type);
    std::string flags = raw_frame_flags_to_string_(entry.flags);
    DAIKIN_DBG(TAG, "CD ctx rel=%d seq=%u type=%s flags=%s ts_ms=%u dt_prev=%u len=%u",
               rel, entry.seq, type.c_str(), flags.c_str(), entry.timestamp_ms, dt_prev, entry.length);
    prev_ts = entry.timestamp_ms;
  }
}

bool DaikinEkhheComponent::is_known_offset_(uint8_t packet_type, size_t offset, size_t length) const {
  if (offset == 0 || offset == length - 1) {
    return true;
  }

  auto has_offset = [](const uint8_t *list, size_t count, size_t value) {
    for (size_t i = 0; i < count; ++i) {
      if (list[i] == value) {
        return true;
      }
    }
    return false;
  };

  switch (packet_type) {
    case DD_PACKET_START_BYTE: {
      static const uint8_t dd_known[] = {
          DD_PACKET_START_IDX,
          DD_PACKET_B_IDX,
          DD_PACKET_A_IDX,
          DD_PACKET_C_IDX,
          DD_PACKET_D_IDX,
          DD_PACKET_E_IDX,
          DD_PACKET_F_IDX,
          DD_PACKET_G_IDX,
          DD_PACKET_H_IDX,
          DD_PACKET_I_IDX,
          DD_PACKET_DIG_IDX,
          DD_PACKET_END,
      };
      return has_offset(dd_known, sizeof(dd_known) / sizeof(dd_known[0]), offset);
    }
    case D2_PACKET_START_BYTE: {
      static const uint8_t d2_known[] = {
          D2_PACKET_START_IDX,
          D2_PACKET_MASK1_IDX,
          D2_PACKET_MASK2_IDX,
          D2_PACKET_MODE_IDX,
          D2_PACKET_P4_IDX,
          D2_PACKET_P7_IDX,
          D2_PACKET_P10_IDX,
          D2_PACKET_P2_IDX,
          D2_PACKET_VAC_DAYS,
          D2_PACKET_P29_IDX,
          D2_PACKET_P31_IDX,
          D2_PACKET_P8_IDX,
          D2_PACKET_P9_IDX,
          D2_PACKET_ECO_TTARGET_IDX,
          D2_PACKET_AUTO_TTARGET_IDX,
          D2_PACKET_BOOST_TTGARGET_IDX,
          D2_PACKET_ELECTRIC_TTARGET_IDX,
          D2_PACKET_P1_IDX,
          D2_PACKET_P32_IDX,
          D2_PACKET_P3_IDX,
          D2_PACKET_P30_IDX,
          D2_PACKET_P25_IDX,
          D2_PACKET_P26_IDX,
          D2_PACKET_P27_IDX,
          D2_PACKET_P28_IDX,
          D2_PACKET_P12_IDX,
          D2_PACKET_P14_IDX,
          D2_PACKET_P24_IDX,
          D2_PACKET_P16_IDX,
          D2_PACKET_P23_IDX,
          D2_PACKET_P17_IDX,
          D2_PACKET_P18_IDX,
          D2_PACKET_P19_IDX,
          D2_PACKET_P20_IDX,
          D2_PACKET_P21_IDX,
          D2_PACKET_P22_IDX,
          D2_PACKET_P34_IDX,
          D2_PACKET_P37_IDX,
          D2_PACKET_P38_IDX,
          D2_PACKET_P40_IDX,
          D2_PACKET_P36_IDX,
          D2_PACKET_P35_IDX,
          D2_PACKET_P41_IDX,
          D2_PACKET_P42_IDX,
          D2_PACKET_P43_IDX,
          D2_PACKET_P44_IDX,
          D2_PACKET_P45_IDX,
          D2_PACKET_P46_IDX,
          D2_PACKET_HOUR_IDX,
          D2_PACKET_MIN_IDX,
          D2_PACKET_P47_IDX,
          D2_PACKET_P48_IDX,
          D2_PACKET_P49_IDX,
          D2_PACKET_P50_IDX,
          D2_PACKET_P51_IDX,
          D2_PACKET_P52_IDX,
          D2_PACKET_END,
      };
      return has_offset(d2_known, sizeof(d2_known) / sizeof(d2_known[0]), offset);
    }
    case D4_PACKET_START_BYTE: {
      static const uint8_t d4_known[] = {
          D4_PACKET_START_IDX,
          D4_PACKET_END,
      };
      return has_offset(d4_known, sizeof(d4_known) / sizeof(d4_known[0]), offset);
    }
    case C1_PACKET_START_BYTE: {
      static const uint8_t c1_known[] = {
          C1_PACKET_START_IDX,
          C1_PACKET_END,
      };
      return has_offset(c1_known, sizeof(c1_known) / sizeof(c1_known[0]), offset);
    }
    case CC_PACKET_START_BYTE: {
      static const uint8_t cc_known[] = {
          CC_PACKET_START_IDX,
          CC_PACKET_MASK1_IDX,
          CC_PACKET_MASK2_IDX,
          CC_PACKET_MODE_IDX,
          CC_PACKET_P4_IDX,
          CC_PACKET_P7_IDX,
          CC_PACKET_P10_IDX,
          CC_PACKET_P2_IDX,
          CC_PACKET_VAC_DAYS,
          CC_PACKET_P29_IDX,
          CC_PACKET_P31_IDX,
          CC_PACKET_P8_IDX,
          CC_PACKET_P9_IDX,
          CC_PACKET_ECO_TTARGET_IDX,
          CC_PACKET_AUTO_TTARGET_IDX,
          CC_PACKET_BOOST_TTGARGET_IDX,
          CC_PACKET_ELECTRIC_TTARGET_IDX,
          CC_PACKET_P1_IDX,
          CC_PACKET_P32_IDX,
          CC_PACKET_P3_IDX,
          CC_PACKET_P30_IDX,
          CC_PACKET_P25_IDX,
          CC_PACKET_P26_IDX,
          CC_PACKET_P27_IDX,
          CC_PACKET_P28_IDX,
          CC_PACKET_P12_IDX,
          CC_PACKET_P14_IDX,
          CC_PACKET_P24_IDX,
          CC_PACKET_P16_IDX,
          CC_PACKET_P23_IDX,
          CC_PACKET_P17_IDX,
          CC_PACKET_P18_IDX,
          CC_PACKET_P19_IDX,
          CC_PACKET_P20_IDX,
          CC_PACKET_P21_IDX,
          CC_PACKET_P22_IDX,
          CC_PACKET_P34_IDX,
          CC_PACKET_P37_IDX,
          CC_PACKET_P38_IDX,
          CC_PACKET_P40_IDX,
          CC_PACKET_P36_IDX,
          CC_PACKET_P35_IDX,
          CC_PACKET_P41_IDX,
          CC_PACKET_P42_IDX,
          CC_PACKET_P43_IDX,
          CC_PACKET_P44_IDX,
          CC_PACKET_P45_IDX,
          CC_PACKET_P46_IDX,
          CC_PACKET_HOUR_IDX,
          CC_PACKET_MIN_IDX,
          CC_PACKET_P47_IDX,
          CC_PACKET_P48_IDX,
          CC_PACKET_P49_IDX,
          CC_PACKET_P50_IDX,
          CC_PACKET_P51_IDX,
          CC_PACKET_P52_IDX,
          CC_PACKET_P54_IDX,
          CC_PACKET_END,
      };
      return has_offset(cc_known, sizeof(cc_known) / sizeof(cc_known[0]), offset);
    }
    default:
      return false;
  }
}

std::string DaikinEkhheComponent::format_unknown_fields_(const RawFrameEntry &entry) const {
  std::string out;
  char header[64];
  snprintf(header, sizeof(header), "type=0x%02X len=%u unknown:", entry.packet_type, entry.length);
  out.append(header);
  bool any = false;
  size_t rows = (entry.length + 15) / 16;
  for (size_t row = 0; row < rows; ++row) {
    size_t row_start = row * 16;
    size_t row_end = row_start + 16;
    if (row_end > entry.length) {
      row_end = entry.length;
    }
    bool row_has_unknown = false;
    for (size_t i = row_start; i < row_end; ++i) {
      if (!is_known_offset_(entry.packet_type, i, entry.length)) {
        row_has_unknown = true;
        break;
      }
    }
    if (!row_has_unknown) {
      continue;
    }
    any = true;
    out.push_back('\n');
    char row_buf[16];
    snprintf(row_buf, sizeof(row_buf), "%u:", static_cast<unsigned>(row_start));
    out.append(row_buf);
    for (size_t i = row_start; i < row_end; ++i) {
      out.push_back(' ');
      if (is_known_offset_(entry.packet_type, i, entry.length)) {
        out.append("--");
      } else {
        char byte_buf[4];
        snprintf(byte_buf, sizeof(byte_buf), "%02X", entry.data[i]);
        out.append(byte_buf);
      }
    }
  }
  if (!any) {
    out.append("\nnone");
  }
  return out;
}

std::string DaikinEkhheComponent::format_frame_diff_(const RawFrameEntry &entry,
                                                     const RawFrameEntry *prev) const {
  static const char hex[] = "0123456789ABCDEF";
  char header[96];
  if (prev == nullptr) {
    snprintf(header, sizeof(header), "type=0x%02X len=%u diff=none (no previous)", entry.packet_type, entry.length);
    return std::string(header);
  }
  size_t length = entry.length < prev->length ? entry.length : prev->length;
  size_t changes = 0;
  std::string out;
  snprintf(header, sizeof(header), "type=0x%02X len=%u prev_len=%u diff:", entry.packet_type, entry.length,
           prev->length);
  out.append(header);
  for (size_t i = 0; i < length; ++i) {
    uint8_t now = entry.data[i];
    uint8_t before = prev->data[i];
    if (now == before) {
      continue;
    }
    if (changes % 8 == 0) {
      out.push_back('\n');
    }
    char chunk[24];
    snprintf(chunk, sizeof(chunk), "%u:%02X>%02X ", static_cast<unsigned>(i), before, now);
    out.append(chunk);
    changes++;
    if (changes >= 32) {
      out.append("...");
      break;
    }
  }
  if (changes == 0 && entry.length != prev->length) {
    out.append("\nlen change only");
  } else if (changes == 0) {
    out.append("\nno changes");
  }
  return out;
}

void DaikinEkhheComponent::publish_debug_outputs_() {
  if (!debug_mode_) {
    return;
  }

  uint32_t now_ms = millis();
  auto publish_debug_sensor = [this](const std::string &key, float value, uint32_t min_interval_ms,
                                     uint32_t refresh_ms) {
    if (debug_sensors_.find(key) == debug_sensors_.end()) {
      return;
    }
    if (should_publish_float_(key, value, debug_last_published_values_, debug_last_published_values_ms_,
                              min_interval_ms, 0.0f, refresh_ms)) {
      defer([this, key, value]() { debug_sensors_[key]->publish_state(value); });
    }
  };

  publish_debug_sensor("frames_captured_total", raw_frames_captured_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("frames_dropped_total", raw_frames_dropped_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("frames_truncated_total", raw_frames_truncated_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("crc_errors_total", raw_crc_errors_total_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("framing_errors_total", raw_framing_errors_total_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("bytes_captured_total", raw_bytes_captured_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);
  publish_debug_sensor("cycle_parse_time_ms", cycle_parse_ms_, kDebugTimingPublishMinIntervalMs, 0);
  publish_debug_sensor("cycle_total_time_ms", cycle_total_ms_, kDebugTimingPublishMinIntervalMs, 0);
  publish_debug_sensor("cycle_over_budget_total", cycle_over_budget_total_, kDebugCounterPublishIntervalMs,
                       kDebugCounterPublishIntervalMs);

  size_t index = 0;
  size_t back = 0;
  const RawFrameEntry *entry = select_raw_frame_(index, back);
  if (entry == nullptr) {
    return;
  }

  auto publish_debug_text = [this](const std::string &key, const std::string &value) {
    if (debug_text_sensors_.find(key) == debug_text_sensors_.end()) {
      return;
    }
    if (should_publish_debug_text_(key, value, kDebugTextPublishMinIntervalMs)) {
      defer([this, key, value]() { debug_text_sensors_[key]->publish_state(value); });
    }
  };

  publish_debug_text("daikin_raw_frame_hex", format_raw_frame_hex_(*entry));
  publish_debug_text("daikin_raw_frame_meta", format_raw_frame_meta_(*entry, index, back, now_ms));
  publish_debug_text("daikin_unknown_fields", format_unknown_fields_(*entry));

  size_t prev_index = 0;
  const RawFrameEntry *prev = find_previous_frame_by_type_(entry->packet_type, entry->seq, prev_index, true);
  if (prev == nullptr) {
    prev = find_previous_frame_by_type_(entry->packet_type, entry->seq, prev_index, false);
  }
  publish_debug_text("daikin_frame_diff", format_frame_diff_(*entry, prev));

  if (dd_b1_text_ != nullptr || dd_b5_text_ != nullptr) {
    size_t dd_index = 0;
    const RawFrameEntry *dd_entry = find_latest_frame_by_type_(DD_PACKET_START_BYTE, dd_index, false);
    const bool has_dd = dd_entry != nullptr && dd_entry->length >= 6 &&
                        (dd_entry->flags & RAW_FRAME_TRUNCATED) == 0;
    if (has_dd) {
      uint8_t b1 = dd_entry->data[1];
      uint8_t b5 = dd_entry->data[5];
      if (dd_b1_text_ != nullptr &&
          (!last_dd_b1_valid_ || b1 != last_dd_b1_) &&
          (now_ms - last_dd_b1_publish_ms_) >= kDebugTextPublishMinIntervalMs) {
        char buffer[8];
        snprintf(buffer, sizeof(buffer), "0x%02X", b1);
        dd_b1_text_->publish_state(buffer);
        last_dd_b1_publish_ms_ = now_ms;
        last_dd_b1_valid_ = true;
      }
      if (dd_b5_text_ != nullptr &&
          (!last_dd_b5_valid_ || b5 != last_dd_b5_) &&
          (now_ms - last_dd_b5_publish_ms_) >= kDebugTextPublishMinIntervalMs) {
        char buffer[8];
        snprintf(buffer, sizeof(buffer), "0x%02X", b5);
        dd_b5_text_->publish_state(buffer);
        last_dd_b5_publish_ms_ = now_ms;
        last_dd_b5_valid_ = true;
      }
      last_dd_b1_ = b1;
      last_dd_b5_ = b5;
    } else {
      if (dd_b1_text_ != nullptr && last_dd_b1_valid_) {
        dd_b1_text_->publish_state("");
        last_dd_b1_valid_ = false;
      }
      if (dd_b5_text_ != nullptr && last_dd_b5_valid_) {
        dd_b5_text_->publish_state("");
        last_dd_b5_valid_ = false;
      }
      last_dd_b1_ = 0xFF;
      last_dd_b5_ = 0xFF;
    }
  }
}

void DaikinEkhheComponent::update_dd_b1_bit_sensors_() {
  if (dd_heating_demand_ == nullptr && hp_active_ == nullptr && eh_active_ == nullptr) {
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
    if (dd_heating_demand_ != nullptr) {
      dd_heating_demand_->publish_state(demand);
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

void DaikinEkhheComponent::publish_cc_snapshot_(const char *override_text) {
  if (cc_snapshot_sensor_ == nullptr || !debug_mode_) {
    return;
  }
  if (override_text != nullptr) {
    cc_snapshot_sensor_->publish_state(override_text);
    return;
  }
  if (!cc_snapshot_valid_ || cc_snapshot_len_ == 0) {
    cc_snapshot_sensor_->publish_state("EMPTY");
    return;
  }
  std::string formatted = format_raw_frame_hex_data_(cc_snapshot_data_, cc_snapshot_len_);
  cc_snapshot_sensor_->publish_state(formatted);
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
    reset_cycle_stats_();
}

void DaikinEkhheComponent::process_packet_set() {
  processing_updates_ = true;

  uint32_t parse_start_ms = millis();

  bool has_required = (cycle_packet_types_seen_ & kRequiredPacketMask) == kRequiredPacketMask;
  cycle_publish_allowed_ = has_required;

  // Assign stored packets to last known packet values
  last_dd_packet_ = latest_packets_[DD_PACKET_START_BYTE];
  last_d2_packet_ = latest_packets_[D2_PACKET_START_BYTE];
  last_d4_packet_ = latest_packets_[D4_PACKET_START_BYTE];
  last_c1_packet_ = latest_packets_[C1_PACKET_START_BYTE];
  last_cc_packet_ = latest_packets_[CC_PACKET_START_BYTE];

  parse_dd_packet(last_dd_packet_);
  cycle_packets_parsed_++;
  //parse_d2_packet(last_d2_packet);  // Don't process D2 since all info is in CC as well
  parse_d4_packet(last_d4_packet_);
  cycle_packets_parsed_++;
  parse_c1_packet(last_c1_packet_);
  cycle_packets_parsed_++;
  parse_cc_packet(last_cc_packet_);
  cycle_packets_parsed_++;

  uint32_t now_ms = millis();
  cycle_parse_ms_ = now_ms - parse_start_ms;
  cycle_total_ms_ = now_ms - cycle_start_ms_;
  if (cycle_total_ms_ > kCycleOverBudgetMs) {
    cycle_over_budget_total_++;
  }
  bool success = has_required && !cycle_timeout_logged_;
  if (success) {
    std::string types = packet_mask_to_string_(cycle_packet_types_seen_);
    DAIKIN_HEALTH(TAG, "Cycle ok: bytes=%u packets=%u parsed=%u types=%s parse_ms=%u chk_err=%u frm_err=%u",
                 cycle_bytes_read_, cycle_packets_seen_, cycle_packets_parsed_, types.c_str(),
                 cycle_parse_ms_, cycle_checksum_errors_, cycle_framing_errors_);
  } else {
    if (!has_required) {
      uint8_t missing_mask = kRequiredPacketMask & ~cycle_packet_types_seen_;
      std::string missing = packet_mask_to_string_(missing_mask);
      DAIKIN_WARN(TAG, "Cycle partial: missing=%s bytes=%u packets=%u crc=%u frame=%u",
                 missing.c_str(), cycle_bytes_read_, cycle_packets_seen_, cycle_checksum_errors_,
                 cycle_framing_errors_);
    }
    if (cycle_framing_errors_ > 0 && !cycle_timeout_logged_) {
      DAIKIN_WARN(TAG, "Cycle warn: framing=%u last_start=0x%02X bytes=%u packets=%u",
                  cycle_framing_errors_, cycle_framing_error_start_, cycle_bytes_read_,
                  cycle_packets_seen_);
    }
    if (!has_required && cycle_checksum_errors_ > 0) {
      std::string types = packet_mask_to_string_(cycle_packet_types_seen_);
      std::string checksum_types = packet_mask_to_string_(cycle_checksum_error_mask_);
      DAIKIN_ERROR(TAG, "Cycle error: checksum=%u checksum_types=%s types=%s bytes=%u packets=%u",
                   cycle_checksum_errors_, checksum_types.c_str(), types.c_str(),
                   cycle_bytes_read_, cycle_packets_seen_);
    }
  }

  publish_debug_outputs_();
  update_dd_b1_bit_sensors_();

  // Reset UART cycle
  processing_updates_ = false;
  last_process_time_ = millis();
  if (pending_tx_.active || tx_ui_sync_.active || (debug_mode_ && continuous_rx_)) {
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
    set_binary_sensor_value(entry.first, entry.second);
  }

  return;
}

void DaikinEkhheComponent::parse_d2_packet(std::vector<uint8_t> buffer) {

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

  return;
}

void DaikinEkhheComponent::parse_d4_packet(std::vector<uint8_t> buffer) {
  // STUB function only
  return;
}

void DaikinEkhheComponent::parse_c1_packet(std::vector<uint8_t> buffer) {
  // STUB function only
  return;
}

void DaikinEkhheComponent::parse_cc_packet(std::vector<uint8_t> buffer) {
  const bool cc_sync_matched =
      tx_ui_sync_.active &&
      field_matches_target_(buffer, tx_ui_sync_.index, tx_ui_sync_.value, tx_ui_sync_.bit_position);

  // update numbers, unsigned and signed separately
  for (const auto &entry : U_NUMBER_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if (pending_tx_.active && pending_tx_.index == param_index &&
        pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    uint8_t value = buffer[param_index];
    set_number_value(param_name, value);
  }

  for (const auto &entry : I_NUMBER_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if (pending_tx_.active && pending_tx_.index == param_index &&
        pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    int8_t value = static_cast<int8_t>(buffer[param_index]);  // cast as signed
    set_number_value(param_name, value);
  }

  // Process Standard Selects (Full-Byte Values)
  for (const auto &entry : SELECT_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if (pending_tx_.active && pending_tx_.index == param_index &&
        pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    uint8_t value = buffer[param_index];
    set_select_value(param_name, value);
  }

  // Process Bitmask-Based Selects (Modify Only One Bit)
  for (const auto &entry : SELECT_BITMASKS) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second.first;  // The byte index
    uint8_t bit_position = entry.second.second;  // The specific bit to extract
    if (pending_tx_.active && pending_tx_.index == param_index &&
        pending_tx_.bit_position == bit_position) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == bit_position && !cc_sync_matched) {
      continue;
    }
    uint8_t bit_value = (buffer[param_index] >> bit_position) & 0x01;  // Extract the bit
    set_select_value(param_name, bit_value);
  }

  update_timestamp(buffer[CC_PACKET_HOUR_IDX], buffer[CC_PACKET_MIN_IDX]);

  if (tx_ui_sync_.active) {
    if (cc_sync_matched) {
      DAIKIN_DBG(TAG, "TX UI synced: index=%u bit=%u value=0x%02X cc_cycles=%u",
                 tx_ui_sync_.index, tx_ui_sync_.bit_position, tx_ui_sync_.value,
                 tx_ui_sync_.cycles_waited + 1);
      tx_ui_sync_.active = false;
      tx_ui_sync_.cycles_waited = 0;
    } else {
      tx_ui_sync_.cycles_waited++;
      if (tx_ui_sync_.cycles_waited >= kTxUiSyncMaxCycles) {
        DAIKIN_WARN(TAG, "TX UI sync timeout: index=%u bit=%u value=0x%02X cycles=%u",
                    tx_ui_sync_.index, tx_ui_sync_.bit_position, tx_ui_sync_.value,
                    tx_ui_sync_.cycles_waited);
        tx_ui_sync_.active = false;
        tx_ui_sync_.cycles_waited = 0;
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
  }
}

void DaikinEkhheComponent::register_select(const std::string &select_name, select::Select *select) {
  if (select != nullptr) {
      //selects_[select_name] = select;
      selects_[select_name] = static_cast<DaikinEkhheSelect *>(select);
  }
}

void DaikinEkhheComponent::register_timestamp_sensor(text_sensor::TextSensor *sensor) {
    this->timestamp_sensor_ = sensor;
}

void DaikinEkhheComponent::register_debug_text_sensor(const std::string &sensor_name,
                                                      esphome::text_sensor::TextSensor *sensor) {
  if (sensor != nullptr) {
    debug_text_sensors_[sensor_name] = sensor;
  }
}

void DaikinEkhheComponent::register_debug_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor) {
  if (sensor != nullptr) {
    debug_sensors_[sensor_name] = sensor;
  }
}

void DaikinEkhheComponent::register_debug_select(DaikinEkhheDebugSelect *select) {
  this->debug_packet_select_ = select;
  if (this->debug_packet_select_ != nullptr) {
    this->debug_packet_select_->publish_state(packet_type_to_string_(debug_packet_type_));
  }
}

#if DAIKIN_EKHHE_DEBUG && defined(USE_SWITCH)
void DaikinEkhheComponent::register_debug_switch(DaikinEkhheDebugSwitch *sw) {
  this->debug_freeze_switch_ = sw;
  if (this->debug_freeze_switch_ != nullptr) {
    this->debug_freeze_switch_->publish_state(debug_freeze_);
  }
}
#endif

void DaikinEkhheComponent::register_cc_snapshot_sensor(esphome::text_sensor::TextSensor *sensor) {
  this->cc_snapshot_sensor_ = sensor;
  if (this->cc_snapshot_sensor_ != nullptr) {
    publish_cc_snapshot_(nullptr);
  }
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

uint8_t DaikinEkhheComponent::ekhhe_checksum(const std::vector<uint8_t>& data_bytes) {
  // Compute the checksum as (sum of data bytes) mod 256 + 170
  uint16_t sum = std::accumulate(data_bytes.begin(), data_bytes.end()-1, 0);
  return (sum % 256 + 170) & 0xFF;
}

void DaikinEkhheComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Daikin EKHHE:");
    ESP_LOGCONFIG(TAG, "  Update interval: %lu ms", this->update_interval_);
    ESP_LOGCONFIG(TAG, "  Debug mode: %s", YESNO(debug_mode_));
    ESP_LOGCONFIG(TAG, "  Continuous RX: %s", YESNO(this->continuous_rx_));

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

void DaikinEkhheComponent::set_debug_packet(const std::string &value) {
  debug_packet_type_ = packet_type_from_string_(value);
  if (debug_packet_select_ != nullptr) {
    debug_packet_select_->publish_state(packet_type_to_string_(debug_packet_type_));
  }
}

void DaikinEkhheComponent::set_debug_freeze(bool enabled) {
  debug_freeze_ = enabled;
  if (debug_freeze_) {
    size_t index = 0;
    size_t back = 0;
    const RawFrameEntry *entry = select_raw_frame_(index, back);
    debug_frozen_seq_ = entry != nullptr ? entry->seq : 0;
  } else {
    debug_frozen_seq_ = 0;
  }
#if DAIKIN_EKHHE_DEBUG && defined(USE_SWITCH)
  if (debug_freeze_switch_ != nullptr) {
    debug_freeze_switch_->publish_state(debug_freeze_);
  }
#endif
}

void DaikinEkhheComponent::save_cc_snapshot() {
  if (!debug_mode_) {
    return;
  }
  size_t index = 0;
  const RawFrameEntry *entry = find_latest_frame_by_type_(CC_PACKET_START_BYTE, index, true);
  if (entry == nullptr) {
    cc_snapshot_valid_ = false;
    cc_snapshot_len_ = 0;
    publish_cc_snapshot_("EMPTY");
    return;
  }
  cc_snapshot_len_ = entry->length;
  if (cc_snapshot_len_ > kRawFrameMaxLen) {
    cc_snapshot_len_ = kRawFrameMaxLen;
  }
  std::memcpy(cc_snapshot_data_, entry->data, cc_snapshot_len_);
  cc_snapshot_ts_ms_ = millis();
  cc_snapshot_valid_ = true;
  publish_cc_snapshot_(nullptr);
}

void DaikinEkhheComponent::restore_cc_snapshot() {
  if (!debug_mode_) {
    return;
  }
  if (!cc_snapshot_valid_ || cc_snapshot_len_ == 0) {
    publish_cc_snapshot_("EMPTY");
    return;
  }
  std::vector<uint8_t> packet(cc_snapshot_data_, cc_snapshot_data_ + cc_snapshot_len_);
  send_uart_cc_packet_(packet, false, 0, 0, BIT_POSITION_NO_BITMASK);
  publish_cc_snapshot_(nullptr);
}

void DaikinEkhheDebugSelect::control(const std::string &value) {
  if (this->parent_ == nullptr) {
    return;
  }
  this->parent_->set_debug_packet(value);
}

#if DAIKIN_EKHHE_DEBUG && defined(USE_SWITCH)
void DaikinEkhheDebugSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    return;
  }
  this->parent_->set_debug_freeze(state);
}
#endif

#if DAIKIN_EKHHE_DEBUG && defined(USE_BUTTON)
void DaikinEkhheDebugButton::press_action() {
  if (this->parent_ == nullptr) {
    return;
  }
  if (this->action_ == Action::SAVE_SNAPSHOT) {
    this->parent_->save_cc_snapshot();
  } else if (this->action_ == Action::RESTORE_SNAPSHOT) {
    this->parent_->restore_cc_snapshot();
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

    // Get the CC array index from map, send UART command and update UI state
    auto it_u = U_NUMBER_PARAM_INDEX.find(name);
    auto it_i = I_NUMBER_PARAM_INDEX.find(name);
    if (it_u != U_NUMBER_PARAM_INDEX.end()) {
        uint8_t index = it_u->second;
        this->parent_->send_uart_cc_command(index, (uint8_t)value, BIT_POSITION_NO_BITMASK);
        this->parent_->update_number_cache(name, value);
        this->publish_state(value);
    } 
    else if (it_i != I_NUMBER_PARAM_INDEX.end()) {
        uint8_t index = it_i->second;
        this->parent_->send_uart_cc_command(index, (int8_t)value, BIT_POSITION_NO_BITMASK);
        this->parent_->update_number_cache(name, value);
        this->publish_state(value);
    }
    else {
        DAIKIN_WARN(TAG, "No matching UART command for Number: %s", name.c_str());
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
  
    // Look up the CC packet index for this select entity
    auto index_it = SELECT_PARAM_INDEX.find(name);
    uint8_t param_index = PARAM_INDEX_INVALID;
    if (index_it != SELECT_PARAM_INDEX.end()) {
      param_index = index_it->second;
    } else {
      DAIKIN_WARN(TAG, "Select %s NOT found in SELECT_PARAM_INDEX", name.c_str());
    }

    // Look up if this select is part of a bitmask
    uint8_t bit_position = BIT_POSITION_NO_BITMASK;  // Default to no bitmask
    auto bitmask_it = SELECT_BITMASKS.find(name);
    if (bitmask_it != SELECT_BITMASKS.end()) {
        param_index = bitmask_it->second.first;  // Get the byte index
        bit_position = bitmask_it->second.second;  // Get the bit position
    } else {
            DAIKIN_DBG(TAG, "Select %s not in SELECT_BITMASKS", name.c_str());
    }

    // Update value in ESPHome
    this->parent_->send_uart_cc_command(param_index, uart_value, bit_position);
    this->parent_->update_select_cache(name, value);
    this->publish_state(value);
}

void DaikinEkhheComponent::send_uart_cc_packet_(const std::vector<uint8_t> &base_packet, bool apply_change,
                                                uint8_t index, uint8_t value, uint8_t bit_position) {
    if (base_packet.empty()) {
        DAIKIN_WARN(TAG, "Base CC packet empty, cannot send.");
        return;
    }
    if (base_packet.size() < CD_PACKET_SIZE) {
        DAIKIN_WARN(TAG, "Base CC packet length %u invalid, cannot send.", static_cast<unsigned>(base_packet.size()));
        return;
    }

    unsigned long time_since_last_rx = millis() - last_rx_time_;

    // UART flow control
    uart_active_ = false;
    uart_tx_active_ = true;

    // Construct command packet
    std::vector<uint8_t> command = base_packet;
    command[0] = CD_PACKET_START_BYTE;

    if (apply_change) {
      // Reconstruct array byte depending on bitmask or not
      if (bit_position != BIT_POSITION_NO_BITMASK) {
        uint8_t current_value = command[index];

        // Clear the specific bit and set to selected value
        current_value &= ~(1 << bit_position);
        current_value |= (value << bit_position);
        command[index] = current_value;
      } else {
        // If it's a normal parameter, just assign the value
      command[index] = value;
    }
    }

    command.back() = ekhhe_checksum(command);

    size_t cc_index = 0;
    const RawFrameEntry *base_cc_entry = find_latest_frame_by_type_(CC_PACKET_START_BYTE, cc_index, true);
    uint32_t base_cc_age_ms = base_cc_entry != nullptr ? (millis() - base_cc_entry->timestamp_ms) : 0;
    uint32_t tx_start_ms = millis();

    size_t dd_index = 0;
    size_t c1_index = 0;
    size_t d2_index = 0;
    const RawFrameEntry *latest_dd_entry = find_latest_frame_by_type_(DD_PACKET_START_BYTE, dd_index, true);
    const RawFrameEntry *latest_c1_entry = find_latest_frame_by_type_(C1_PACKET_START_BYTE, c1_index, true);
    const RawFrameEntry *latest_d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);

    if (debug_mode_ && apply_change) {
      auto age_or_zero = [tx_start_ms](const RawFrameEntry *entry) -> uint32_t {
        return entry != nullptr ? (tx_start_ms - entry->timestamp_ms) : 0;
      };

      DAIKIN_DBG(TAG, "TX start: since_dd=%u since_c1=%u since_d2=%u since_cc=%u base_cc_age=%u len=%u",
                 age_or_zero(latest_dd_entry), age_or_zero(latest_c1_entry), age_or_zero(latest_d2_entry),
                 age_or_zero(base_cc_entry), base_cc_age_ms, static_cast<unsigned>(command.size()));
    }

    // Send the updated packet over UART
    this->write_array(command);
    this->flush();
    tx_sent_ms_ = millis();
    tx_waiting_for_first_rx_ = apply_change;
    tx_waiting_for_first_cc_ = apply_change;

    if (apply_change) {
      ESP_LOGI(TAG, "TX CD sent: index=%u value=0x%02X bit=%u len=%u",
               index, value, bit_position, static_cast<unsigned>(command.size()));
      DAIKIN_DBG(TAG, "TX timing: request_to_send=%u since_last_rx=%u base_cc_age=%u",
                 tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_cc_age_ms);
    } else {
      ESP_LOGI(TAG, "TX CD sent: snapshot len=%u", static_cast<unsigned>(command.size()));
      DAIKIN_DBG(TAG, "TX timing: snapshot_send since_last_rx=%u base_cc_age=%u", time_since_last_rx,
                 base_cc_age_ms);
    }

    // Trigger a new read cycle w/ small timeout
    set_timeout(10, [this]() {
          uart_tx_active_ = false;
          start_uart_cycle();
    });
}

void DaikinEkhheComponent::check_pending_tx_(const std::vector<uint8_t> &buffer) {
  if (!pending_tx_.active) {
    return;
  }
  if (pending_tx_.index >= buffer.size()) {
    pending_tx_.active = false;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    queued_tx_.active = false;
    queued_tx_.scheduled = false;
    return;
  }

  bool matched = field_matches_target_(buffer, pending_tx_.index, pending_tx_.value, pending_tx_.bit_position);

  if (matched) {
    if (pending_tx_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "TX already current: index=%u value=0x%02X bit=%u",
                 pending_tx_.index, pending_tx_.value, pending_tx_.bit_position);
    } else {
      bool retried = pending_tx_.attempts_sent > 1;
      if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: index=%u value=0x%02X attempts=%u",
                      pending_tx_.index, pending_tx_.value, pending_tx_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: index=%u value=0x%02X",
                   pending_tx_.index, pending_tx_.value);
        }
      } else {
        uint8_t bit = (buffer[pending_tx_.index] >> pending_tx_.bit_position) & 0x01;
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: index=%u bit=%u value=%u attempts=%u",
                      pending_tx_.index, pending_tx_.bit_position, bit, pending_tx_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: index=%u bit=%u value=%u",
                   pending_tx_.index, pending_tx_.bit_position, bit);
        }
      }
      DAIKIN_DBG(TAG, "TX timing: applied_after=%u attempts=%u", millis() - tx_sent_ms_, pending_tx_.attempts_sent);
      tx_ui_sync_.active = true;
      tx_ui_sync_.index = pending_tx_.index;
      tx_ui_sync_.value = pending_tx_.value;
      tx_ui_sync_.bit_position = pending_tx_.bit_position;
      tx_ui_sync_.cycles_waited = 0;
    }
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    pending_tx_.active = false;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    queued_tx_.active = false;
    queued_tx_.scheduled = false;
    return;
  }

  if (pending_tx_.attempts_sent == 0) {
    return;
  }

  if (pending_tx_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "TX retry pending: index=%u attempt=%u/%u",
               pending_tx_.index, pending_tx_.attempts_sent, kTxMaxRepeats);
    return;
  }

  if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
    DAIKIN_WARN(TAG, "TX not applied: index=%u expected=0x%02X current=0x%02X",
                pending_tx_.index, pending_tx_.value, buffer[pending_tx_.index]);
  } else {
    uint8_t bit = (buffer[pending_tx_.index] >> pending_tx_.bit_position) & 0x01;
    DAIKIN_WARN(TAG, "TX not applied: index=%u bit=%u expected=%u current=%u",
                pending_tx_.index, pending_tx_.bit_position,
                pending_tx_.value & 0x01, bit);
  }
  DAIKIN_DBG(TAG, "TX timing: failed_after=%u attempts=%u", millis() - tx_sent_ms_, pending_tx_.attempts_sent);
  uint8_t index = pending_tx_.index;
  if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
    for (const auto &entry : U_NUMBER_PARAM_INDEX) {
      if (entry.second == index) {
        set_number_value(entry.first, buffer[index]);
      }
    }
    for (const auto &entry : I_NUMBER_PARAM_INDEX) {
      if (entry.second == index) {
        set_number_value(entry.first, static_cast<int8_t>(buffer[index]));
      }
    }
    for (const auto &entry : SELECT_PARAM_INDEX) {
      if (entry.second == index) {
        set_select_value(entry.first, buffer[index]);
      }
    }
  } else {
    uint8_t bit = (buffer[index] >> pending_tx_.bit_position) & 0x01;
    for (const auto &entry : SELECT_BITMASKS) {
      if (entry.second.first == index && entry.second.second == pending_tx_.bit_position) {
        set_select_value(entry.first, bit);
      }
    }
  }
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
  pending_tx_.active = false;
  pending_tx_.attempts_sent = 0;
  pending_tx_.last_attempt_d2_seq = 0;
  tx_ui_sync_.active = false;
  tx_ui_sync_.cycles_waited = 0;
  queued_tx_.active = false;
  queued_tx_.scheduled = false;
}

void DaikinEkhheComponent::send_uart_cc_command(uint8_t index, uint8_t value, uint8_t bit_position) {
    // Check that a CC packet is stored
    // since we need the last one as a basis for the new packet
    if (last_cc_packet_.empty()) {
        DAIKIN_WARN(TAG, "No CC packet received yet. Cannot send command.");
        return;
    }

    tx_request_ms_ = millis();
    if (debug_mode_) {
      const uint32_t now_ms = tx_request_ms_;
      const uint32_t since_last_frame = last_frame_profile_ms_ == 0 ? 0 : (now_ms - last_frame_profile_ms_);
      const uint32_t since_last_cc = last_cc_profile_ms_ == 0 ? 0 : (now_ms - last_cc_profile_ms_);
      const std::string last_type =
          last_frame_profile_ms_ == 0 ? "none" : packet_type_to_string_(last_frame_profile_type_);
      size_t d2_phase_index = 0;
      const RawFrameEntry *d2_phase_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_phase_index, true);
      const uint32_t since_last_d2 = d2_phase_entry != nullptr ? (now_ms - d2_phase_entry->timestamp_ms) : 0;
      std::string cycle_types = packet_mask_to_string_(cycle_packet_types_seen_);
      DAIKIN_DBG(TAG,
                 "TX request phase: index=%u value=0x%02X bit=%u last=%s since_last=%u since_d2=%u since_cc=%u cycle_ms=%u cycle_types=%s uart_active=%u",
                 index, value, bit_position, last_type.c_str(), since_last_frame, since_last_d2, since_last_cc,
                 now_ms - cycle_start_ms_, cycle_types.c_str(), uart_active_);
    }
    pending_tx_.active = true;
    pending_tx_.index = index;
    pending_tx_.value = value;
    pending_tx_.bit_position = bit_position;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    tx_ui_sync_.active = false;
    tx_ui_sync_.cycles_waited = 0;

    queued_tx_.active = true;
    queued_tx_.scheduled = false;
    queued_tx_.index = index;
    queued_tx_.value = value;
    queued_tx_.bit_position = bit_position;
    queued_tx_.generation++;
    queued_tx_.request_ms = tx_request_ms_;
    queued_tx_.anchor_ms = 0;
    queued_tx_.anchor_seq = 0;
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;

    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }

    size_t d2_index = 0;
    const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
    if (d2_entry != nullptr) {
      uint32_t d2_age_ms = millis() - d2_entry->timestamp_ms;
      if (d2_age_ms <= kTxDelayAfterD2Ms) {
        schedule_queued_tx_from_d2_(*d2_entry);
        return;
      }
    }

    DAIKIN_DBG(TAG, "TX scheduling: waiting_for_next_d2 index=%u value=0x%02X bit=%u",
               index, value, bit_position);
}


}  // namespace daikin_ekkhe
}  // namespace esphome
