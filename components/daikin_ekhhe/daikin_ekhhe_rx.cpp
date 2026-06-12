#include "daikin_ekhhe.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"
#include "daikin_ekhhe_metadata.h"

#include <cstring>
#include <string>
#include <vector>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

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

  // Store only the latest version of each valid packet.
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

  // Assign stored packets to last known packet values.
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

  // Reset UART cycle.
  processing_updates_ = false;
  last_process_time_ = millis();
  if (any_tx_or_ui_sync_active_() || continuous_rx_) {
    start_uart_cycle();
  } else {
    uart_active_ = false;
  }
}

}  // namespace daikin_ekkhe
}  // namespace esphome
