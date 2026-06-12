#include "daikin_ekhhe.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"
#include "daikin_ekhhe_metadata.h"

#include <cstdio>
#include <string>
#include <vector>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

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
#if defined(USE_WATER_HEATER)
  if (water_heater_tx_active_()) {
    return TxOperationKind::WATER_HEATER;
  }
#endif
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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER:
      return "water_heater";
#endif
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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER:
      return water_heater_tx_().family;
#endif
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

#if defined(USE_WATER_HEATER)
bool DaikinEkhheComponent::water_heater_tx_active_() const {
  return tx_operation_.kind == TxOperationKind::WATER_HEATER;
}

DaikinEkhheComponent::WaterHeaterTxPayload &DaikinEkhheComponent::water_heater_tx_() {
  return tx_operation_.water_heater;
}

const DaikinEkhheComponent::WaterHeaterTxPayload &DaikinEkhheComponent::water_heater_tx_() const {
  return tx_operation_.water_heater;
}

bool DaikinEkhheComponent::water_heater_tx_busy_() const {
  return water_heater_tx_active_() || queued_water_heater_tx_.active ||
         queued_water_heater_tx_.scheduled || tx_ui_sync_active_(TxOperationKind::WATER_HEATER);
}
#endif

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
  bool busy = single_field_tx_busy_() || restore_tx_busy_() || profile_restore_tx_busy_() ||
              time_band_tx_busy_();
#if defined(USE_WATER_HEATER)
  busy = busy || water_heater_tx_busy_();
#endif
  return busy;
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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::start_water_heater_ui_sync_(const WaterHeaterTxPayload &target) {
  reset_tx_ui_sync_();
  tx_ui_sync_.kind = TxOperationKind::WATER_HEATER;
  tx_ui_sync_.water_heater = target;
}
#endif

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
#if defined(USE_WATER_HEATER)
  tx_operation_.water_heater = WaterHeaterTxPayload{};
#endif
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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::reset_water_heater_tx_lifecycle_(bool clear_ui_sync) {
  reset_pending_water_heater_();
  reset_queued_water_heater_();
  if (clear_ui_sync && tx_ui_sync_active_(TxOperationKind::WATER_HEATER)) {
    reset_tx_ui_sync_();
  }
}
#endif

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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER:
      reset_water_heater_tx_lifecycle_(clear_ui_sync);
      break;
#endif
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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::reset_pending_water_heater_() {
  reset_tx_operation_();
}
#endif

void DaikinEkhheComponent::reset_queued_time_band_() {
  queued_time_band_tx_.active = false;
  queued_time_band_tx_.scheduled = false;
  queued_time_band_tx_.anchor_ms = 0;
  queued_time_band_tx_.anchor_seq = 0;
}

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::reset_queued_water_heater_() {
  queued_water_heater_tx_.active = false;
  queued_water_heater_tx_.scheduled = false;
  queued_water_heater_tx_.anchor_ms = 0;
  queued_water_heater_tx_.anchor_seq = 0;
}
#endif

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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER: {
      if (!water_heater_tx_active_() || queued_water_heater_tx_.scheduled) {
        return false;
      }
      generation = queued_water_heater_tx_.generation;
      queued_water_heater_tx_.active = true;
      queued_water_heater_tx_.scheduled = true;
      queued_water_heater_tx_.anchor_ms = d2_entry.timestamp_ms;
      queued_water_heater_tx_.anchor_seq = d2_entry.seq;

      DAIKIN_DBG(TAG, "Native water heater scheduling: d2_seq=%u d2_age=%u delay=%u request_age=%u",
                 d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_water_heater_tx_.request_ms);
      break;
    }
#endif
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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER:
      return queued_water_heater_tx_.active && queued_water_heater_tx_.generation == generation;
#endif
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
#if defined(USE_WATER_HEATER)
    case TxOperationKind::WATER_HEATER: {
      const uint32_t anchor_seq = queued_water_heater_tx_.anchor_seq;
      const uint32_t anchor_ms = queued_water_heater_tx_.anchor_ms;
      queued_water_heater_tx_.active = false;
      queued_water_heater_tx_.scheduled = false;

      std::vector<uint8_t> base_packet = last_cc_packet_;
      auto latest_cc = latest_packets_.find(CC_PACKET_START_BYTE);
      if (latest_cc != latest_packets_.end()) {
        base_packet = latest_cc->second;
      }

      tx_operation_.attempts_sent++;
      tx_operation_.last_attempt_d2_seq = anchor_seq;

      DAIKIN_DBG(TAG,
                 "Native water heater scheduling: sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_cc=%u",
                 anchor_seq, millis() - anchor_ms, tx_operation_.attempts_sent, kTxMaxRepeats,
                 latest_cc != latest_packets_.end());
      send_water_heater_packet_(base_packet);
      return;
    }
#endif
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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::schedule_queued_water_heater_from_d2_(const RawFrameEntry &d2_entry) {
  schedule_tx_after_d2_(TxOperationKind::WATER_HEATER, d2_entry);
}
#endif

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

#if defined(USE_WATER_HEATER)
bool DaikinEkhheComponent::prepare_water_heater_tx_payload_(WaterHeaterTxPayload &payload,
                                                            std::string &reason) const {
  payload = WaterHeaterTxPayload{};
  payload.family = TxPacketFamily::MAIN;
  const auto &spec = tx_packet_family_spec_(payload.family);

  auto latest_base = latest_packets_.find(spec.base_packet_type);
  if (latest_base != latest_packets_.end()) {
    payload.base_packet = latest_base->second;
  } else {
    payload.base_packet = last_cc_packet_;
  }

  if (payload.base_packet.empty()) {
    reason = "no CC base packet captured";
    return false;
  }
  if (payload.base_packet.size() != spec.packet_size) {
    char msg[96];
    snprintf(msg, sizeof(msg), "CC base packet size %u is not expected %u",
             static_cast<unsigned>(payload.base_packet.size()), static_cast<unsigned>(spec.packet_size));
    reason = msg;
    return false;
  }
  if (payload.base_packet[0] != spec.base_packet_type) {
    char msg[80];
    snprintf(msg, sizeof(msg), "CC base start byte 0x%02X is not %s",
             payload.base_packet[0], packet_type_to_string_(spec.base_packet_type).c_str());
    reason = msg;
    return false;
  }

  return true;
}

bool DaikinEkhheComponent::add_water_heater_tx_field_(WaterHeaterTxPayload &payload,
                                                      const char *name,
                                                      uint8_t write_index,
                                                      uint8_t write_value,
                                                      uint8_t write_bit_position,
                                                      uint8_t write_bit_width) {
  if (payload.family != TxPacketFamily::MAIN) {
    return false;
  }
  const uint8_t effective_width = effective_bit_width(write_bit_position, write_bit_width);
  if (!payload.base_packet.empty() && write_index >= payload.base_packet.size()) {
    return false;
  }

  WaterHeaterTxField field;
  field.name = name;
  field.write_index = write_index;
  field.write_value = write_value;
  field.write_bit_position = write_bit_position;
  field.write_bit_width = effective_width;
  field.readback_index = tx_readback_index_(payload.family, write_index, write_bit_position);
  field.readback_value = write_value;
  field.readback_bit_position = tx_readback_bit_position_(payload.family, write_index, write_bit_position);
  field.readback_bit_width = tx_readback_bit_width_(payload.family, write_index,
                                                    write_bit_position, effective_width);

  for (auto &existing : payload.fields) {
    if (existing.write_index == field.write_index &&
        existing.write_bit_position == field.write_bit_position) {
      existing = field;
      return true;
    }
  }

  payload.fields.push_back(field);
  return true;
}

bool DaikinEkhheComponent::water_heater_add_power_field_(WaterHeaterTxPayload &payload, bool power_on) {
  const uint8_t value = power_on ? 1 : 0;
  if (field_matches_target_(payload.base_packet, CC_PACKET_MASK1_IDX, value, 0, 1)) {
    return true;
  }
  return add_water_heater_tx_field_(payload, "power", CC_PACKET_MASK1_IDX, value, 0, 1);
}

bool DaikinEkhheComponent::water_heater_add_mode_field_(WaterHeaterTxPayload &payload, uint8_t mode) {
  if (field_matches_target_(payload.base_packet, CC_PACKET_MODE_IDX, mode, BIT_POSITION_NO_BITMASK, 1)) {
    return true;
  }
  return add_water_heater_tx_field_(payload, "mode", CC_PACKET_MODE_IDX, mode,
                                    BIT_POSITION_NO_BITMASK, 1);
}

bool DaikinEkhheComponent::water_heater_add_target_field_(WaterHeaterTxPayload &payload,
                                                          uint8_t mode,
                                                          uint8_t target) {
  uint8_t index = CC_PACKET_AUTO_TTARGET_IDX;
  const char *name = "auto_target";
  switch (mode) {
    case kOperationalModeEco:
      index = CC_PACKET_ECO_TTARGET_IDX;
      name = "eco_target";
      break;
    case kOperationalModeBoost:
      index = CC_PACKET_BOOST_TTGARGET_IDX;
      name = "boost_target";
      break;
    case kOperationalModeElectric:
      index = CC_PACKET_ELECTRIC_TTARGET_IDX;
      name = "electric_target";
      break;
    case kOperationalModeAuto:
      break;
    default:
      return false;
  }
  if (field_matches_target_(payload.base_packet, index, target, BIT_POSITION_NO_BITMASK, 1)) {
    return true;
  }
  return add_water_heater_tx_field_(payload, name, index, target, BIT_POSITION_NO_BITMASK, 1);
}

bool DaikinEkhheComponent::build_water_heater_tx_command_(const WaterHeaterTxPayload &payload,
                                                          std::vector<uint8_t> &command,
                                                          std::string &reason) {
  if (payload.family != TxPacketFamily::MAIN) {
    reason = "water-heater transactions are only valid for the main packet family";
    return false;
  }
  const auto &spec = tx_packet_family_spec_(payload.family);
  if (payload.base_packet.empty()) {
    reason = "base packet empty";
    return false;
  }
  if (payload.base_packet.size() != spec.packet_size) {
    char msg[96];
    snprintf(msg, sizeof(msg), "base packet size %u is not expected %u",
             static_cast<unsigned>(payload.base_packet.size()), static_cast<unsigned>(spec.packet_size));
    reason = msg;
    return false;
  }
  if (payload.base_packet[0] != spec.base_packet_type) {
    char msg[80];
    snprintf(msg, sizeof(msg), "base start byte 0x%02X is not %s",
             payload.base_packet[0], packet_type_to_string_(spec.base_packet_type).c_str());
    reason = msg;
    return false;
  }
  if (payload.fields.empty()) {
    reason = "no transaction fields";
    return false;
  }

  command = payload.base_packet;
  command[0] = spec.write_packet_type;
  for (const auto &field : payload.fields) {
    if (field.write_index >= command.size()) {
      char msg[96];
      snprintf(msg, sizeof(msg), "field %s write index %u out of range",
               field.name, field.write_index);
      reason = msg;
      return false;
    }
    if (field.write_bit_position == BIT_POSITION_NO_BITMASK) {
      command[field.write_index] = field.write_value;
    } else {
      apply_field_value(command, field.write_index, field.write_bit_position,
                        effective_bit_width(field.write_bit_position, field.write_bit_width),
                        field.write_value);
    }
  }
  command.back() = ekhhe_checksum(command);

  return validate_water_heater_tx_command_(payload, command, reason);
}

bool DaikinEkhheComponent::water_heater_tx_matches_packet_(const WaterHeaterTxPayload &payload,
                                                           const std::vector<uint8_t> &buffer,
                                                           const WaterHeaterTxField **first_mismatch) const {
  if (first_mismatch != nullptr) {
    *first_mismatch = nullptr;
  }
  for (const auto &field : payload.fields) {
    if (field.readback_index >= buffer.size() ||
        !field_matches_target_(buffer, field.readback_index, field.readback_value,
                               field.readback_bit_position, field.readback_bit_width)) {
      if (first_mismatch != nullptr) {
        *first_mismatch = &field;
      }
      return false;
    }
  }
  return !payload.fields.empty();
}

bool DaikinEkhheComponent::water_heater_tx_matches_base_packet_(
    const WaterHeaterTxPayload &payload, const std::vector<uint8_t> &buffer,
    const WaterHeaterTxField **first_mismatch) const {
  if (first_mismatch != nullptr) {
    *first_mismatch = nullptr;
  }
  for (const auto &field : payload.fields) {
    if (field.write_index >= buffer.size() ||
        !field_matches_target_(buffer, field.write_index, field.write_value,
                               field.write_bit_position, field.write_bit_width)) {
      if (first_mismatch != nullptr) {
        *first_mismatch = &field;
      }
      return false;
    }
  }
  return !payload.fields.empty();
}

bool DaikinEkhheComponent::validate_water_heater_tx_command_(const WaterHeaterTxPayload &payload,
                                                             const std::vector<uint8_t> &command,
                                                             std::string &reason) {
  if (payload.family != TxPacketFamily::MAIN) {
    reason = "water-heater transactions are only valid for the main packet family";
    return false;
  }
  const auto &spec = tx_packet_family_spec_(payload.family);
  if (payload.base_packet.empty() || command.empty()) {
    reason = "base or command packet empty";
    return false;
  }
  if (payload.base_packet.size() != command.size()) {
    char msg[96];
    snprintf(msg, sizeof(msg), "size mismatch base=%u command=%u",
             static_cast<unsigned>(payload.base_packet.size()), static_cast<unsigned>(command.size()));
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
  if (payload.base_packet[0] != spec.base_packet_type) {
    char msg[80];
    snprintf(msg, sizeof(msg), "base start byte 0x%02X is not %s",
             payload.base_packet[0], packet_type_to_string_(spec.base_packet_type).c_str());
    reason = msg;
    return false;
  }
  if (command[0] != spec.write_packet_type) {
    char msg[80];
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

  std::vector<uint8_t> expected = payload.base_packet;
  expected[0] = spec.write_packet_type;
  for (const auto &field : payload.fields) {
    if (field.write_index >= expected.size()) {
      char msg[96];
      snprintf(msg, sizeof(msg), "field %s write index %u out of range",
               field.name, field.write_index);
      reason = msg;
      return false;
    }
    if (field.write_bit_position == BIT_POSITION_NO_BITMASK) {
      expected[field.write_index] = field.write_value;
    } else {
      apply_field_value(expected, field.write_index, field.write_bit_position,
                        effective_bit_width(field.write_bit_position, field.write_bit_width),
                        field.write_value);
    }
  }
  expected.back() = ekhhe_checksum(expected);

  if (expected == command) {
    return true;
  }

  size_t first_diff = 0;
  size_t diff_count = 0;
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
           "water-heater command changed unexpected bytes count=%u first_byte=%u expected=0x%02X actual=0x%02X",
           static_cast<unsigned>(diff_count), static_cast<unsigned>(first_diff),
           expected[first_diff], command[first_diff]);
  reason = msg;
  return false;
}
#endif

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
#if defined(USE_WATER_HEATER)
  } else if (kind == TxPacketKind::WATER_HEATER) {
    const auto &target = water_heater_tx_();
    ESP_LOGI(TAG, "TX water heater sent: fields=%u attempt=%u/%u len=%u",
             static_cast<unsigned>(target.fields.size()), attempts_sent, kTxMaxRepeats,
             static_cast<unsigned>(command.size()));
    DAIKIN_DBG(TAG, "TX timing: water_heater_request_to_send=%u since_last_rx=%u base_age=%u",
               tx_sent_ms_ - tx_request_ms_, time_since_last_rx, base_age_ms);
#endif
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
#if defined(USE_WATER_HEATER)
  } else if (kind == TxPacketKind::WATER_HEATER) {
    if (!water_heater_tx_active_()) {
      reason = "no active water-heater transaction";
      return false;
    }
    return validate_water_heater_tx_command_(water_heater_tx_(), command, reason);
#endif
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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::send_water_heater_packet_(const std::vector<uint8_t> &base_packet) {
  if (!water_heater_tx_active_()) {
    return;
  }
  if (base_packet.empty()) {
    DAIKIN_WARN(TAG, "Native water heater TX blocked: base CC packet empty.");
    reset_tx_lifecycle_(TxOperationKind::WATER_HEATER, false);
    return;
  }
  if (base_packet.size() != CC_PACKET_SIZE) {
    DAIKIN_WARN(TAG, "Native water heater TX blocked: base CC packet length %u invalid.",
                static_cast<unsigned>(base_packet.size()));
    reset_tx_lifecycle_(TxOperationKind::WATER_HEATER, false);
    return;
  }

  auto &target = water_heater_tx_();
  target.base_packet = base_packet;

  std::vector<uint8_t> command;
  std::string validation_error;
  if (!build_water_heater_tx_command_(target, command, validation_error)) {
    DAIKIN_WARN(TAG, "Native water heater TX blocked: %s", validation_error.c_str());
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::WATER_HEATER, false);
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(TxPacketFamily::MAIN, command, TxPacketKind::WATER_HEATER, 0, 0,
                           BIT_POSITION_NO_BITMASK, 0, tx_operation_.attempts_sent);
}
#endif

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

#if defined(USE_WATER_HEATER)
void DaikinEkhheComponent::check_pending_water_heater_(const std::vector<uint8_t> &buffer) {
  if (!water_heater_tx_active_()) {
    return;
  }

  const auto &target = water_heater_tx_();
  const auto &spec = tx_packet_family_spec_(target.family);
  const WaterHeaterTxField *first_mismatch = nullptr;
  const bool matched = water_heater_tx_matches_packet_(target, buffer, &first_mismatch);

  if (matched) {
    if (tx_operation_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "Native water heater already current: fields=%u",
                 static_cast<unsigned>(target.fields.size()));
    } else if (tx_operation_.attempts_sent > 1) {
      DAIKIN_WARN(TAG, "Native water heater applied after retries: fields=%u attempts=%u",
                  static_cast<unsigned>(target.fields.size()), tx_operation_.attempts_sent);
      DAIKIN_DBG(TAG, "TX timing: water_heater_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    } else {
      ESP_LOGI(TAG, "Native water heater applied: fields=%u",
               static_cast<unsigned>(target.fields.size()));
      DAIKIN_DBG(TAG, "TX timing: water_heater_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, tx_operation_.attempts_sent);
    }

    update_water_heater_main_cache_from_bus_(buffer, true);
    if (tx_operation_.attempts_sent > 0) {
      start_water_heater_ui_sync_(target);
    }
    clear_tx_wait_markers_();
    reset_tx_lifecycle_(TxOperationKind::WATER_HEATER, false);
    flush_deferred_user_tx_();
    return;
  }

  if (tx_operation_.attempts_sent == 0) {
    return;
  }

  if (tx_operation_.attempts_sent < kTxMaxRepeats) {
    if (first_mismatch != nullptr && first_mismatch->readback_index < buffer.size()) {
      uint8_t current_value = buffer[first_mismatch->readback_index];
      uint8_t expected_value = first_mismatch->readback_value;
      if (first_mismatch->readback_bit_position != BIT_POSITION_NO_BITMASK) {
        current_value = extract_field_value(buffer, first_mismatch->readback_index,
                                            first_mismatch->readback_bit_position,
                                            first_mismatch->readback_bit_width);
        expected_value &= field_value_mask(first_mismatch->readback_index,
                                           first_mismatch->readback_bit_position,
                                           first_mismatch->readback_bit_width);
      }
      DAIKIN_DBG(TAG,
                 "Native water heater retry pending: first_mismatch=%s expected=0x%02X current=0x%02X attempt=%u/%u",
                 first_mismatch->name, expected_value, current_value,
                 tx_operation_.attempts_sent, kTxMaxRepeats);
    } else {
      DAIKIN_DBG(TAG, "Native water heater retry pending: readback=%s attempt=%u/%u",
                 packet_type_to_string_(spec.readback_packet_type).c_str(),
                 tx_operation_.attempts_sent, kTxMaxRepeats);
    }
    return;
  }

  if (first_mismatch != nullptr && first_mismatch->readback_index < buffer.size()) {
    uint8_t current_value = buffer[first_mismatch->readback_index];
    uint8_t expected_value = first_mismatch->readback_value;
    if (first_mismatch->readback_bit_position != BIT_POSITION_NO_BITMASK) {
      current_value = extract_field_value(buffer, first_mismatch->readback_index,
                                          first_mismatch->readback_bit_position,
                                          first_mismatch->readback_bit_width);
      expected_value &= field_value_mask(first_mismatch->readback_index,
                                         first_mismatch->readback_bit_position,
                                         first_mismatch->readback_bit_width);
    }
    DAIKIN_WARN(TAG,
                "Native water heater not applied: first_mismatch=%s readback_index=%u expected=0x%02X current=0x%02X attempts=%u",
                first_mismatch->name, first_mismatch->readback_index, expected_value, current_value,
                tx_operation_.attempts_sent);
  } else {
    DAIKIN_WARN(TAG, "Native water heater not applied: readback=%s attempts=%u",
                packet_type_to_string_(spec.readback_packet_type).c_str(), tx_operation_.attempts_sent);
  }
  DAIKIN_DBG(TAG, "TX timing: water_heater_failed_after=%u attempts=%u",
             millis() - tx_sent_ms_, tx_operation_.attempts_sent);

  update_water_heater_main_cache_from_bus_(buffer, true);
  clear_tx_wait_markers_();
  reset_tx_lifecycle_(TxOperationKind::WATER_HEATER, true);
  flush_deferred_user_tx_();
}
#endif

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
