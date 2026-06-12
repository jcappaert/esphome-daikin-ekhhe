#include "daikin_ekhhe.h"
#include "daikin_ekhhe_metadata.h"
#include "daikin_ekhhe_const.h"
#include "daikin_ekhhe_log.h"

#include <cstdio>
#include <ctime>
#include <cstring>

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

static const char *const TAG = "daikin_ekhhe";

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

}  // namespace daikin_ekkhe
}  // namespace esphome
