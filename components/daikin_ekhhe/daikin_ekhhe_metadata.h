#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "daikin_ekhhe.h"

namespace esphome {
namespace daikin_ekkhe {

constexpr uint8_t DD_PACKET_START_BYTE = 0xDD;
constexpr uint8_t D2_PACKET_START_BYTE = 0xD2;
constexpr uint8_t D4_PACKET_START_BYTE = 0xD4;
constexpr uint8_t C1_PACKET_START_BYTE = 0xC1;
constexpr uint8_t C2_PACKET_START_BYTE = 0xC2;
constexpr uint8_t CC_PACKET_START_BYTE = 0xCC;
constexpr uint8_t CD_PACKET_START_BYTE = 0xCD;
constexpr uint8_t SILENT_MODE_BIT_POSITION = 6;

constexpr uint32_t PROFILE_MAGIC = 0x454b4850U;
constexpr uint8_t PROFILE_VERSION = 2;
constexpr uint32_t KNOWN_GOOD_PROFILE_PREF_KEY = 0x444b4d31U;
constexpr uint32_t AUTO_SNAPSHOT_PREF_KEY = 0x444b4d32U;

extern const std::map<uint8_t, uint8_t> PACKET_SIZES;

struct PacketFieldRef {
  uint8_t index;
  uint8_t bit_position;
  uint8_t bit_width;
};

struct RestoreFieldSpec {
  const char *name;
  PacketFieldRef write;
  PacketFieldRef readback;
  uint8_t value;

  constexpr RestoreFieldSpec(const char *name, uint8_t write_index, uint8_t write_bit_position,
                             uint8_t write_bit_width, uint8_t readback_index,
                             uint8_t readback_bit_position, uint8_t readback_bit_width,
                             uint8_t value)
      : name(name),
        write{write_index, write_bit_position, write_bit_width},
        readback{readback_index, readback_bit_position, readback_bit_width},
        value(value) {}
};

struct ManagedFieldSpec {
  const char *name;
  PacketFieldRef write;
  PacketFieldRef readback;

  constexpr ManagedFieldSpec(const char *name, uint8_t write_index, uint8_t write_bit_position,
                             uint8_t write_bit_width, uint8_t readback_index,
                             uint8_t readback_bit_position, uint8_t readback_bit_width)
      : name(name),
        write{write_index, write_bit_position, write_bit_width},
        readback{readback_index, readback_bit_position, readback_bit_width} {}
};

constexpr uint8_t encode_i8(int value) {
  return static_cast<uint8_t>(static_cast<int8_t>(value));
}

bool field_is_extended(WritablePacketFamily family);
float decode_number_field_value(uint8_t value, const NumberFieldSpec &field);
float decode_number_field(const std::vector<uint8_t> &buffer, const NumberFieldSpec &field);
uint8_t encode_number_field(float value, const NumberFieldSpec &field);

const RestoreFieldSpec *restore_default_fields(bool extended, size_t &count);
bool restore_scope_field_matches(const RestoreFieldSpec &field, const std::vector<uint8_t> &buffer,
                                 bool use_d2_indices);
void apply_restore_defaults_to_packet(std::vector<uint8_t> &packet, bool extended = false);
bool restore_defaults_match_packet(const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                   bool extended = false);
const RestoreFieldSpec *first_restore_mismatch(const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                               uint8_t &current_value, bool extended = false);
bool is_restore_scope_field(uint8_t cc_index, uint8_t bit_position, bool extended = false);

const ManagedFieldSpec *profile_managed_fields(bool extended, size_t &count);
bool is_profile_managed_field(bool extended, uint8_t cc_index, uint8_t bit_position);
const ManagedFieldSpec *find_managed_field_by_cc(uint8_t cc_index, uint8_t bit_position);

uint8_t field_value_mask(uint8_t index, uint8_t bit_position, uint8_t bit_width);
uint8_t field_log_width(uint8_t index, uint8_t bit_position, uint8_t bit_width);
uint8_t extract_field_value(const uint8_t *data, size_t size, uint8_t index,
                            uint8_t bit_position, uint8_t bit_width);
uint8_t effective_bit_width(uint8_t bit_position, uint8_t bit_width);
uint8_t extract_field_value(const std::vector<uint8_t> &data, uint8_t index,
                            uint8_t bit_position, uint8_t bit_width);
uint8_t extract_field_value(const uint8_t *data, size_t size, const PacketFieldRef &field);
uint8_t extract_field_value(const std::vector<uint8_t> &data, const PacketFieldRef &field);
void apply_field_value(std::vector<uint8_t> &packet, uint8_t index, uint8_t bit_position,
                       uint8_t bit_width, uint8_t value);
void apply_field_value(std::vector<uint8_t> &packet, const PacketFieldRef &field, uint8_t value);

uint32_t profile_data_hash(const uint8_t *data, size_t length);
bool profile_matches_packet(bool extended, const uint8_t *profile_data, size_t profile_len,
                            const std::vector<uint8_t> &buffer, bool use_d2_indices);
const ManagedFieldSpec *first_profile_mismatch(bool extended,
                                               const uint8_t *profile_data, size_t profile_len,
                                               const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                               uint8_t &expected_value, uint8_t &current_value);
void merge_profile_managed_fields(bool extended, std::vector<uint8_t> &packet, const uint8_t *profile_data,
                                  size_t profile_len);
bool managed_fields_equal_between_packets(bool extended, const uint8_t *lhs, size_t lhs_len,
                                          const uint8_t *rhs, size_t rhs_len);

}  // namespace daikin_ekkhe
}  // namespace esphome
