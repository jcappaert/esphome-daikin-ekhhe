#include "daikin_ekhhe.h"
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

static const uint8_t DD_PACKET_START_BYTE = 0xDD;
static const uint8_t D2_PACKET_START_BYTE = 0xD2;
static const uint8_t D4_PACKET_START_BYTE = 0xD4;
static const uint8_t C1_PACKET_START_BYTE = 0xC1;
static const uint8_t C2_PACKET_START_BYTE = 0xC2;
static const uint8_t CC_PACKET_START_BYTE = 0xCC;
static const uint8_t CD_PACKET_START_BYTE = 0xCD;
static const uint8_t SILENT_MODE_BIT_POSITION = 6;


// Packet definitions
static const std::map<uint8_t, uint8_t> PACKET_SIZES = {
    {DD_PACKET_START_BYTE, DaikinEkhheComponent::DD_PACKET_SIZE},
    {D2_PACKET_START_BYTE, DaikinEkhheComponent::D2_PACKET_SIZE},
    {D4_PACKET_START_BYTE, DaikinEkhheComponent::D4_PACKET_SIZE},
    {C1_PACKET_START_BYTE, DaikinEkhheComponent::C1_PACKET_SIZE},
    {C2_PACKET_START_BYTE, DaikinEkhheComponent::C2_PACKET_SIZE},
    {CC_PACKET_START_BYTE, DaikinEkhheComponent::CC_PACKET_SIZE},
    {CD_PACKET_START_BYTE, DaikinEkhheComponent::CD_PACKET_SIZE},
};

struct RestoreFieldSpec {
  const char *name;
  uint8_t cc_index;
  uint8_t cc_bit_position;
  uint8_t cc_bit_width;
  uint8_t d2_index;
  uint8_t d2_bit_position;
  uint8_t d2_bit_width;
  uint8_t value;
};

constexpr uint8_t encode_i8(int value) {
  return static_cast<uint8_t>(static_cast<int8_t>(value));
}

uint8_t extract_field_value(const std::vector<uint8_t> &data, uint8_t index,
                            uint8_t bit_position, uint8_t bit_width);
void apply_field_value(std::vector<uint8_t> &packet, uint8_t index, uint8_t bit_position,
                       uint8_t bit_width, uint8_t value);
uint8_t field_value_mask(uint8_t index, uint8_t bit_position, uint8_t bit_width);

static const RestoreFieldSpec RESTORE_DEFAULT_FIELDS[] = {
    {"P1", DaikinEkhheComponent::CC_PACKET_P1_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P1_IDX, BIT_POSITION_NO_BITMASK, 0, 7},
    {"P2", DaikinEkhheComponent::CC_PACKET_P2_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P2_IDX, BIT_POSITION_NO_BITMASK, 0, 6},
    {"P3", DaikinEkhheComponent::CC_PACKET_P3_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P3_IDX, BIT_POSITION_NO_BITMASK, 0, 75},
    {"P4", DaikinEkhheComponent::CC_PACKET_P4_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P4_IDX, BIT_POSITION_NO_BITMASK, 0, 30},
    {"P5", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 2, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 2, 1, 1},
    {"P6", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 3, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 3, 1, 0},
    {"P7", DaikinEkhheComponent::CC_PACKET_P7_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P7_IDX, BIT_POSITION_NO_BITMASK, 0, 60},
    {"P8", DaikinEkhheComponent::CC_PACKET_P8_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P8_IDX, BIT_POSITION_NO_BITMASK, 0, encode_i8(-5)},
    {"P9", DaikinEkhheComponent::CC_PACKET_P9_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P9_IDX, BIT_POSITION_NO_BITMASK, 0, 3},
    {"P10", DaikinEkhheComponent::CC_PACKET_P10_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P10_IDX, BIT_POSITION_NO_BITMASK, 0, 10},
    {"P11", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 0, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 0, 1, 1},
    {"P12", DaikinEkhheComponent::CC_PACKET_P12_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P12_IDX, BIT_POSITION_NO_BITMASK, 0, 1},
    {"P13", DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 4, 1,
     DaikinEkhheComponent::D2_PACKET_MASK1_IDX, 4, 1, 0},
    {"P14", DaikinEkhheComponent::CC_PACKET_P14_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P14_IDX, BIT_POSITION_NO_BITMASK, 0, 3},
    {"P15", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 1, BIT_WIDTH_P15_STEPPED,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 1, BIT_WIDTH_P15_STEPPED, 2},
    {"P16", DaikinEkhheComponent::CC_PACKET_P16_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P16_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P17", DaikinEkhheComponent::CC_PACKET_P17_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P17_IDX, BIT_POSITION_NO_BITMASK, 0, 20},
    {"P18", DaikinEkhheComponent::CC_PACKET_P18_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P18_IDX, BIT_POSITION_NO_BITMASK, 0, 40},
    {"P19", DaikinEkhheComponent::CC_PACKET_P19_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P19_IDX, BIT_POSITION_NO_BITMASK, 0, 10},
    {"P20", DaikinEkhheComponent::CC_PACKET_P20_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P20_IDX, BIT_POSITION_NO_BITMASK, 0, 140},
    {"P21", DaikinEkhheComponent::CC_PACKET_P21_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P21_IDX, BIT_POSITION_NO_BITMASK, 0, 62},
    {"P22", DaikinEkhheComponent::CC_PACKET_P22_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P22_IDX, BIT_POSITION_NO_BITMASK, 0, 75},
    {"P23", DaikinEkhheComponent::CC_PACKET_P23_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P23_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P24", DaikinEkhheComponent::CC_PACKET_P24_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P24_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P25", DaikinEkhheComponent::CC_PACKET_P25_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P25_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P26", DaikinEkhheComponent::CC_PACKET_P26_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P26_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P27", DaikinEkhheComponent::CC_PACKET_P27_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P27_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P28", DaikinEkhheComponent::CC_PACKET_P28_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P28_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P29", DaikinEkhheComponent::CC_PACKET_P29_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P29_IDX, BIT_POSITION_NO_BITMASK, 0, 23},
    {"P30", DaikinEkhheComponent::CC_PACKET_P30_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P30_IDX, BIT_POSITION_NO_BITMASK, 0, 7},
    {"P31", DaikinEkhheComponent::CC_PACKET_P31_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P31_IDX, BIT_POSITION_NO_BITMASK, 0, 30},
    {"P32", DaikinEkhheComponent::CC_PACKET_P32_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P32_IDX, BIT_POSITION_NO_BITMASK, 0, 4},
    {"P33", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 4, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 4, 1, 1},
    {"P34", DaikinEkhheComponent::CC_PACKET_P34_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P34_IDX, BIT_POSITION_NO_BITMASK, 0, 30},
    {"P35", DaikinEkhheComponent::CC_PACKET_P35_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P35_IDX, BIT_POSITION_NO_BITMASK, 0, 4},
    {"P36", DaikinEkhheComponent::CC_PACKET_P36_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P36_IDX, BIT_POSITION_NO_BITMASK, 0, 88},
    {"P37", DaikinEkhheComponent::CC_PACKET_P37_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P37_IDX, BIT_POSITION_NO_BITMASK, 0, 15},
    {"P38", DaikinEkhheComponent::CC_PACKET_P38_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P38_IDX, BIT_POSITION_NO_BITMASK, 0, 9},
    {"P39", DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 2, 1,
     DaikinEkhheComponent::D2_PACKET_MASK1_IDX, 2, 1, 0},
    {"P40", DaikinEkhheComponent::CC_PACKET_P40_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P40_IDX, BIT_POSITION_NO_BITMASK, 0, 25},
    {"P41", DaikinEkhheComponent::CC_PACKET_P41_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P41_IDX, BIT_POSITION_NO_BITMASK, 0, encode_i8(-1)},
    {"P42", DaikinEkhheComponent::CC_PACKET_P42_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P42_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P43", DaikinEkhheComponent::CC_PACKET_P43_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P43_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P44", DaikinEkhheComponent::CC_PACKET_P44_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P44_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P45", DaikinEkhheComponent::CC_PACKET_P45_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P45_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P46", DaikinEkhheComponent::CC_PACKET_P46_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P46_IDX, BIT_POSITION_NO_BITMASK, 0, 1},
    {"P47", DaikinEkhheComponent::CC_PACKET_P47_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P47_IDX, BIT_POSITION_NO_BITMASK, 0, 43},
    {"P48", DaikinEkhheComponent::CC_PACKET_P48_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P48_IDX, BIT_POSITION_NO_BITMASK, 0, encode_i8(-7)},
    {"P49", DaikinEkhheComponent::CC_PACKET_P49_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P49_IDX, BIT_POSITION_NO_BITMASK, 0, 25},
    {"P50", DaikinEkhheComponent::CC_PACKET_P50_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P50_IDX, BIT_POSITION_NO_BITMASK, 0, 12},
    {"P51", DaikinEkhheComponent::CC_PACKET_P51_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P51_IDX, BIT_POSITION_NO_BITMASK, 0, 90},
    {"P52", DaikinEkhheComponent::CC_PACKET_P52_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P52_IDX, BIT_POSITION_NO_BITMASK, 0, 50},
    {"P54", DaikinEkhheComponent::CC_PACKET_P54_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P54_IDX, BIT_POSITION_NO_BITMASK, 0, 1},
    {"ECO_TARGET", DaikinEkhheComponent::CC_PACKET_ECO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_ECO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0, 55},
    {"AUTO_TARGET", DaikinEkhheComponent::CC_PACKET_AUTO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_AUTO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0, 55},
    {"BOOST_TARGET", DaikinEkhheComponent::CC_PACKET_BOOST_TTGARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_BOOST_TTGARGET_IDX, BIT_POSITION_NO_BITMASK, 0, 55},
    {"ELECTRIC_TARGET", DaikinEkhheComponent::CC_PACKET_ELECTRIC_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_ELECTRIC_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0, 55},
};

static const RestoreFieldSpec RESTORE_DEFAULT_EXTENDED_FIELDS[] = {
    {"P53", DaikinEkhheComponent::EXT_PACKET_P53_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P53_IDX, BIT_POSITION_NO_BITMASK, 0, 50},
    {"P55", DaikinEkhheComponent::EXT_PACKET_P55_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P55_IDX, BIT_POSITION_NO_BITMASK, 0, 4},
    {"P56", DaikinEkhheComponent::EXT_PACKET_P56_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P56_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P57", DaikinEkhheComponent::EXT_PACKET_P57_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P57_IDX, BIT_POSITION_NO_BITMASK, 0, 1},
    {"P58", DaikinEkhheComponent::EXT_PACKET_P58_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P58_IDX, BIT_POSITION_NO_BITMASK, 0, 0},
    {"P59", DaikinEkhheComponent::EXT_PACKET_P59_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P59_IDX, BIT_POSITION_NO_BITMASK, 0, 40},
    {"P60", DaikinEkhheComponent::EXT_PACKET_P60_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P60_IDX, BIT_POSITION_NO_BITMASK, 0, 4},
    {"P61", DaikinEkhheComponent::EXT_PACKET_P61_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P61_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P62", DaikinEkhheComponent::EXT_PACKET_P62_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P62_IDX, BIT_POSITION_NO_BITMASK, 0, 6},
    {"P63", DaikinEkhheComponent::EXT_PACKET_P63_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P63_IDX, BIT_POSITION_NO_BITMASK, 0, 3},
    {"P64", DaikinEkhheComponent::EXT_PACKET_P64_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P64_IDX, BIT_POSITION_NO_BITMASK, 0, 10},
    {"P65", DaikinEkhheComponent::EXT_PACKET_P65_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P65_IDX, BIT_POSITION_NO_BITMASK, 0, 18},
    {"P66", DaikinEkhheComponent::EXT_PACKET_P66_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P66_IDX, BIT_POSITION_NO_BITMASK, 0, 2},
    {"P67", DaikinEkhheComponent::EXT_PACKET_P67_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P67_IDX, BIT_POSITION_NO_BITMASK, 0, 9},
    {"P68", DaikinEkhheComponent::EXT_PACKET_P68_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P68_IDX, BIT_POSITION_NO_BITMASK, 0, 5},
    {"P69", DaikinEkhheComponent::EXT_PACKET_P69_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P69_IDX, BIT_POSITION_NO_BITMASK, 0, 10},
    {"P70", DaikinEkhheComponent::EXT_PACKET_P70_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P70_IDX, BIT_POSITION_NO_BITMASK, 0, 5},
    {"P71", DaikinEkhheComponent::EXT_PACKET_P71_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P71_IDX, BIT_POSITION_NO_BITMASK, 0, 15},
    {"P72", DaikinEkhheComponent::EXT_PACKET_P72_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P72_IDX, BIT_POSITION_NO_BITMASK, 0, 5},
};

constexpr size_t RESTORE_DEFAULT_MAIN_FIELD_COUNT =
    sizeof(RESTORE_DEFAULT_FIELDS) / sizeof(RESTORE_DEFAULT_FIELDS[0]);
constexpr size_t RESTORE_DEFAULT_EXTENDED_FIELD_COUNT =
    sizeof(RESTORE_DEFAULT_EXTENDED_FIELDS) / sizeof(RESTORE_DEFAULT_EXTENDED_FIELDS[0]);
constexpr size_t RESTORE_DEFAULT_FIELD_COUNT =
    RESTORE_DEFAULT_MAIN_FIELD_COUNT + RESTORE_DEFAULT_EXTENDED_FIELD_COUNT;

const RestoreFieldSpec *restore_default_fields(bool extended, size_t &count) {
  if (extended) {
    count = RESTORE_DEFAULT_EXTENDED_FIELD_COUNT;
    return RESTORE_DEFAULT_EXTENDED_FIELDS;
  }
  count = RESTORE_DEFAULT_MAIN_FIELD_COUNT;
  return RESTORE_DEFAULT_FIELDS;
}

bool restore_scope_field_matches(const RestoreFieldSpec &field, const std::vector<uint8_t> &buffer,
                                 bool use_d2_indices) {
  const uint8_t index = use_d2_indices ? field.d2_index : field.cc_index;
  const uint8_t bit_position = use_d2_indices ? field.d2_bit_position : field.cc_bit_position;
  const uint8_t bit_width = use_d2_indices ? field.d2_bit_width : field.cc_bit_width;
  if (index >= buffer.size()) {
    return false;
  }
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return buffer[index] == field.value;
  }
  return extract_field_value(buffer, index, bit_position, bit_width) ==
         (field.value & field_value_mask(index, bit_position, bit_width));
}

void apply_restore_defaults_to_packet(std::vector<uint8_t> &packet, bool extended = false) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (field.cc_index >= packet.size()) {
      continue;
    }
    apply_field_value(packet, field.cc_index, field.cc_bit_position, field.cc_bit_width, field.value);
  }
}

bool restore_defaults_match_packet(const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                   bool extended = false) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (!restore_scope_field_matches(field, buffer, use_d2_indices)) {
      return false;
    }
  }
  return true;
}

const RestoreFieldSpec *first_restore_mismatch(const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                               uint8_t &current_value, bool extended = false) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (restore_scope_field_matches(field, buffer, use_d2_indices)) {
      continue;
    }
    const uint8_t index = use_d2_indices ? field.d2_index : field.cc_index;
    const uint8_t bit_position = use_d2_indices ? field.d2_bit_position : field.cc_bit_position;
    const uint8_t bit_width = use_d2_indices ? field.d2_bit_width : field.cc_bit_width;
    if (index >= buffer.size()) {
      current_value = 0;
    } else if (bit_position == BIT_POSITION_NO_BITMASK) {
      current_value = buffer[index];
    } else {
      current_value = extract_field_value(buffer, index, bit_position, bit_width);
    }
    return &field;
  }
  current_value = 0;
  return nullptr;
}

bool is_restore_scope_field(uint8_t cc_index, uint8_t bit_position, bool extended = false) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (field.cc_index == cc_index && field.cc_bit_position == bit_position) {
      return true;
    }
  }
  return false;
}

struct ManagedFieldSpec {
  const char *name;
  uint8_t cc_index;
  uint8_t cc_bit_position;
  uint8_t cc_bit_width;
  uint8_t d2_index;
  uint8_t d2_bit_position;
  uint8_t d2_bit_width;
};

static const ManagedFieldSpec PROFILE_MANAGED_FIELDS[] = {
    {"POWER_STATUS", DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 0, 1,
     DaikinEkhheComponent::D2_PACKET_MASK1_IDX, 0, 1},
    {"OPERATIONAL_MODE", DaikinEkhheComponent::CC_PACKET_MODE_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_MODE_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"SILENT_MODE", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION, 1},
    {"P1", DaikinEkhheComponent::CC_PACKET_P1_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P1_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P2", DaikinEkhheComponent::CC_PACKET_P2_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P2_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P3", DaikinEkhheComponent::CC_PACKET_P3_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P3_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P4", DaikinEkhheComponent::CC_PACKET_P4_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P4_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P5", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 2, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 2, 1},
    {"P6", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 3, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 3, 1},
    {"P7", DaikinEkhheComponent::CC_PACKET_P7_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P7_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P8", DaikinEkhheComponent::CC_PACKET_P8_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P8_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P9", DaikinEkhheComponent::CC_PACKET_P9_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P9_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P10", DaikinEkhheComponent::CC_PACKET_P10_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P10_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P11", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 0, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 0, 1},
    {"P12", DaikinEkhheComponent::CC_PACKET_P12_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P12_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P13", DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 4, 1,
     DaikinEkhheComponent::D2_PACKET_MASK1_IDX, 4, 1},
    {"P14", DaikinEkhheComponent::CC_PACKET_P14_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P14_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P15", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 1, BIT_WIDTH_P15_STEPPED,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 1, BIT_WIDTH_P15_STEPPED},
    {"P16", DaikinEkhheComponent::CC_PACKET_P16_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P16_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P17", DaikinEkhheComponent::CC_PACKET_P17_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P17_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P18", DaikinEkhheComponent::CC_PACKET_P18_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P18_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P19", DaikinEkhheComponent::CC_PACKET_P19_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P19_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P20", DaikinEkhheComponent::CC_PACKET_P20_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P20_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P21", DaikinEkhheComponent::CC_PACKET_P21_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P21_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P22", DaikinEkhheComponent::CC_PACKET_P22_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P22_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P23", DaikinEkhheComponent::CC_PACKET_P23_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P23_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P24", DaikinEkhheComponent::CC_PACKET_P24_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P24_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P25", DaikinEkhheComponent::CC_PACKET_P25_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P25_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P26", DaikinEkhheComponent::CC_PACKET_P26_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P26_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P27", DaikinEkhheComponent::CC_PACKET_P27_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P27_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P28", DaikinEkhheComponent::CC_PACKET_P28_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P28_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P29", DaikinEkhheComponent::CC_PACKET_P29_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P29_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P30", DaikinEkhheComponent::CC_PACKET_P30_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P30_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P31", DaikinEkhheComponent::CC_PACKET_P31_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P31_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P32", DaikinEkhheComponent::CC_PACKET_P32_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P32_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P33", DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 4, 1,
     DaikinEkhheComponent::D2_PACKET_MASK2_IDX, 4, 1},
    {"P34", DaikinEkhheComponent::CC_PACKET_P34_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P34_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P35", DaikinEkhheComponent::CC_PACKET_P35_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P35_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P36", DaikinEkhheComponent::CC_PACKET_P36_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P36_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P37", DaikinEkhheComponent::CC_PACKET_P37_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P37_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P38", DaikinEkhheComponent::CC_PACKET_P38_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P38_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P39", DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 2, 1,
     DaikinEkhheComponent::D2_PACKET_MASK1_IDX, 2, 1},
    {"P40", DaikinEkhheComponent::CC_PACKET_P40_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P40_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P41", DaikinEkhheComponent::CC_PACKET_P41_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P41_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P42", DaikinEkhheComponent::CC_PACKET_P42_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P42_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P43", DaikinEkhheComponent::CC_PACKET_P43_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P43_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P44", DaikinEkhheComponent::CC_PACKET_P44_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P44_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P45", DaikinEkhheComponent::CC_PACKET_P45_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P45_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P46", DaikinEkhheComponent::CC_PACKET_P46_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P46_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P47", DaikinEkhheComponent::CC_PACKET_P47_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P47_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P48", DaikinEkhheComponent::CC_PACKET_P48_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P48_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P49", DaikinEkhheComponent::CC_PACKET_P49_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P49_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P50", DaikinEkhheComponent::CC_PACKET_P50_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P50_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P51", DaikinEkhheComponent::CC_PACKET_P51_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P51_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P52", DaikinEkhheComponent::CC_PACKET_P52_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P52_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P54", DaikinEkhheComponent::CC_PACKET_P54_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_P54_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"ECO_TARGET", DaikinEkhheComponent::CC_PACKET_ECO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_ECO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"AUTO_TARGET", DaikinEkhheComponent::CC_PACKET_AUTO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_AUTO_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"BOOST_TARGET", DaikinEkhheComponent::CC_PACKET_BOOST_TTGARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_BOOST_TTGARGET_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"ELECTRIC_TARGET", DaikinEkhheComponent::CC_PACKET_ELECTRIC_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_ELECTRIC_TTARGET_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"VAC_DAYS", DaikinEkhheComponent::CC_PACKET_VAC_DAYS, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::D2_PACKET_VAC_DAYS, BIT_POSITION_NO_BITMASK, 0},
};

static const ManagedFieldSpec PROFILE_MANAGED_EXTENDED_FIELDS[] = {
    {"P53", DaikinEkhheComponent::EXT_PACKET_P53_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P53_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P55", DaikinEkhheComponent::EXT_PACKET_P55_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P55_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P56", DaikinEkhheComponent::EXT_PACKET_P56_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P56_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P57", DaikinEkhheComponent::EXT_PACKET_P57_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P57_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P58", DaikinEkhheComponent::EXT_PACKET_P58_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P58_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P59", DaikinEkhheComponent::EXT_PACKET_P59_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P59_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P60", DaikinEkhheComponent::EXT_PACKET_P60_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P60_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P61", DaikinEkhheComponent::EXT_PACKET_P61_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P61_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P62", DaikinEkhheComponent::EXT_PACKET_P62_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P62_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P63", DaikinEkhheComponent::EXT_PACKET_P63_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P63_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P64", DaikinEkhheComponent::EXT_PACKET_P64_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P64_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P65", DaikinEkhheComponent::EXT_PACKET_P65_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P65_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P66", DaikinEkhheComponent::EXT_PACKET_P66_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P66_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P67", DaikinEkhheComponent::EXT_PACKET_P67_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P67_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P68", DaikinEkhheComponent::EXT_PACKET_P68_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P68_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P69", DaikinEkhheComponent::EXT_PACKET_P69_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P69_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P70", DaikinEkhheComponent::EXT_PACKET_P70_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P70_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P71", DaikinEkhheComponent::EXT_PACKET_P71_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P71_IDX, BIT_POSITION_NO_BITMASK, 0},
    {"P72", DaikinEkhheComponent::EXT_PACKET_P72_IDX, BIT_POSITION_NO_BITMASK, 0,
     DaikinEkhheComponent::EXT_PACKET_P72_IDX, BIT_POSITION_NO_BITMASK, 0},
};

constexpr size_t PROFILE_MANAGED_MAIN_FIELD_COUNT =
    sizeof(PROFILE_MANAGED_FIELDS) / sizeof(PROFILE_MANAGED_FIELDS[0]);
constexpr size_t PROFILE_MANAGED_EXTENDED_FIELD_COUNT =
    sizeof(PROFILE_MANAGED_EXTENDED_FIELDS) / sizeof(PROFILE_MANAGED_EXTENDED_FIELDS[0]);
constexpr size_t PROFILE_MANAGED_FIELD_COUNT =
    PROFILE_MANAGED_MAIN_FIELD_COUNT + PROFILE_MANAGED_EXTENDED_FIELD_COUNT;

const ManagedFieldSpec *profile_managed_fields(bool extended, size_t &count) {
  if (extended) {
    count = PROFILE_MANAGED_EXTENDED_FIELD_COUNT;
    return PROFILE_MANAGED_EXTENDED_FIELDS;
  }
  count = PROFILE_MANAGED_MAIN_FIELD_COUNT;
  return PROFILE_MANAGED_FIELDS;
}

bool is_profile_managed_field(bool extended, uint8_t cc_index, uint8_t bit_position) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    if (field.cc_index == cc_index && field.cc_bit_position == bit_position) {
      return true;
    }
  }
  return false;
}

const ManagedFieldSpec *find_managed_field_by_cc(uint8_t cc_index, uint8_t bit_position) {
  for (const auto &field : PROFILE_MANAGED_FIELDS) {
    if (field.cc_index != cc_index) {
      continue;
    }
    if (bit_position == BIT_POSITION_NO_BITMASK && field.cc_bit_position == BIT_POSITION_NO_BITMASK) {
      return &field;
    }
    if (bit_position != BIT_POSITION_NO_BITMASK && field.cc_bit_position == bit_position) {
      return &field;
    }
  }
  return nullptr;
}

constexpr uint32_t PROFILE_MAGIC = 0x454b4850U;
constexpr uint8_t PROFILE_VERSION = 2;
constexpr uint32_t KNOWN_GOOD_PROFILE_PREF_KEY = 0x444b4d31U;
constexpr uint32_t AUTO_SNAPSHOT_PREF_KEY = 0x444b4d32U;

bool is_p15_stepped_field(uint8_t index, uint8_t bit_position, uint8_t bit_width) {
  return index == DaikinEkhheComponent::CC_PACKET_MASK2_IDX && bit_position == 1 &&
         bit_width == BIT_WIDTH_P15_STEPPED;
}

uint8_t field_value_mask(uint8_t index, uint8_t bit_position, uint8_t bit_width) {
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return 0xFF;
  }
  if (is_p15_stepped_field(index, bit_position, bit_width)) {
    return 0x03;
  }
  const uint8_t width = bit_width == 0 ? 1 : bit_width;
  return static_cast<uint8_t>((1U << width) - 1U);
}

uint8_t field_log_width(uint8_t index, uint8_t bit_position, uint8_t bit_width) {
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return 0;
  }
  if (is_p15_stepped_field(index, bit_position, bit_width)) {
    return 2;
  }
  return bit_width == 0 ? 1 : bit_width;
}

uint8_t extract_field_value(const uint8_t *data, size_t size, uint8_t index,
                            uint8_t bit_position, uint8_t bit_width) {
  if (index >= size) {
    return 0;
  }
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return data[index];
  }
  if (is_p15_stepped_field(index, bit_position, bit_width)) {
    const uint8_t low_pressure_bit = (data[index] >> 5) & 0x01;
    const uint8_t switch_type_bit = (data[index] >> 1) & 0x01;
    return static_cast<uint8_t>(switch_type_bit + low_pressure_bit);
  }
  const uint8_t mask = static_cast<uint8_t>((1U << bit_width) - 1U);
  return static_cast<uint8_t>((data[index] >> bit_position) & mask);
}

uint8_t effective_bit_width(uint8_t bit_position, uint8_t bit_width) {
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    return 0;
  }
  return bit_width == 0 ? 1 : bit_width;
}

uint8_t extract_field_value(const std::vector<uint8_t> &data, uint8_t index,
                            uint8_t bit_position, uint8_t bit_width) {
  return extract_field_value(data.data(), data.size(), index, bit_position, bit_width);
}

void apply_field_value(std::vector<uint8_t> &packet, uint8_t index, uint8_t bit_position,
                       uint8_t bit_width, uint8_t value) {
  if (index >= packet.size()) {
    return;
  }
  if (bit_position == BIT_POSITION_NO_BITMASK) {
    packet[index] = value;
    return;
  }
  if (is_p15_stepped_field(index, bit_position, bit_width)) {
    uint8_t current = packet[index];
    current &= static_cast<uint8_t>(~((1U << 1) | (1U << 5)));
    if (value >= 1) {
      current |= static_cast<uint8_t>(1U << 1);
    }
    if (value >= 2) {
      current |= static_cast<uint8_t>(1U << 5);
    }
    packet[index] = current;
    return;
  }
  const uint8_t mask = static_cast<uint8_t>((1U << bit_width) - 1U);
  uint8_t current = packet[index];
  current &= static_cast<uint8_t>(~(mask << bit_position));
  current |= static_cast<uint8_t>((value & mask) << bit_position);
  packet[index] = current;
}

uint32_t profile_data_hash(const uint8_t *data, size_t length) {
  uint32_t hash = 2166136261u;
  for (size_t i = 0; i < length; i++) {
    hash ^= data[i];
    hash *= 16777619u;
  }
  return hash;
}

bool profile_matches_packet(bool extended, const uint8_t *profile_data, size_t profile_len,
                            const std::vector<uint8_t> &buffer, bool use_d2_indices) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    const uint8_t expected = extract_field_value(profile_data, profile_len, field.cc_index,
                                                 field.cc_bit_position, field.cc_bit_width);
    const uint8_t actual = use_d2_indices
                               ? extract_field_value(buffer, field.d2_index, field.d2_bit_position,
                                                     field.d2_bit_width)
                               : extract_field_value(buffer, field.cc_index, field.cc_bit_position,
                                                     field.cc_bit_width);
    if (expected != actual) {
      return false;
    }
  }
  return true;
}

const ManagedFieldSpec *first_profile_mismatch(bool extended,
                                               const uint8_t *profile_data, size_t profile_len,
                                               const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                               uint8_t &expected_value, uint8_t &current_value) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    expected_value = extract_field_value(profile_data, profile_len, field.cc_index,
                                         field.cc_bit_position, field.cc_bit_width);
    current_value = use_d2_indices
                        ? extract_field_value(buffer, field.d2_index, field.d2_bit_position,
                                              field.d2_bit_width)
                        : extract_field_value(buffer, field.cc_index, field.cc_bit_position,
                                              field.cc_bit_width);
    if (expected_value != current_value) {
      return &field;
    }
  }
  expected_value = 0;
  current_value = 0;
  return nullptr;
}

void merge_profile_managed_fields(bool extended, std::vector<uint8_t> &packet, const uint8_t *profile_data,
                                  size_t profile_len) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    const uint8_t value = extract_field_value(profile_data, profile_len, field.cc_index,
                                              field.cc_bit_position, field.cc_bit_width);
    apply_field_value(packet, field.cc_index, field.cc_bit_position, field.cc_bit_width, value);
  }
}

bool managed_fields_equal_between_packets(bool extended, const uint8_t *lhs, size_t lhs_len,
                                          const uint8_t *rhs, size_t rhs_len) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    if (extract_field_value(lhs, lhs_len, field.cc_index, field.cc_bit_position, field.cc_bit_width) !=
        extract_field_value(rhs, rhs_len, field.cc_index, field.cc_bit_position, field.cc_bit_width)) {
      return false;
    }
  }
  return true;
}


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
  if (tx_operation_active_() && tx_waiting_for_first_rx_) {
    DAIKIN_DBG(TAG, "TX timing: first_rx_after_tx type=%s dt=%u",
               packet_type_to_string_(byte).c_str(), now_ms - tx_sent_ms_);
    tx_waiting_for_first_rx_ = false;
  }

  uint8_t tx_readback_type = D2_PACKET_START_BYTE;
  if (pending_tx_.active) {
    tx_readback_type = tx_packet_family_spec_(pending_tx_.family).readback_packet_type;
  } else if (pending_restore_.active) {
    tx_readback_type = tx_packet_family_spec_(pending_restore_.family).readback_packet_type;
  } else if (pending_profile_restore_.active) {
    tx_readback_type = tx_packet_family_spec_(pending_profile_restore_.family).readback_packet_type;
  }
  if (tx_operation_active_() && tx_waiting_for_first_cc_ && byte == tx_readback_type) {
    DAIKIN_DBG(TAG, "TX timing: first_readback_after_tx type=%s dt=%u",
               packet_type_to_string_(byte).c_str(), now_ms - tx_sent_ms_);
    tx_waiting_for_first_cc_ = false;
  }

  last_frame_profile_ms_ = now_ms;
  last_frame_profile_type_ = byte;
  if (byte == CC_PACKET_START_BYTE) {
    last_cc_profile_ms_ = now_ms;
  }
  if (byte == C2_PACKET_START_BYTE) {
    last_c2_packet_ = packet;
  }
  if (pending_restore_.active &&
      byte == tx_packet_family_spec_(pending_restore_.family).readback_packet_type) {
    check_pending_restore_(packet);
  }
  if (pending_profile_restore_.active &&
      byte == tx_packet_family_spec_(pending_profile_restore_.family).readback_packet_type) {
    check_pending_profile_restore_(packet);
  }
  if (byte == D2_PACKET_START_BYTE && pending_time_band_tx_.active) {
    check_pending_time_band_(packet);
  }
  if (pending_tx_.active && byte == tx_packet_family_spec_(pending_tx_.family).readback_packet_type) {
    check_pending_tx_(packet);
  }
  if (byte == D2_PACKET_START_BYTE &&
      (pending_tx_.active || pending_restore_.active || pending_profile_restore_.active ||
       pending_time_band_tx_.active)) {
    size_t d2_index = 0;
    const RawFrameEntry *d2_entry = find_latest_frame_by_type_(D2_PACKET_START_BYTE, d2_index, true);
    if (d2_entry != nullptr) {
      if (pending_restore_.active) {
        schedule_queued_restore_from_d2_(*d2_entry);
      } else if (pending_profile_restore_.active) {
        schedule_queued_profile_restore_from_d2_(*d2_entry);
      } else if (pending_time_band_tx_.active) {
        schedule_queued_time_band_from_d2_(*d2_entry);
      } else if (pending_tx_.active) {
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
  const uint8_t flag_idx =
      d2_packet ? D2_PACKET_TIME_BAND_FLAG_IDX : CC_PACKET_TIME_BAND_FLAG_IDX;
  const uint8_t start_hour_idx =
      d2_packet ? D2_PACKET_TIME_BAND_START_HOUR_IDX : CC_PACKET_TIME_BAND_START_HOUR_IDX;
  const uint8_t start_minute_idx =
      d2_packet ? D2_PACKET_TIME_BAND_START_MINUTE_IDX : CC_PACKET_TIME_BAND_START_MINUTE_IDX;
  const uint8_t end_hour_idx =
      d2_packet ? D2_PACKET_TIME_BAND_END_HOUR_IDX : CC_PACKET_TIME_BAND_END_HOUR_IDX;
  const uint8_t end_minute_idx =
      d2_packet ? D2_PACKET_TIME_BAND_END_MINUTE_IDX : CC_PACKET_TIME_BAND_END_MINUTE_IDX;
  const uint8_t mode_idx =
      d2_packet ? D2_PACKET_TIME_BAND_MODE_IDX : CC_PACKET_TIME_BAND_MODE_IDX;

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
  return field != nullptr ? field->d2_index : write_index;
}

uint8_t DaikinEkhheComponent::tx_readback_bit_position_(TxPacketFamily family, uint8_t write_index,
                                                        uint8_t bit_position) const {
  if (family == TxPacketFamily::EXTENDED || bit_position == BIT_POSITION_NO_BITMASK) {
    return bit_position;
  }

  const ManagedFieldSpec *field = find_managed_field_by_cc(write_index, bit_position);
  return field != nullptr ? field->d2_bit_position : bit_position;
}

uint8_t DaikinEkhheComponent::tx_readback_bit_width_(TxPacketFamily family, uint8_t write_index,
                                                     uint8_t bit_position, uint8_t bit_width) const {
  if (family == TxPacketFamily::EXTENDED || bit_position == BIT_POSITION_NO_BITMASK) {
    return bit_width;
  }

  const ManagedFieldSpec *field = find_managed_field_by_cc(write_index, bit_position);
  return field != nullptr ? field->d2_bit_width : bit_width;
}

bool DaikinEkhheComponent::tx_operation_active_() const {
  return pending_tx_.active || pending_restore_.active || pending_profile_restore_.active ||
         pending_time_band_tx_.active;
}

void DaikinEkhheComponent::reset_pending_restore_() {
  pending_restore_.active = false;
  pending_restore_.family = TxPacketFamily::MAIN;
  pending_restore_.main_applied = false;
  pending_restore_.extended_applied = false;
  pending_restore_.main_write_sent = false;
  pending_restore_.extended_write_sent = false;
  pending_restore_.attempts_sent = 0;
  pending_restore_.last_attempt_d2_seq = 0;
}

void DaikinEkhheComponent::reset_queued_restore_() {
  queued_restore_.active = false;
  queued_restore_.scheduled = false;
  queued_restore_.family = TxPacketFamily::MAIN;
  queued_restore_.anchor_ms = 0;
  queued_restore_.anchor_seq = 0;
}

void DaikinEkhheComponent::reset_pending_profile_restore_() {
  pending_profile_restore_.active = false;
  pending_profile_restore_.known_good = false;
  pending_profile_restore_.family = TxPacketFamily::MAIN;
  pending_profile_restore_.main_applied = false;
  pending_profile_restore_.extended_applied = false;
  pending_profile_restore_.main_write_sent = false;
  pending_profile_restore_.extended_write_sent = false;
  pending_profile_restore_.attempts_sent = 0;
  pending_profile_restore_.last_attempt_d2_seq = 0;
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
  pending_time_band_tx_.active = false;
  pending_time_band_tx_.flag = 0;
  pending_time_band_tx_.attempts_sent = 0;
  pending_time_band_tx_.last_attempt_d2_seq = 0;
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
  if (pending_tx_.active || queued_tx_.active || queued_tx_.scheduled || tx_ui_sync_.active ||
      pending_restore_.active || restore_ui_sync_.active ||
      pending_profile_restore_.active || profile_ui_sync_.active ||
      pending_time_band_tx_.active || queued_time_band_tx_.active ||
      queued_time_band_tx_.scheduled || time_band_ui_sync_.active) {
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

void DaikinEkhheComponent::schedule_queued_tx_from_d2_(const RawFrameEntry &d2_entry) {
  if (!pending_tx_.active || queued_tx_.scheduled) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms =
      d2_age_ms >= tx_delay_after_d2_ms_ ? 0 : (tx_delay_after_d2_ms_ - d2_age_ms);
  const uint32_t generation = queued_tx_.generation;

  queued_tx_.active = true;
  queued_tx_.scheduled = true;
  queued_tx_.family = pending_tx_.family;
  queued_tx_.index = pending_tx_.index;
  queued_tx_.value = pending_tx_.value;
  queued_tx_.bit_position = pending_tx_.bit_position;
  queued_tx_.bit_width = pending_tx_.bit_width;
  queued_tx_.anchor_ms = d2_entry.timestamp_ms;
  queued_tx_.anchor_seq = d2_entry.seq;

  const auto &pending_spec = tx_packet_family_spec_(pending_tx_.family);
  DAIKIN_DBG(TAG, "TX scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
             pending_spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_tx_.request_ms);

  set_timeout(delay_ms, [this, generation]() {
    if (!queued_tx_.active || queued_tx_.generation != generation) {
      return;
    }

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

    pending_tx_.attempts_sent++;
    pending_tx_.last_attempt_d2_seq = anchor_seq;

    DAIKIN_DBG(TAG,
               "TX scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
               spec.label, anchor_seq, millis() - anchor_ms, pending_tx_.attempts_sent, kTxMaxRepeats,
               latest_base != latest_packets_.end());
    send_uart_tx_packet_(family, base_packet, true, index, value, bit_position, bit_width);
  });
}

void DaikinEkhheComponent::schedule_queued_restore_from_d2_(const RawFrameEntry &d2_entry) {
  if (!pending_restore_.active || queued_restore_.scheduled) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms =
      d2_age_ms >= tx_delay_after_d2_ms_ ? 0 : (tx_delay_after_d2_ms_ - d2_age_ms);
  const uint32_t generation = queued_restore_.generation;
  const TxPacketFamily family = pending_restore_.family;
  const auto &spec = tx_packet_family_spec_(family);

  queued_restore_.active = true;
  queued_restore_.scheduled = true;
  queued_restore_.family = family;
  queued_restore_.anchor_ms = d2_entry.timestamp_ms;
  queued_restore_.anchor_seq = d2_entry.seq;

  DAIKIN_DBG(TAG, "Restore defaults scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
             spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_restore_.request_ms);

  set_timeout(delay_ms, [this, generation]() {
    if (!queued_restore_.active || queued_restore_.generation != generation) {
      return;
    }

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
      reset_pending_restore_();
      reset_queued_restore_();
      return;
    }

    std::vector<uint8_t> packet = base_packet;
    apply_restore_defaults_to_packet(packet, extended);
    pending_restore_.attempts_sent++;
    if (extended) {
      pending_restore_.extended_write_sent = true;
    } else {
      pending_restore_.main_write_sent = true;
    }
    pending_restore_.last_attempt_d2_seq = anchor_seq;

    DAIKIN_DBG(TAG,
               "Restore defaults scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
               spec.label, anchor_seq, millis() - anchor_ms, pending_restore_.attempts_sent, kTxMaxRepeats,
               latest_base != latest_packets_.end());
    send_restore_defaults_packet_(family, base_packet, packet);
  });
}

void DaikinEkhheComponent::schedule_queued_profile_restore_from_d2_(const RawFrameEntry &d2_entry) {
  if (!pending_profile_restore_.active || queued_profile_restore_.scheduled) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms =
      d2_age_ms >= tx_delay_after_d2_ms_ ? 0 : (tx_delay_after_d2_ms_ - d2_age_ms);
  const uint32_t generation = queued_profile_restore_.generation;
  const TxPacketFamily family = pending_profile_restore_.family;
  const auto &spec = tx_packet_family_spec_(family);

  queued_profile_restore_.active = true;
  queued_profile_restore_.scheduled = true;
  queued_profile_restore_.known_good = pending_profile_restore_.known_good;
  queued_profile_restore_.family = family;
  queued_profile_restore_.anchor_ms = d2_entry.timestamp_ms;
  queued_profile_restore_.anchor_seq = d2_entry.seq;

  DAIKIN_DBG(TAG, "%s restore scheduling: family=%s d2_seq=%u d2_age=%u delay=%u request_age=%u",
             queued_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot",
             spec.label, d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_profile_restore_.request_ms);

  set_timeout(delay_ms, [this, generation]() {
    if (!queued_profile_restore_.active || queued_profile_restore_.generation != generation) {
      return;
    }

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
      reset_pending_profile_restore_();
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
      reset_pending_profile_restore_();
      reset_queued_profile_restore_();
      return;
    }

    std::vector<uint8_t> packet = base_packet;
    merge_profile_managed_fields(extended, packet, profile_data, profile_length);
    pending_profile_restore_.attempts_sent++;
    if (extended) {
      pending_profile_restore_.extended_write_sent = true;
    } else {
      pending_profile_restore_.main_write_sent = true;
    }
    pending_profile_restore_.last_attempt_d2_seq = anchor_seq;

    DAIKIN_DBG(TAG,
               "%s restore scheduling: family=%s sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_base=%u",
               known_good ? "Known-good profile" : "Auto snapshot", spec.label,
               anchor_seq, millis() - anchor_ms, pending_profile_restore_.attempts_sent, kTxMaxRepeats,
               latest_base != latest_packets_.end());
    send_profile_restore_packet_(family, base_packet, packet, known_good);
  });
}

void DaikinEkhheComponent::schedule_queued_time_band_from_d2_(const RawFrameEntry &d2_entry) {
  if (!pending_time_band_tx_.active || queued_time_band_tx_.scheduled) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t d2_age_ms = now_ms - d2_entry.timestamp_ms;
  const uint32_t delay_ms =
      d2_age_ms >= tx_delay_after_d2_ms_ ? 0 : (tx_delay_after_d2_ms_ - d2_age_ms);
  const uint32_t generation = queued_time_band_tx_.generation;

  queued_time_band_tx_.active = true;
  queued_time_band_tx_.scheduled = true;
  queued_time_band_tx_.anchor_ms = d2_entry.timestamp_ms;
  queued_time_band_tx_.anchor_seq = d2_entry.seq;

  DAIKIN_DBG(TAG, "Time-band scheduling: d2_seq=%u d2_age=%u delay=%u request_age=%u",
             d2_entry.seq, d2_age_ms, delay_ms, now_ms - queued_time_band_tx_.request_ms);

  set_timeout(delay_ms, [this, generation]() {
    if (!queued_time_band_tx_.active || queued_time_band_tx_.generation != generation) {
      return;
    }

    const uint32_t anchor_seq = queued_time_band_tx_.anchor_seq;
    const uint32_t anchor_ms = queued_time_band_tx_.anchor_ms;
    queued_time_band_tx_.active = false;
    queued_time_band_tx_.scheduled = false;

    std::vector<uint8_t> base_packet = last_cc_packet_;
    auto latest_cc = latest_packets_.find(CC_PACKET_START_BYTE);
    if (latest_cc != latest_packets_.end()) {
      base_packet = latest_cc->second;
    }

    pending_time_band_tx_.attempts_sent++;
    pending_time_band_tx_.last_attempt_d2_seq = anchor_seq;

    DAIKIN_DBG(TAG,
               "Time-band scheduling: sending_after_d2 d2_seq=%u d2_to_send=%u attempt=%u/%u using_current_cycle_cc=%u",
               anchor_seq, millis() - anchor_ms, pending_time_band_tx_.attempts_sent, kTxMaxRepeats,
               latest_cc != latest_packets_.end());
    send_time_band_packet_(base_packet);
  });
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
  if (pending_profile_restore_.active || profile_ui_sync_.active) {
    DAIKIN_WARN(TAG, "%s restore already in progress.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }
  if (pending_restore_.active || restore_ui_sync_.active || pending_tx_.active || tx_ui_sync_.active ||
      pending_time_band_tx_.active || queued_time_band_tx_.active ||
      queued_time_band_tx_.scheduled || time_band_ui_sync_.active) {
    DAIKIN_WARN(TAG, "%s restore blocked: another write is currently active.",
                known_good ? "Known-good profile" : "Auto snapshot");
    return;
  }

  tx_request_ms_ = millis();
  pending_profile_restore_.active = true;
  pending_profile_restore_.known_good = known_good;
  pending_profile_restore_.family = TxPacketFamily::MAIN;
  pending_profile_restore_.main_applied = false;
  pending_profile_restore_.extended_applied = false;
  pending_profile_restore_.main_write_sent = false;
  pending_profile_restore_.extended_write_sent = false;
  pending_profile_restore_.attempts_sent = 0;
  pending_profile_restore_.last_attempt_d2_seq = 0;
  profile_ui_sync_.active = false;
  profile_ui_sync_.known_good = known_good;
  profile_ui_sync_.main_synced = false;
  profile_ui_sync_.extended_synced = false;
  profile_ui_sync_.cycles_waited = 0;

  reset_queued_profile_restore_();
  queued_profile_restore_.active = true;
  queued_profile_restore_.scheduled = false;
  queued_profile_restore_.known_good = known_good;
  queued_profile_restore_.family = TxPacketFamily::MAIN;
  queued_profile_restore_.generation++;
  queued_profile_restore_.request_ms = tx_request_ms_;

  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;

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

  update_dd_b1_bit_sensors_();
  flush_deferred_user_tx_();

  // Reset UART cycle
  processing_updates_ = false;
  last_process_time_ = millis();
  if (pending_tx_.active || pending_restore_.active || pending_profile_restore_.active ||
      pending_time_band_tx_.active ||
      tx_ui_sync_.active || restore_ui_sync_.active || profile_ui_sync_.active ||
      time_band_ui_sync_.active ||
      continuous_rx_) {
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
  std::map<std::string, bool> binary_sensor_values = {
      {DIG1_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x01)},
      {DIG2_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x02)},
      {DIG3_CONFIG, (bool)(buffer[DD_PACKET_DIG_IDX] & 0x04)},
      {P01_TANK_LOWER_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x04)},
      {P02_TANK_UPPER_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x02)},
      {P03_DEFROST_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x01)},
      {P04_INLET_AIR_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x08)},
      {P05_EVAPORATOR_INLET_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x80)},
      {P06_EVAPORATOR_OUTLET_PROBE_FAULT, (bool)(buffer[DD_PACKET_ALARM_IDX] & 0x01)},
      {P07_COMPRESSOR_FLOW_PROBE_FAULT, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x10)},
      {P08_SOLAR_COLLECTOR_PROBE_FAULT, (bool)(buffer[DD_PACKET_ALARM2_IDX] & 0x01)},
      {E01_HIGH_PRESSURE_PROTECTION, (bool)(buffer[DD_PACKET_PROBE_FAULT_IDX] & 0x40)},
      {E02_SOLAR_RECIRCULATION_ALARM, (bool)(buffer[DD_PACKET_ALARM_IDX] & 0x02)},
      {E03_ELECTRONIC_FAN_FAULT, (bool)(buffer[DD_PACKET_ALARM2_IDX] & 0x08)},
      {PA_HEAT_PUMP_TEMP_UNSUITABLE_ALARM, (bool)(buffer[DD_PACKET_ALARM_IDX] & 0x10)},
  };

  for (const auto &entry : binary_sensor_values) {
    set_binary_sensor_value(entry.first, entry.second);
  }

  return;
}

void DaikinEkhheComponent::update_time_band_state_from_bus_(const std::vector<uint8_t> &buffer,
                                                            bool d2_packet, bool force) {
  const uint8_t flag_idx =
      d2_packet ? D2_PACKET_TIME_BAND_FLAG_IDX : CC_PACKET_TIME_BAND_FLAG_IDX;
  const uint8_t start_hour_idx =
      d2_packet ? D2_PACKET_TIME_BAND_START_HOUR_IDX : CC_PACKET_TIME_BAND_START_HOUR_IDX;
  const uint8_t start_minute_idx =
      d2_packet ? D2_PACKET_TIME_BAND_START_MINUTE_IDX : CC_PACKET_TIME_BAND_START_MINUTE_IDX;
  const uint8_t end_hour_idx =
      d2_packet ? D2_PACKET_TIME_BAND_END_HOUR_IDX : CC_PACKET_TIME_BAND_END_HOUR_IDX;
  const uint8_t end_minute_idx =
      d2_packet ? D2_PACKET_TIME_BAND_END_MINUTE_IDX : CC_PACKET_TIME_BAND_END_MINUTE_IDX;
  const uint8_t mode_idx =
      d2_packet ? D2_PACKET_TIME_BAND_MODE_IDX : CC_PACKET_TIME_BAND_MODE_IDX;

  if (buffer.size() <= mode_idx) {
    return;
  }
  if ((time_band_state_.staged_dirty || time_band_ui_sync_.active) && !force) {
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

  for (const auto &entry : SELECT_BITMASKS) {
    const std::string &param_name = entry.first;
    const SelectBitmaskSpec &field = entry.second;
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
        pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN &&
        pending_tx_.index == CC_PACKET_MASK2_IDX &&
        pending_tx_.bit_position == SILENT_MODE_BIT_POSITION;
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
  const ProfileState &profile_sync_profile =
      profile_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
  const bool profile_d4_sync_matched =
      profile_ui_sync_.active && profile_sync_profile.valid &&
      profile_matches_packet(true, profile_sync_profile.extended_data,
                             profile_sync_profile.extended_length, buffer, true);
  const bool pending_profile_active = pending_profile_restore_.active;

  for (const auto &entry : U_NUMBER_EXTENDED_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if (param_index >= buffer.size()) {
      continue;
    }
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::EXTENDED &&
         pending_tx_.index == param_index && pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_d4_sync_matched)) &&
        is_profile_managed_field(true, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    set_number_value(param_name, buffer[param_index]);
  }
  for (const auto &entry : SELECT_EXTENDED_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if (param_index >= buffer.size()) {
      continue;
    }
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::EXTENDED &&
         pending_tx_.index == param_index && pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::EXTENDED, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_d4_sync_matched)) &&
        is_profile_managed_field(true, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    set_select_value(param_name, buffer[param_index]);
  }
}

void DaikinEkhheComponent::parse_c1_packet(std::vector<uint8_t> buffer) {
  if (restore_ui_sync_.active &&
      restore_defaults_match_packet(buffer, false, true)) {
    restore_ui_sync_.extended_synced = true;
    if (restore_ui_sync_.main_synced) {
      DAIKIN_DBG(TAG, "Restore defaults UI synced: cc_c1_cycles=%u",
                 restore_ui_sync_.cycles_waited + 1);
      restore_ui_sync_.active = false;
      restore_ui_sync_.main_synced = false;
      restore_ui_sync_.extended_synced = false;
      restore_ui_sync_.cycles_waited = 0;
    }
  }

  if (profile_ui_sync_.active) {
    const ProfileState &profile_sync_profile =
        profile_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
    if (profile_sync_profile.valid &&
        profile_matches_packet(true, profile_sync_profile.extended_data,
                               profile_sync_profile.extended_length, buffer, false)) {
      profile_ui_sync_.extended_synced = true;
      if (profile_ui_sync_.main_synced) {
        DAIKIN_DBG(TAG, "%s UI synced: cc_c1_cycles=%u",
                   profile_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                   profile_ui_sync_.cycles_waited + 1);
        profile_ui_sync_.active = false;
        profile_ui_sync_.main_synced = false;
        profile_ui_sync_.extended_synced = false;
        profile_ui_sync_.cycles_waited = 0;
      }
    }
  }

  if (!tx_ui_sync_.active || tx_ui_sync_.family != TxPacketFamily::EXTENDED) {
    return;
  }

  const bool c1_sync_matched =
      field_matches_target_(buffer, tx_ui_sync_.index, tx_ui_sync_.value,
                            tx_ui_sync_.bit_position, tx_ui_sync_.bit_width);
  if (c1_sync_matched) {
    DAIKIN_DBG(TAG, "TX UI synced: family=extended index=%u bit=%u value=0x%02X c1_cycles=%u",
               tx_ui_sync_.index, tx_ui_sync_.bit_position, tx_ui_sync_.value,
               tx_ui_sync_.cycles_waited + 1);
    tx_ui_sync_.active = false;
    tx_ui_sync_.cycles_waited = 0;
    return;
  }

  tx_ui_sync_.cycles_waited++;
  if (tx_ui_sync_.cycles_waited >= kTxUiSyncMaxCycles) {
    DAIKIN_WARN(TAG, "TX UI sync timeout: family=extended index=%u bit=%u value=0x%02X cycles=%u",
                tx_ui_sync_.index, tx_ui_sync_.bit_position, tx_ui_sync_.value,
                tx_ui_sync_.cycles_waited);
    tx_ui_sync_.active = false;
    tx_ui_sync_.cycles_waited = 0;
  }
}

void DaikinEkhheComponent::parse_cc_packet(std::vector<uint8_t> buffer) {
  const bool cc_sync_matched =
      tx_ui_sync_.active && tx_ui_sync_.family == TxPacketFamily::MAIN &&
      field_matches_target_(buffer, tx_ui_sync_.index, tx_ui_sync_.value,
                            tx_ui_sync_.bit_position, tx_ui_sync_.bit_width);
  const bool restore_cc_sync_matched =
      restore_ui_sync_.active && restore_defaults_match_packet(buffer, false, false);
  const ProfileState &profile_sync_profile =
      profile_ui_sync_.known_good ? known_good_profile_ : auto_snapshot_;
  const bool profile_cc_sync_matched =
      profile_ui_sync_.active && profile_sync_profile.valid &&
      profile_matches_packet(false, profile_sync_profile.main_data,
                             profile_sync_profile.main_length, buffer, false);
  const bool time_band_cc_sync_matched =
      time_band_ui_sync_.active &&
      time_band_matches_packet_(buffer, false, time_band_ui_sync_.flag,
                                time_band_ui_sync_.start_hour, time_band_ui_sync_.start_minute,
                                time_band_ui_sync_.end_hour, time_band_ui_sync_.end_minute,
                                time_band_ui_sync_.mode);
  const bool pending_profile_active = pending_profile_restore_.active;

  update_time_band_state_from_bus_(buffer, false, time_band_cc_sync_matched);

  // update numbers, unsigned and signed separately
  for (const auto &entry : U_NUMBER_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN && pending_tx_.index == param_index &&
         pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    if ((pending_restore_.active || (restore_ui_sync_.active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    uint8_t value = buffer[param_index];
    set_number_value(param_name, value);
  }

  for (const auto &entry : I_NUMBER_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN && pending_tx_.index == param_index &&
         pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    if ((pending_restore_.active || (restore_ui_sync_.active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    int8_t value = static_cast<int8_t>(buffer[param_index]);  // cast as signed
    set_number_value(param_name, value);
  }

  // Process Standard Selects (Full-Byte Values)
  for (const auto &entry : SELECT_PARAM_INDEX) {
    const std::string &param_name = entry.first;
    uint8_t param_index = entry.second;
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN && pending_tx_.index == param_index &&
         pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == BIT_POSITION_NO_BITMASK && !cc_sync_matched) {
      continue;
    }
    if ((pending_restore_.active || (restore_ui_sync_.active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, BIT_POSITION_NO_BITMASK)) {
      continue;
    }
    uint8_t value = buffer[param_index];
    set_select_value(param_name, value);
  }

  // Process Bitmask-Based Selects (Modify Only One Bit)
  for (const auto &entry : SELECT_BITMASKS) {
    const std::string &param_name = entry.first;
    const SelectBitmaskSpec &field = entry.second;
    uint8_t param_index = field.index;
    uint8_t bit_position = field.bit_position;
    uint8_t bit_width = field.bit_width;
    if ((pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN && pending_tx_.index == param_index &&
         pending_tx_.bit_position == bit_position) ||
        has_deferred_user_tx_(TxPacketFamily::MAIN, param_index, bit_position)) {
      continue;
    }
    if (tx_ui_sync_.active && tx_ui_sync_.index == param_index &&
        tx_ui_sync_.bit_position == bit_position && !cc_sync_matched) {
      continue;
    }
    if ((pending_restore_.active || (restore_ui_sync_.active && !restore_cc_sync_matched)) &&
        is_restore_scope_field(param_index, bit_position)) {
      continue;
    }
    if ((pending_profile_active || (profile_ui_sync_.active && !profile_cc_sync_matched)) &&
        is_profile_managed_field(false, param_index, bit_position)) {
      continue;
    }
    uint8_t value = extract_field_value(buffer, param_index, bit_position,
                                        effective_bit_width(bit_position, bit_width));
    set_select_value(param_name, value);
  }

#if defined(USE_SWITCH)
  if (CC_PACKET_MASK2_IDX < buffer.size()) {
    const bool pending_silent_write =
        pending_tx_.active && pending_tx_.family == TxPacketFamily::MAIN &&
        pending_tx_.index == CC_PACKET_MASK2_IDX &&
        pending_tx_.bit_position == SILENT_MODE_BIT_POSITION;
    const bool deferred_silent_write =
        has_deferred_user_tx_(TxPacketFamily::MAIN, CC_PACKET_MASK2_IDX, SILENT_MODE_BIT_POSITION);
    const bool waiting_for_silent_ui_sync =
        tx_ui_sync_.active && tx_ui_sync_.family == TxPacketFamily::MAIN &&
        tx_ui_sync_.index == CC_PACKET_MASK2_IDX &&
        tx_ui_sync_.bit_position == SILENT_MODE_BIT_POSITION && !cc_sync_matched;
    const bool waiting_for_silent_profile_sync =
        (pending_profile_active || (profile_ui_sync_.active && !profile_cc_sync_matched)) &&
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

  if (tx_ui_sync_.active && tx_ui_sync_.family == TxPacketFamily::MAIN) {
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

  if (restore_ui_sync_.active) {
    if (restore_cc_sync_matched) {
      restore_ui_sync_.main_synced = true;
    }
    if (restore_ui_sync_.main_synced && restore_ui_sync_.extended_synced) {
      DAIKIN_DBG(TAG, "Restore defaults UI synced: cc_c1_cycles=%u", restore_ui_sync_.cycles_waited + 1);
      restore_ui_sync_.active = false;
      restore_ui_sync_.main_synced = false;
      restore_ui_sync_.extended_synced = false;
      restore_ui_sync_.cycles_waited = 0;
    } else {
      restore_ui_sync_.cycles_waited++;
      if (restore_ui_sync_.cycles_waited >= kRestoreUiSyncMaxCycles) {
        uint8_t current_value = 0;
        const bool extended_pending = restore_ui_sync_.main_synced && !restore_ui_sync_.extended_synced;
        const std::vector<uint8_t> &sync_buffer = extended_pending ? last_c1_packet_ : buffer;
        const RestoreFieldSpec *field = first_restore_mismatch(sync_buffer, false, current_value,
                                                               extended_pending);
        if (field != nullptr) {
          DAIKIN_WARN(TAG,
                      "Restore defaults UI sync timeout: family=%s first_mismatch=%s expected=0x%02X current=0x%02X cc_c1_cycles=%u",
                      extended_pending ? "extended" : "main",
                      field->name, field->value, current_value, restore_ui_sync_.cycles_waited);
        } else {
          DAIKIN_WARN(TAG,
                      "Restore defaults UI sync timeout: main_synced=%u extended_synced=%u cc_c1_cycles=%u",
                      restore_ui_sync_.main_synced, restore_ui_sync_.extended_synced,
                      restore_ui_sync_.cycles_waited);
        }
        restore_ui_sync_.active = false;
        restore_ui_sync_.main_synced = false;
        restore_ui_sync_.extended_synced = false;
        restore_ui_sync_.cycles_waited = 0;
      }
    }
  }

  if (profile_ui_sync_.active) {
    if (profile_cc_sync_matched) {
      profile_ui_sync_.main_synced = true;
    }
    if (profile_ui_sync_.main_synced && profile_ui_sync_.extended_synced) {
      DAIKIN_DBG(TAG, "%s UI synced: cc_c1_cycles=%u",
                 profile_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                 profile_ui_sync_.cycles_waited + 1);
      profile_ui_sync_.active = false;
      profile_ui_sync_.main_synced = false;
      profile_ui_sync_.extended_synced = false;
      profile_ui_sync_.cycles_waited = 0;
    } else {
      profile_ui_sync_.cycles_waited++;
      if (profile_ui_sync_.cycles_waited >= kProfileUiSyncMaxCycles) {
        uint8_t expected_value = 0;
        uint8_t current_value = 0;
        const bool extended_pending = profile_ui_sync_.main_synced && !profile_ui_sync_.extended_synced;
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
                      profile_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                      extended_pending ? "extended" : "main",
                      field->name, expected_value, current_value, profile_ui_sync_.cycles_waited);
        } else {
          DAIKIN_WARN(TAG, "%s UI sync timeout: main_synced=%u extended_synced=%u cc_c1_cycles=%u",
                      profile_ui_sync_.known_good ? "Known-good profile" : "Auto snapshot",
                      profile_ui_sync_.main_synced, profile_ui_sync_.extended_synced,
                      profile_ui_sync_.cycles_waited);
        }
        profile_ui_sync_.active = false;
        profile_ui_sync_.main_synced = false;
        profile_ui_sync_.extended_synced = false;
        profile_ui_sync_.cycles_waited = 0;
      }
    }
  }

  if (time_band_ui_sync_.active) {
    if (time_band_cc_sync_matched) {
      DAIKIN_DBG(TAG, "Time-band UI synced: cc_cycles=%u", time_band_ui_sync_.cycles_waited + 1);
      time_band_ui_sync_.active = false;
      time_band_ui_sync_.cycles_waited = 0;
    } else {
      time_band_ui_sync_.cycles_waited++;
      if (time_band_ui_sync_.cycles_waited >= kTimeBandUiSyncMaxCycles) {
        DAIKIN_WARN(TAG,
                    "Time-band UI sync timeout: expected flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u cc_cycles=%u",
                    time_band_ui_sync_.flag, time_band_ui_sync_.start_hour,
                    time_band_ui_sync_.start_minute, time_band_ui_sync_.end_hour,
                    time_band_ui_sync_.end_minute, time_band_ui_sync_.mode,
                    time_band_ui_sync_.cycles_waited);
        time_band_ui_sync_.active = false;
        time_band_ui_sync_.cycles_waited = 0;
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
  if (pending_time_band_tx_.active || queued_time_band_tx_.active || queued_time_band_tx_.scheduled) {
    DAIKIN_WARN(TAG, "Time-band command already in progress.");
    return;
  }
  if (pending_tx_.active || tx_ui_sync_.active || pending_restore_.active || restore_ui_sync_.active ||
      pending_profile_restore_.active || profile_ui_sync_.active || time_band_ui_sync_.active) {
    DAIKIN_WARN(TAG, "Time-band command blocked: another write is currently active.");
    return;
  }

  tx_request_ms_ = millis();
  pending_time_band_tx_.active = true;
  pending_time_band_tx_.flag = flag;
  pending_time_band_tx_.start_hour = start_hour;
  pending_time_band_tx_.start_minute = start_minute;
  pending_time_band_tx_.end_hour = end_hour;
  pending_time_band_tx_.end_minute = end_minute;
  pending_time_band_tx_.mode = mode;
  pending_time_band_tx_.attempts_sent = 0;
  pending_time_band_tx_.last_attempt_d2_seq = 0;
  time_band_ui_sync_.active = false;
  time_band_ui_sync_.cycles_waited = 0;

  reset_queued_time_band_();
  queued_time_band_tx_.active = true;
  queued_time_band_tx_.scheduled = false;
  queued_time_band_tx_.generation++;
  queued_time_band_tx_.request_ms = tx_request_ms_;

  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;

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
  if (pending_profile_restore_.active || profile_ui_sync_.active) {
    DAIKIN_WARN(TAG, "Restore defaults blocked: a profile restore is currently active.");
    return;
  }
  if (pending_restore_.active || restore_ui_sync_.active) {
    DAIKIN_WARN(TAG, "Restore defaults already in progress.");
    return;
  }
  if (pending_tx_.active || tx_ui_sync_.active ||
      pending_time_band_tx_.active || queued_time_band_tx_.active ||
      queued_time_band_tx_.scheduled || time_band_ui_sync_.active) {
    DAIKIN_WARN(TAG, "Restore defaults blocked: another write is currently active.");
    return;
  }

  tx_request_ms_ = millis();
  pending_restore_.active = true;
  pending_restore_.family = TxPacketFamily::MAIN;
  pending_restore_.main_applied = false;
  pending_restore_.extended_applied = false;
  pending_restore_.main_write_sent = false;
  pending_restore_.extended_write_sent = false;
  pending_restore_.attempts_sent = 0;
  pending_restore_.last_attempt_d2_seq = 0;
  restore_ui_sync_.active = false;
  restore_ui_sync_.main_synced = false;
  restore_ui_sync_.extended_synced = false;
  restore_ui_sync_.cycles_waited = 0;

  reset_queued_restore_();
  queued_restore_.active = true;
  queued_restore_.scheduled = false;
  queued_restore_.family = TxPacketFamily::MAIN;
  queued_restore_.generation++;
  queued_restore_.request_ms = tx_request_ms_;

  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;

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

    // Get the CC array index from map, send UART command and update UI state
    auto it_u = U_NUMBER_PARAM_INDEX.find(name);
    auto it_i = I_NUMBER_PARAM_INDEX.find(name);
    auto it_ext = U_NUMBER_EXTENDED_PARAM_INDEX.find(name);
    if (it_u != U_NUMBER_PARAM_INDEX.end()) {
        uint8_t index = it_u->second;
        if (this->parent_->send_uart_cc_command(index, (uint8_t)value, BIT_POSITION_NO_BITMASK)) {
            this->parent_->update_number_cache(name, value);
            this->publish_state(value);
        }
    }
    else if (it_i != I_NUMBER_PARAM_INDEX.end()) {
        uint8_t index = it_i->second;
        if (this->parent_->send_uart_cc_command(index, (int8_t)value, BIT_POSITION_NO_BITMASK)) {
            this->parent_->update_number_cache(name, value);
            this->publish_state(value);
        }
    }
    else if (it_ext != U_NUMBER_EXTENDED_PARAM_INDEX.end()) {
        uint8_t index = it_ext->second;
        if (this->parent_->send_uart_c2_command(index, (uint8_t)value, BIT_POSITION_NO_BITMASK)) {
            this->parent_->update_number_cache(name, value);
            this->publish_state(value);
        }
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

    if (name == TIME_BAND_MODE) {
        if (this->parent_->stage_time_band_mode(uart_value)) {
            this->parent_->update_select_cache(name, value);
            this->publish_state(value);
        }
        return;
    }

    bool extended_family = false;

    // Look up the packet index for this select entity
    auto index_it = SELECT_PARAM_INDEX.find(name);
    uint8_t param_index = PARAM_INDEX_INVALID;
    if (index_it != SELECT_PARAM_INDEX.end()) {
      param_index = index_it->second;
    } else {
      auto extended_index_it = SELECT_EXTENDED_PARAM_INDEX.find(name);
      if (extended_index_it != SELECT_EXTENDED_PARAM_INDEX.end()) {
        param_index = extended_index_it->second;
        extended_family = true;
      }
    }

    // Look up if this select is part of a bitmask
    uint8_t bit_position = BIT_POSITION_NO_BITMASK;  // Default to no bitmask
    uint8_t bit_width = 1;
    auto bitmask_it = SELECT_BITMASKS.find(name);
    if (bitmask_it != SELECT_BITMASKS.end()) {
        param_index = bitmask_it->second.index;
        bit_position = bitmask_it->second.bit_position;
        bit_width = bitmask_it->second.bit_width;
        extended_family = false;
    } else {
            DAIKIN_DBG(TAG, "Select %s not in SELECT_BITMASKS", name.c_str());
    }

    if (param_index == PARAM_INDEX_INVALID) {
        DAIKIN_WARN(TAG, "No matching UART command for Select: %s", name.c_str());
        return;
    }

    // Update value in ESPHome
    bool sent = extended_family
                    ? this->parent_->send_uart_c2_command(param_index, uart_value, bit_position, bit_width)
                    : this->parent_->send_uart_cc_command(param_index, uart_value, bit_position, bit_width);
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
    ESP_LOGI(TAG, "TX time band sent: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u attempt=%u/%u len=%u",
             pending_time_band_tx_.flag, pending_time_band_tx_.start_hour,
             pending_time_band_tx_.start_minute, pending_time_band_tx_.end_hour,
             pending_time_band_tx_.end_minute, pending_time_band_tx_.mode,
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
      pending_tx_.active = false;
      pending_tx_.attempts_sent = 0;
      pending_tx_.last_attempt_d2_seq = 0;
      queued_tx_.active = false;
      queued_tx_.scheduled = false;
      tx_ui_sync_.active = false;
      tx_ui_sync_.cycles_waited = 0;
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
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    if (apply_change) {
      pending_tx_.active = false;
      pending_tx_.attempts_sent = 0;
      pending_tx_.last_attempt_d2_seq = 0;
      queued_tx_.active = false;
      queued_tx_.scheduled = false;
      tx_ui_sync_.active = false;
      tx_ui_sync_.cycles_waited = 0;
      if (!uart_active_ && !uart_tx_active_) {
        start_uart_cycle();
      }
    }
    return;
  }

  send_prebuilt_cd_packet_(family, command, apply_change ? TxPacketKind::SINGLE_FIELD : TxPacketKind::SNAPSHOT,
                           index, value, bit_position, bit_width, pending_tx_.attempts_sent);
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
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_restore_();
    reset_queued_restore_();
    restore_ui_sync_.active = false;
    restore_ui_sync_.main_synced = false;
    restore_ui_sync_.extended_synced = false;
    restore_ui_sync_.cycles_waited = 0;
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(family, command, TxPacketKind::RESTORE_DEFAULTS, 0, 0, BIT_POSITION_NO_BITMASK,
                           0, pending_restore_.attempts_sent);
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
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_profile_restore_();
    reset_queued_profile_restore_();
    profile_ui_sync_.active = false;
    profile_ui_sync_.main_synced = false;
    profile_ui_sync_.extended_synced = false;
    profile_ui_sync_.cycles_waited = 0;
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
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_profile_restore_();
    reset_queued_profile_restore_();
    profile_ui_sync_.active = false;
    profile_ui_sync_.main_synced = false;
    profile_ui_sync_.extended_synced = false;
    profile_ui_sync_.cycles_waited = 0;
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(family, command, TxPacketKind::PROFILE_RESTORE, 0, 0, BIT_POSITION_NO_BITMASK,
                           0, pending_profile_restore_.attempts_sent);
}

void DaikinEkhheComponent::send_time_band_packet_(const std::vector<uint8_t> &base_packet) {
  if (!pending_time_band_tx_.active) {
    return;
  }
  if (base_packet.empty()) {
    DAIKIN_WARN(TAG, "Time-band TX blocked: base CC packet empty.");
    reset_pending_time_band_();
    reset_queued_time_band_();
    return;
  }
  if (base_packet.size() != CC_PACKET_SIZE) {
    DAIKIN_WARN(TAG, "Time-band TX blocked: base CC packet length %u invalid.",
                static_cast<unsigned>(base_packet.size()));
    reset_pending_time_band_();
    reset_queued_time_band_();
    return;
  }

  std::vector<uint8_t> command = base_packet;
  command[0] = CD_PACKET_START_BYTE;
  command[CC_PACKET_TIME_BAND_FLAG_IDX] = pending_time_band_tx_.flag;
  command[CC_PACKET_TIME_BAND_START_HOUR_IDX] = pending_time_band_tx_.start_hour;
  command[CC_PACKET_TIME_BAND_START_MINUTE_IDX] = pending_time_band_tx_.start_minute;
  command[CC_PACKET_TIME_BAND_END_HOUR_IDX] = pending_time_band_tx_.end_hour;
  command[CC_PACKET_TIME_BAND_END_MINUTE_IDX] = pending_time_band_tx_.end_minute;
  command[CC_PACKET_TIME_BAND_MODE_IDX] = pending_time_band_tx_.mode;
  command.back() = ekhhe_checksum(command);

  std::string validation_error;
  if (!validate_outbound_cd_packet_(TxPacketFamily::MAIN, base_packet, command, TxPacketKind::TIME_BAND,
                                    0, 0, BIT_POSITION_NO_BITMASK, 0, validation_error)) {
    DAIKIN_WARN(TAG, "Time-band TX blocked by packet diff guard: %s", validation_error.c_str());
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_time_band_();
    reset_queued_time_band_();
    if (!uart_active_ && !uart_tx_active_) {
      start_uart_cycle();
    }
    return;
  }

  send_prebuilt_cd_packet_(TxPacketFamily::MAIN, command, TxPacketKind::TIME_BAND, 0, 0,
                           BIT_POSITION_NO_BITMASK, 0, pending_time_band_tx_.attempts_sent);
}

void DaikinEkhheComponent::check_pending_tx_(const std::vector<uint8_t> &buffer) {
  if (!pending_tx_.active) {
    return;
  }
  const auto &spec = tx_packet_family_spec_(pending_tx_.family);
  const uint8_t readback_index =
      tx_readback_index_(pending_tx_.family, pending_tx_.index, pending_tx_.bit_position);
  const uint8_t readback_bit_position =
      tx_readback_bit_position_(pending_tx_.family, pending_tx_.index, pending_tx_.bit_position);
  const uint8_t readback_bit_width =
      tx_readback_bit_width_(pending_tx_.family, pending_tx_.index,
                             pending_tx_.bit_position, pending_tx_.bit_width);
  if (readback_index >= buffer.size()) {
    pending_tx_.active = false;
    pending_tx_.family = TxPacketFamily::MAIN;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    queued_tx_.active = false;
    queued_tx_.scheduled = false;
    flush_deferred_user_tx_();
    return;
  }

  bool matched = field_matches_target_(buffer, readback_index, pending_tx_.value,
                                       readback_bit_position, readback_bit_width);

  if (matched) {
    if (pending_tx_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "TX already current: family=%s index=%u readback_index=%u value=0x%02X bit=%u",
                 spec.label, pending_tx_.index, readback_index, pending_tx_.value, pending_tx_.bit_position);
    } else {
      bool retried = pending_tx_.attempts_sent > 1;
      if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: family=%s readback=%s index=%u readback_index=%u value=0x%02X attempts=%u",
                      spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                      pending_tx_.index, readback_index, pending_tx_.value, pending_tx_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: family=%s readback=%s index=%u readback_index=%u value=0x%02X",
                   spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                   pending_tx_.index, readback_index, pending_tx_.value);
        }
      } else {
        uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                                  effective_bit_width(readback_bit_position, readback_bit_width));
        const uint8_t log_width = field_log_width(readback_index, readback_bit_position, readback_bit_width);
        if (retried) {
          DAIKIN_WARN(TAG, "TX applied after retries: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u value=%u attempts=%u",
                      spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                      pending_tx_.index, readback_index, pending_tx_.bit_position, readback_bit_position,
                      log_width, field_value, pending_tx_.attempts_sent);
        } else {
          ESP_LOGI(TAG, "TX applied: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u value=%u",
                   spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                   pending_tx_.index, readback_index, pending_tx_.bit_position, readback_bit_position,
                   log_width, field_value);
        }
      }
      DAIKIN_DBG(TAG, "TX timing: applied_after=%u attempts=%u", millis() - tx_sent_ms_, pending_tx_.attempts_sent);
      tx_ui_sync_.active = true;
      tx_ui_sync_.family = pending_tx_.family;
      tx_ui_sync_.index = pending_tx_.index;
      tx_ui_sync_.value = pending_tx_.value;
      tx_ui_sync_.bit_position = pending_tx_.bit_position;
      tx_ui_sync_.bit_width = pending_tx_.bit_width;
      tx_ui_sync_.cycles_waited = 0;
    }
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    pending_tx_.active = false;
    pending_tx_.family = TxPacketFamily::MAIN;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    queued_tx_.active = false;
    queued_tx_.scheduled = false;
    flush_deferred_user_tx_();
    return;
  }

  if (pending_tx_.attempts_sent == 0) {
    return;
  }

  if (pending_tx_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "TX retry pending: family=%s readback=%s index=%u readback_index=%u attempt=%u/%u",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               pending_tx_.index, readback_index, pending_tx_.attempts_sent, kTxMaxRepeats);
    return;
  }

  if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
    DAIKIN_WARN(TAG, "TX not applied: family=%s readback=%s index=%u readback_index=%u expected=0x%02X current=0x%02X",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                pending_tx_.index, readback_index, pending_tx_.value, buffer[readback_index]);
  } else {
    uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                              effective_bit_width(readback_bit_position, readback_bit_width));
    const uint8_t mask = field_value_mask(readback_index, readback_bit_position, readback_bit_width);
    const uint8_t log_width = field_log_width(readback_index, readback_bit_position, readback_bit_width);
    DAIKIN_WARN(TAG, "TX not applied: family=%s readback=%s index=%u readback_index=%u bit=%u readback_bit=%u width=%u expected=%u current=%u",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                pending_tx_.index, readback_index, pending_tx_.bit_position, readback_bit_position,
                log_width, pending_tx_.value & mask, field_value);
  }
  DAIKIN_DBG(TAG, "TX timing: failed_after=%u attempts=%u", millis() - tx_sent_ms_, pending_tx_.attempts_sent);
  uint8_t index = pending_tx_.index;
  if (pending_tx_.bit_position == BIT_POSITION_NO_BITMASK) {
    if (pending_tx_.family == TxPacketFamily::EXTENDED) {
      for (const auto &entry : U_NUMBER_EXTENDED_PARAM_INDEX) {
        if (entry.second == index) {
          set_number_value(entry.first, buffer[readback_index]);
        }
      }
      for (const auto &entry : SELECT_EXTENDED_PARAM_INDEX) {
        if (entry.second == index) {
          set_select_value(entry.first, buffer[readback_index]);
        }
      }
    } else {
      for (const auto &entry : U_NUMBER_PARAM_INDEX) {
        if (entry.second == index) {
          set_number_value(entry.first, buffer[readback_index]);
        }
      }
      for (const auto &entry : I_NUMBER_PARAM_INDEX) {
        if (entry.second == index) {
          set_number_value(entry.first, static_cast<int8_t>(buffer[readback_index]));
        }
      }
      for (const auto &entry : SELECT_PARAM_INDEX) {
        if (entry.second == index) {
          set_select_value(entry.first, buffer[readback_index]);
        }
      }
    }
  } else {
    uint8_t field_value = extract_field_value(buffer, readback_index, readback_bit_position,
                                              effective_bit_width(readback_bit_position, readback_bit_width));
    for (const auto &entry : SELECT_BITMASKS) {
      const SelectBitmaskSpec &field = entry.second;
      if (field.index == index && field.bit_position == pending_tx_.bit_position) {
        set_select_value(entry.first, field_value);
      }
    }
#if defined(USE_SWITCH)
    if (pending_tx_.family == TxPacketFamily::MAIN && index == CC_PACKET_MASK2_IDX &&
        pending_tx_.bit_position == SILENT_MODE_BIT_POSITION) {
      set_switch_value(SILENT_MODE, field_value != 0);
    }
#endif
  }
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
  pending_tx_.active = false;
  pending_tx_.family = TxPacketFamily::MAIN;
  pending_tx_.attempts_sent = 0;
  pending_tx_.last_attempt_d2_seq = 0;
  tx_ui_sync_.active = false;
  tx_ui_sync_.cycles_waited = 0;
  queued_tx_.active = false;
  queued_tx_.scheduled = false;
  flush_deferred_user_tx_();
}

void DaikinEkhheComponent::check_pending_restore_(const std::vector<uint8_t> &buffer) {
  if (!pending_restore_.active) {
    return;
  }

  const TxPacketFamily family = pending_restore_.family;
  const auto &spec = tx_packet_family_spec_(family);
  const bool extended = family == TxPacketFamily::EXTENDED;
  const bool matched = restore_defaults_match_packet(buffer, true, extended);
  if (matched) {
    if (family == TxPacketFamily::MAIN) {
      pending_restore_.main_applied = true;
      if (pending_restore_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "Restore defaults main block applied after retries: attempts=%u",
                    pending_restore_.attempts_sent);
      } else if (pending_restore_.attempts_sent > 0) {
        ESP_LOGI(TAG, "Restore defaults main block applied.");
      } else {
        DAIKIN_DBG(TAG, "Restore defaults main block already current.");
      }
      if (pending_restore_.attempts_sent > 0) {
        DAIKIN_DBG(TAG, "TX timing: restore_main_applied_after=%u attempts=%u",
                   millis() - tx_sent_ms_, pending_restore_.attempts_sent);
      }
      pending_restore_.family = TxPacketFamily::EXTENDED;
      pending_restore_.attempts_sent = 0;
      pending_restore_.last_attempt_d2_seq = 0;
      reset_queued_restore_();
      queued_restore_.active = true;
      queued_restore_.scheduled = false;
      queued_restore_.family = TxPacketFamily::EXTENDED;
      queued_restore_.generation++;
      queued_restore_.request_ms = tx_request_ms_;
      tx_waiting_for_first_rx_ = false;
      tx_waiting_for_first_cc_ = false;
      DAIKIN_DBG(TAG, "Restore defaults scheduling: main complete, waiting_for_next_d2 for extended");
      return;
    }

    pending_restore_.extended_applied = true;
    const bool wrote_any = pending_restore_.main_write_sent || pending_restore_.extended_write_sent;
    if (pending_restore_.attempts_sent == 0) {
      if (wrote_any) {
        ESP_LOGI(TAG, "Restore defaults applied.");
      } else {
        DAIKIN_DBG(TAG, "Restore defaults already current.");
      }
    } else {
      if (pending_restore_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "Restore defaults extended block applied after retries: attempts=%u",
                    pending_restore_.attempts_sent);
      } else {
        ESP_LOGI(TAG, "Restore defaults applied.");
      }
      DAIKIN_DBG(TAG, "TX timing: restore_extended_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, pending_restore_.attempts_sent);
    }
    if (wrote_any) {
      restore_ui_sync_.active = true;
      restore_ui_sync_.main_synced = false;
      restore_ui_sync_.extended_synced = false;
      restore_ui_sync_.cycles_waited = 0;
    }
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_restore_();
    reset_queued_restore_();
    return;
  }

  if (pending_restore_.attempts_sent == 0) {
    return;
  }

  if (pending_restore_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "Restore defaults retry pending: family=%s readback=%s attempt=%u/%u",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               pending_restore_.attempts_sent, kTxMaxRepeats);
    return;
  }

  uint8_t current_value = 0;
  const RestoreFieldSpec *field = first_restore_mismatch(buffer, true, current_value, extended);
  if (field != nullptr) {
    DAIKIN_WARN(TAG,
                "Restore defaults not applied: family=%s readback=%s first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u partial_main=%u partial_extended=%u",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                field->name, field->value, current_value, pending_restore_.attempts_sent,
                pending_restore_.main_applied, pending_restore_.extended_applied);
  } else {
    DAIKIN_WARN(TAG, "Restore defaults not applied: family=%s attempts=%u partial_main=%u partial_extended=%u",
                spec.label, pending_restore_.attempts_sent,
                pending_restore_.main_applied, pending_restore_.extended_applied);
  }
  DAIKIN_DBG(TAG, "TX timing: restore_failed_after=%u family=%s attempts=%u",
             millis() - tx_sent_ms_, spec.label, pending_restore_.attempts_sent);
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
  reset_pending_restore_();
  reset_queued_restore_();
}

void DaikinEkhheComponent::check_pending_profile_restore_(const std::vector<uint8_t> &buffer) {
  if (!pending_profile_restore_.active) {
    return;
  }

  const ProfileState &profile =
      pending_profile_restore_.known_good ? known_good_profile_ : auto_snapshot_;
  const TxPacketFamily family = pending_profile_restore_.family;
  const auto &spec = tx_packet_family_spec_(family);
  const bool extended = family == TxPacketFamily::EXTENDED;
  const uint8_t profile_length = extended ? profile.extended_length : profile.main_length;
  const uint8_t *profile_data = extended ? profile.extended_data : profile.main_data;
  if (!profile.valid || profile_length == 0) {
    DAIKIN_WARN(TAG, "%s restore aborted: stored profile missing during confirmation.",
                pending_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot");
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_profile_restore_();
    reset_queued_profile_restore_();
    return;
  }

  const bool matched = profile_matches_packet(extended, profile_data, profile_length, buffer, true);
  if (matched) {
    if (family == TxPacketFamily::MAIN) {
      pending_profile_restore_.main_applied = true;
      if (pending_profile_restore_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "%s main block applied after retries: attempts=%u",
                    pending_profile_restore_.known_good ? "Known-good profile restore"
                                                        : "Auto snapshot restore",
                    pending_profile_restore_.attempts_sent);
      } else if (pending_profile_restore_.attempts_sent > 0) {
        ESP_LOGI(TAG, "%s main block applied.",
                 pending_profile_restore_.known_good ? "Known-good profile restore"
                                                     : "Auto snapshot restore");
      } else {
        DAIKIN_DBG(TAG, "%s main block already current.",
                   pending_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot");
      }
      if (pending_profile_restore_.attempts_sent > 0) {
        DAIKIN_DBG(TAG, "TX timing: profile_restore_main_applied_after=%u attempts=%u",
                   millis() - tx_sent_ms_, pending_profile_restore_.attempts_sent);
      }
      pending_profile_restore_.family = TxPacketFamily::EXTENDED;
      pending_profile_restore_.attempts_sent = 0;
      pending_profile_restore_.last_attempt_d2_seq = 0;
      reset_queued_profile_restore_();
      queued_profile_restore_.active = true;
      queued_profile_restore_.scheduled = false;
      queued_profile_restore_.known_good = pending_profile_restore_.known_good;
      queued_profile_restore_.family = TxPacketFamily::EXTENDED;
      queued_profile_restore_.generation++;
      queued_profile_restore_.request_ms = tx_request_ms_;
      tx_waiting_for_first_rx_ = false;
      tx_waiting_for_first_cc_ = false;
      DAIKIN_DBG(TAG, "%s restore scheduling: main complete, waiting_for_next_d2 for extended",
                 pending_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot");
      return;
    }

    pending_profile_restore_.extended_applied = true;
    const bool wrote_any = pending_profile_restore_.main_write_sent ||
                           pending_profile_restore_.extended_write_sent;
    if (pending_profile_restore_.attempts_sent == 0) {
      if (wrote_any) {
        ESP_LOGI(TAG, "%s applied.",
                 pending_profile_restore_.known_good ? "Known-good profile restore"
                                                     : "Auto snapshot restore");
      } else {
        DAIKIN_DBG(TAG, "%s already current.",
                   pending_profile_restore_.known_good ? "Known-good profile" : "Auto snapshot");
      }
    } else {
      if (pending_profile_restore_.attempts_sent > 1) {
        DAIKIN_WARN(TAG, "%s extended block applied after retries: attempts=%u",
                    pending_profile_restore_.known_good ? "Known-good profile restore"
                                                        : "Auto snapshot restore",
                    pending_profile_restore_.attempts_sent);
      } else {
        ESP_LOGI(TAG, "%s applied.",
                 pending_profile_restore_.known_good ? "Known-good profile restore"
                                                     : "Auto snapshot restore");
      }
      DAIKIN_DBG(TAG, "TX timing: profile_restore_extended_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, pending_profile_restore_.attempts_sent);
    }
    if (wrote_any) {
      profile_ui_sync_.active = true;
      profile_ui_sync_.known_good = pending_profile_restore_.known_good;
      profile_ui_sync_.main_synced = false;
      profile_ui_sync_.extended_synced = false;
      profile_ui_sync_.cycles_waited = 0;
    }
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_profile_restore_();
    reset_queued_profile_restore_();
    return;
  }

  if (pending_profile_restore_.attempts_sent == 0) {
    return;
  }

  if (pending_profile_restore_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG, "%s retry pending: family=%s readback=%s attempt=%u/%u",
               pending_profile_restore_.known_good ? "Known-good profile restore"
                                                   : "Auto snapshot restore",
               spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
               pending_profile_restore_.attempts_sent, kTxMaxRepeats);
    return;
  }

  uint8_t expected_value = 0;
  uint8_t current_value = 0;
  const ManagedFieldSpec *field = first_profile_mismatch(extended, profile_data, profile_length, buffer, true,
                                                         expected_value, current_value);
  if (field != nullptr) {
    DAIKIN_WARN(TAG,
                "%s not applied: family=%s readback=%s first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u partial_main=%u partial_extended=%u",
                pending_profile_restore_.known_good ? "Known-good profile restore"
                                                    : "Auto snapshot restore",
                spec.label, packet_type_to_string_(spec.readback_packet_type).c_str(),
                field->name, expected_value, current_value, pending_profile_restore_.attempts_sent,
                pending_profile_restore_.main_applied, pending_profile_restore_.extended_applied);
  } else {
    DAIKIN_WARN(TAG, "%s not applied: family=%s attempts=%u partial_main=%u partial_extended=%u",
                pending_profile_restore_.known_good ? "Known-good profile restore"
                                                    : "Auto snapshot restore",
                spec.label, pending_profile_restore_.attempts_sent,
                pending_profile_restore_.main_applied, pending_profile_restore_.extended_applied);
  }
  DAIKIN_DBG(TAG, "TX timing: profile_restore_failed_after=%u family=%s attempts=%u",
             millis() - tx_sent_ms_, spec.label, pending_profile_restore_.attempts_sent);
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
  reset_pending_profile_restore_();
  reset_queued_profile_restore_();
}

void DaikinEkhheComponent::check_pending_time_band_(const std::vector<uint8_t> &buffer) {
  if (!pending_time_band_tx_.active) {
    return;
  }
  if (buffer.size() <= D2_PACKET_TIME_BAND_MODE_IDX) {
    DAIKIN_WARN(TAG, "Time-band readback packet too short: len=%u", static_cast<unsigned>(buffer.size()));
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_time_band_();
    reset_queued_time_band_();
    flush_deferred_user_tx_();
    return;
  }

  struct FieldCheck {
    const char *name;
    uint8_t index;
    uint8_t expected;
  };
  const FieldCheck fields[] = {
      {"flag", D2_PACKET_TIME_BAND_FLAG_IDX, pending_time_band_tx_.flag},
      {"start_hour", D2_PACKET_TIME_BAND_START_HOUR_IDX, pending_time_band_tx_.start_hour},
      {"start_minute", D2_PACKET_TIME_BAND_START_MINUTE_IDX, pending_time_band_tx_.start_minute},
      {"end_hour", D2_PACKET_TIME_BAND_END_HOUR_IDX, pending_time_band_tx_.end_hour},
      {"end_minute", D2_PACKET_TIME_BAND_END_MINUTE_IDX, pending_time_band_tx_.end_minute},
      {"mode", D2_PACKET_TIME_BAND_MODE_IDX, pending_time_band_tx_.mode},
  };

  const FieldCheck *first_mismatch = nullptr;
  for (const auto &field : fields) {
    if (buffer[field.index] != field.expected) {
      first_mismatch = &field;
      break;
    }
  }

  if (first_mismatch == nullptr) {
    if (pending_time_band_tx_.attempts_sent == 0) {
      DAIKIN_DBG(TAG, "Time-band already current: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
                 pending_time_band_tx_.flag, pending_time_band_tx_.start_hour,
                 pending_time_band_tx_.start_minute, pending_time_band_tx_.end_hour,
                 pending_time_band_tx_.end_minute, pending_time_band_tx_.mode);
    } else if (pending_time_band_tx_.attempts_sent > 1) {
      DAIKIN_WARN(TAG,
                  "Time-band applied after retries: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u attempts=%u",
                  pending_time_band_tx_.flag, pending_time_band_tx_.start_hour,
                  pending_time_band_tx_.start_minute, pending_time_band_tx_.end_hour,
                  pending_time_band_tx_.end_minute, pending_time_band_tx_.mode,
                  pending_time_band_tx_.attempts_sent);
      DAIKIN_DBG(TAG, "TX timing: time_band_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, pending_time_band_tx_.attempts_sent);
    } else {
      ESP_LOGI(TAG, "Time-band applied: flag=0x%02X start=%02u:%02u end=%02u:%02u mode=%u",
               pending_time_band_tx_.flag, pending_time_band_tx_.start_hour,
               pending_time_band_tx_.start_minute, pending_time_band_tx_.end_hour,
               pending_time_band_tx_.end_minute, pending_time_band_tx_.mode);
      DAIKIN_DBG(TAG, "TX timing: time_band_applied_after=%u attempts=%u",
                 millis() - tx_sent_ms_, pending_time_band_tx_.attempts_sent);
    }

    update_time_band_state_from_bus_(buffer, true, true);
    if (pending_time_band_tx_.attempts_sent > 0) {
      time_band_ui_sync_.active = true;
      time_band_ui_sync_.flag = pending_time_band_tx_.flag;
      time_band_ui_sync_.start_hour = pending_time_band_tx_.start_hour;
      time_band_ui_sync_.start_minute = pending_time_band_tx_.start_minute;
      time_band_ui_sync_.end_hour = pending_time_band_tx_.end_hour;
      time_band_ui_sync_.end_minute = pending_time_band_tx_.end_minute;
      time_band_ui_sync_.mode = pending_time_band_tx_.mode;
      time_band_ui_sync_.cycles_waited = 0;
    }
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;
    reset_pending_time_band_();
    reset_queued_time_band_();
    flush_deferred_user_tx_();
    return;
  }

  if (pending_time_band_tx_.attempts_sent == 0) {
    return;
  }

  if (pending_time_band_tx_.attempts_sent < kTxMaxRepeats) {
    DAIKIN_DBG(TAG,
               "Time-band retry pending: first_mismatch=%s expected=0x%02X current=0x%02X attempt=%u/%u",
               first_mismatch->name, first_mismatch->expected, buffer[first_mismatch->index],
               pending_time_band_tx_.attempts_sent, kTxMaxRepeats);
    return;
  }

  DAIKIN_WARN(TAG,
              "Time-band not applied: first_mismatch=%s expected=0x%02X current=0x%02X attempts=%u",
              first_mismatch->name, first_mismatch->expected, buffer[first_mismatch->index],
              pending_time_band_tx_.attempts_sent);
  DAIKIN_DBG(TAG, "TX timing: time_band_failed_after=%u attempts=%u",
             millis() - tx_sent_ms_, pending_time_band_tx_.attempts_sent);

  update_time_band_state_from_bus_(buffer, true, true);
  tx_waiting_for_first_rx_ = false;
  tx_waiting_for_first_cc_ = false;
  reset_pending_time_band_();
  reset_queued_time_band_();
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
    if (pending_restore_.active || restore_ui_sync_.active) {
        DAIKIN_WARN(TAG, "Restore defaults in progress, ignoring single-parameter write.");
        return false;
    }
    if (pending_profile_restore_.active || profile_ui_sync_.active) {
        DAIKIN_WARN(TAG, "Profile restore in progress, ignoring single-parameter write.");
        return false;
    }
    if (pending_time_band_tx_.active || queued_time_band_tx_.active ||
        queued_time_band_tx_.scheduled || time_band_ui_sync_.active) {
        return defer_single_field_tx_(family, index, value, bit_position, bit_width);
    }
    if (pending_tx_.active || queued_tx_.active || queued_tx_.scheduled || tx_ui_sync_.active) {
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
    pending_tx_.active = true;
    pending_tx_.family = family;
    pending_tx_.index = index;
    pending_tx_.value = value;
    pending_tx_.bit_position = bit_position;
    pending_tx_.bit_width = bit_width;
    pending_tx_.attempts_sent = 0;
    pending_tx_.last_attempt_d2_seq = 0;
    tx_ui_sync_.active = false;
    tx_ui_sync_.family = family;
    tx_ui_sync_.cycles_waited = 0;

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
    tx_waiting_for_first_rx_ = false;
    tx_waiting_for_first_cc_ = false;

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
