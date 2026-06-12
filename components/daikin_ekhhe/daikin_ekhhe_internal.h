#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace esphome {
namespace daikin_ekkhe {

constexpr uint8_t BIT_POSITION_NO_BITMASK = 255;
constexpr uint8_t BIT_WIDTH_P15_STEPPED = 255;

enum class WritablePacketFamily : uint8_t {
  MAIN,
  EXTENDED,
};

enum class NumberFieldEncoding : uint8_t {
  UINT8,
  INT8,
};

struct NumberFieldSpec {
  WritablePacketFamily family;
  uint8_t index;
  NumberFieldEncoding encoding;
};

struct SelectFieldSpec {
  WritablePacketFamily family;
  uint8_t index;
  uint8_t bit_position;
  uint8_t bit_width;
};

extern const std::map<std::string, NumberFieldSpec> NUMBER_FIELD_SPECS;
extern const std::map<std::string, SelectFieldSpec> SELECT_FIELD_SPECS;

}  // namespace daikin_ekkhe
}  // namespace esphome
