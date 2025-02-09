#pragma once

#include <string>
#include <map>
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "daikin_ekhhe_const.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/time/real_time_clock.h"


namespace esphome {
namespace daikin_ekkhe {

class DaikinEkhheNumber : public number::Number {
 public:
  void control(float value) override;
};

class DaikinEkhheSelect : public select::Select, public Component {
 public:
  void control(const std::string &value) override;
  void set_select_mappings(std::map<std::string, int> mappings) {
    this->select_mappings_ = std::move(mappings);
  }
  std::map<std::string, int> get_select_mappings() {
      return this->select_mappings_;
  }

  private:
   std::map<std::string, int> select_mappings_; // this stores the number to read/write for each select option
};

class DaikinEkhheComponent : public Component, public uart::UARTDevice {
 public:

  DaikinEkhheComponent() = default;
  enum EkhheError {
    EKHHE_ERROR_NONE,

    // from library
    EKHHE_ERROR_PACKET_SIZE, 
    EKHHE_ERROR_BUFFER_EMPTY,
    EKHHE_ERROR_CHECKSUM,
    EKHHE_ERROR_PACKET_END_CODE_MISSMATCH,
  };

  struct EkhheReading {
    uint16_t low_water_temp_probe;
  };
  // Nothing really public.

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void loop() override;
  void update();
  void dump_config() override;
  void on_shutdown();

  // Methods to register sensors, binary sensors, and numbers
  void register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor);
  void register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor);
  void register_number(const std::string &number_name, esphome::number::Number *number);
  void register_select(const std::string &select_name, select::Select *select);
  void register_timestamp_sensor(esphome::text_sensor::TextSensor *sensor);

  // Methods to update values dynamically (only for registered components)
  void set_sensor_value(const std::string &sensor_name, float value);
  void set_binary_sensor_value(const std::string &sensor_name, bool value);
  void set_number_value(const std::string &number_name, float value);
  void set_select_value(const std::string &select_name, int value);
  void update_timestamp(uint8_t hour, uint8_t minute);

  enum EkkheDDPacket {
    DD_PACKET_START_IDX = 0,
    DD_PACKET_B_IDX     = 6,
    DD_PACKET_A_IDX     = 7,
    DD_PACKET_C_IDX     = 8,
    DD_PACKET_D_IDX     = 10,
    DD_PACKET_E_IDX     = 11,
    DD_PACKET_F_IDX     = 12,
    DD_PACKET_G_IDX     = 13,
    DD_PACKET_H_IDX     = 14,
    DD_PACKET_I_IDX     = 18,
    DD_PACKET_DIG_IDX   = 21,
    DD_PACKET_END       = 40,
    DD_PACKET_SIZE      = 41,
  };

  enum EkhheD2Packet {
    D2_PACKET_START_IDX = 0,
    D2_PACKET_MASK1_IDX = 1,
    D2_PACKET_MASK2_IDX = 2,
    D2_PACKET_MODE_IDX  = 3,
    D2_PACKET_P4_IDX    = 4,  // CONFIRMED
    D2_PACKET_P7_IDX    = 5,
    D2_PACKET_P10_IDX   = 6,
    D2_PACKET_P2_IDX    = 7,  // CONFIRMED
    D2_PACKET_P29_IDX   = 9,
    D2_PACKET_P31_IDX   = 10,
    D2_PACKET_P8_IDX    = 11,
    D2_PACKET_P9_IDX    = 12,
    D2_PACKET_ECO_TTARGET_IDX       = 13,
    D2_PACKET_AUTO_TTARGET_IDX      = 14,
    D2_PACKET_BOOST_TTGARGET_IDX    = 15,
    D2_PACKET_ELECTRIC_TTARGET_IDX  = 16,
    // TODO: add indexes for target temp for other modes
    D2_PACKET_P1_IDX    = 20, // CONFIRMED
    D2_PACKET_P32_IDX   = 21,
    D2_PACKET_P3_IDX    = 22, // CONFIRMED
    D2_PACKET_P30_IDX   = 23,
    D2_PACKET_P25_IDX   = 24,
    D2_PACKET_P26_IDX   = 25,
    D2_PACKET_P27_IDX   = 26,
    D2_PACKET_P28_IDX   = 27,
    D2_PACKET_P12_IDX   = 28,
    D2_PACKET_P14_IDX   = 29,
    D2_PACKET_P24_IDX   = 30,
    D2_PACKET_P16_IDX   = 31,
    D2_PACKET_P23_IDX   = 32,
    D2_PACKET_P17_IDX   = 33, 
    D2_PACKET_P18_IDX   = 34, // TBC
    D2_PACKET_P19_IDX   = 35, // TBC
    D2_PACKET_P20_IDX   = 36, // TBC
    D2_PACKET_P21_IDX   = 37,
    D2_PACKET_P22_IDX   = 38, // TBC
    D2_PACKET_P34_IDX   = 39,
    D2_PACKET_P37_IDX   = 40,
    D2_PACKET_P38_IDX   = 41,
    D2_PACKET_P40_IDX   = 42,
    D2_PACKET_P36_IDX   = 43, // TBC
    D2_PACKET_P35_IDX   = 44,
    D2_PACKET_P41_IDX   = 45,
    D2_PACKET_P42_IDX   = 46,
    D2_PACKET_P43_IDX   = 47,
    D2_PACKET_P44_IDX   = 48,
    D2_PACKET_P45_IDX   = 49,
    D2_PACKET_P46_IDX   = 50,
    D2_PACKET_HOUR_IDX  = 56,
    D2_PACKET_MIN_IDX   = 57,
    D2_PACKET_P47_IDX   = 59, // TBC
    D2_PACKET_P48_IDX   = 60, // TBC
    D2_PACKET_P49_IDX   = 61, // TBC
    D2_PACKET_P50_IDX   = 62, // TBC
    D2_PACKET_P51_IDX   = 63, // TBC
    D2_PACKET_P52_IDX   = 64, // TBC 
    D2_PACKET_END       = 70, 
    D2_PACKET_SIZE      = 71,
  };

  enum EkhheD4Packet {
    D4_PACKET_START_IDX = 0,
    D4_PACKET_END       = 50,
    D4_PACKET_SIZE      = 51,
  };

  enum EkhheC1Packet {
    C1_PACKET_START_IDX = 0,
    C1_PACKET_END       = 50,
    C1_PACKET_SIZE      = 51,
  };

  enum EkhheCCPacket {
    CC_PACKET_START_IDX = 0,
    CC_PACKET_END       = 70,
    CC_PACKET_SIZE      = 71,
  };

  // This is the TX/control packet
  enum EkhheCDPacket {
    CD_PACKET_START_IDX = 0,
    CD_PACKET_END       = 70,
    CD_PACKET_SIZE      = 71,
  };



 private:
  std::map<std::string, esphome::sensor::Sensor *> sensors_;
  std::map<std::string, esphome::binary_sensor::BinarySensor *> binary_sensors_;
  std::map<std::string, esphome::number::Number *> numbers_;
  std::map<std::string, DaikinEkhheSelect *> selects_;
  text_sensor::TextSensor *timestamp_sensor_ = nullptr;

  esphome::time::RealTimeClock *clock;

  // UART Processing
  uint8_t ekhhe_checksum(const std::vector<uint8_t>& data_bytes);
  void parse_dd_packet();
  void parse_d2_packet();
  void parse_d4_packet();
  void parse_c1_packet();
  void parse_cc_packet();
  void print_buffer();

  std::vector<uint8_t> buffer_;  // Stores incoming UART bytes
  uint8_t expected_length_ = 0;  // Expected packet length
  bool receiving_ = false;       // If we're currently receiving a packet
  DaikinEkhheComponent::EkhheError process_uart_buffer();  
};

}  // namespace daikin_ekkhe
}  // namespace esphome