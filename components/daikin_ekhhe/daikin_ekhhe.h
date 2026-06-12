
#pragma once

#include <string>
#include <map>
#include <type_traits>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#if defined(USE_SWITCH)
#include "esphome/components/switch/switch.h"
#endif
#if defined(USE_BUTTON)
#include "esphome/components/button/button.h"
#endif
#if defined(USE_WATER_HEATER)
#include "esphome/components/water_heater/water_heater.h"
#endif
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/preferences.h"

#include "daikin_ekhhe_const.h"

namespace esphome {
namespace daikin_ekkhe {

class DaikinEkhheComponent;  // Forward declaration


class DaikinEkhheNumber : public number::Number {
 public:
    void control(float value) override;
    void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
    // Needed so we can set this from python and then reference it in the control function
    void set_internal_id(const std::string &id) { this->internal_id_ = id; }

  private:
    DaikinEkhheComponent *parent_;
    std::string internal_id_;
};

class DaikinEkhheSelect : public select::Select, public Component {
 public:
  void control(const std::string &value) override;
  void set_select_mappings(std::map<std::string, int> mappings) {
    this->select_mappings_ = std::move(mappings);
  }
  // this stores the number to read/write for each select option
  std::map<std::string, int> get_select_mappings() {
      return this->select_mappings_;
  }
  void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
  void set_internal_id(const std::string &id) { this->internal_id_ = id; }

  private:
   std::map<std::string, int> select_mappings_; 
   DaikinEkhheComponent *parent_;
  std::string internal_id_;
};

#if defined(USE_SWITCH)
class DaikinEkhheSwitch : public switch_::Switch {
 public:
  void write_state(bool state) override;
  void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
  void set_internal_id(const std::string &id) { this->internal_id_ = id; }

 private:
  DaikinEkhheComponent *parent_;
  std::string internal_id_;
};
#endif

#if defined(USE_BUTTON)
class DaikinEkhheActionButton : public button::Button {
 public:
  enum class Action : uint8_t {
    RESTORE_DEFAULT_SETTINGS,
    SAVE_KNOWN_GOOD_PROFILE,
    RESTORE_KNOWN_GOOD_PROFILE,
    RESTORE_AUTO_SNAPSHOT,
    APPLY_TIME_BAND,
    CLEAR_TIME_BAND,
  };

  explicit DaikinEkhheActionButton(Action action) : action_(action) {}
  void press_action() override;
  void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }

 private:
  DaikinEkhheComponent *parent_;
  Action action_;
};
#endif

#if defined(USE_WATER_HEATER)
enum WaterHeaterCurrentTemperatureSource : uint8_t {
  WATER_HEATER_TEMP_SOURCE_DISPLAY,
  WATER_HEATER_TEMP_SOURCE_UPPER,
  WATER_HEATER_TEMP_SOURCE_LOWER,
};

class DaikinEkhheWaterHeater : public water_heater::WaterHeater {
 public:
  water_heater::WaterHeaterCallInternal make_call() override;
  void control(const water_heater::WaterHeaterCall &call) override;
  void set_parent(DaikinEkhheComponent *parent) { this->parent_ = parent; }
  void set_current_temperature_source(WaterHeaterCurrentTemperatureSource source) {
    this->current_temperature_source_ = source;
  }
  WaterHeaterCurrentTemperatureSource get_current_temperature_source() const {
    return this->current_temperature_source_;
  }
  void publish_readback(float current_temperature, float target_temperature,
                        water_heater::WaterHeaterMode mode, bool on, bool away);

 protected:
  water_heater::WaterHeaterTraits traits() override;

 private:
  DaikinEkhheComponent *parent_{nullptr};
  WaterHeaterCurrentTemperatureSource current_temperature_source_{WATER_HEATER_TEMP_SOURCE_DISPLAY};
};
#endif

class DaikinEkhheComponent : public Component, public uart::UARTDevice {
 public:

  DaikinEkhheComponent() = default;
  enum EkhheError {
    EKHHE_ERROR_NONE,
    EKHHE_ERROR_PACKET_SIZE, 
    EKHHE_ERROR_BUFFER_EMPTY,
    EKHHE_ERROR_CHECKSUM,
  };


  // ========== INTERNAL METHODS ==========
  void setup() override;
  void loop() override;
  void update();
  void dump_config() override;
  void on_shutdown();
  void set_update_interval(int interval_ms);
  void set_continuous_rx(bool enabled);
  void set_tx_delay_after_d2_ms(uint32_t delay_ms);
  uint32_t get_tx_delay_after_d2_ms() const { return this->tx_delay_after_d2_ms_; }

  // Methods to register sensors, binary sensors, and numbers
  void register_sensor(const std::string &sensor_name, esphome::sensor::Sensor *sensor);
  void register_binary_sensor(const std::string &sensor_name, esphome::binary_sensor::BinarySensor *binary_sensor);
  void register_number(const std::string &number_name, esphome::number::Number *number);
  void register_select(const std::string &select_name, select::Select *select);
  void register_text_sensor(const std::string &text_sensor_name, esphome::text_sensor::TextSensor *sensor);
  void register_timestamp_sensor(esphome::text_sensor::TextSensor *sensor);
#if defined(USE_SWITCH)
  void register_switch(const std::string &switch_name, switch_::Switch *sw);
#endif
#if defined(USE_WATER_HEATER)
  void register_water_heater(DaikinEkhheWaterHeater *water_heater);
  bool request_water_heater_control_(const water_heater::WaterHeaterCall &call);
  bool republish_water_heater_state();
#endif
  void register_known_good_profile_status_sensor(esphome::text_sensor::TextSensor *sensor);
  void register_auto_snapshot_status_sensor(esphome::text_sensor::TextSensor *sensor);
  void set_heating_demand(binary_sensor::BinarySensor *sensor) { this->heating_demand_ = sensor; }
  void set_hp_active(binary_sensor::BinarySensor *sensor) { this->hp_active_ = sensor; }
  void set_eh_active(binary_sensor::BinarySensor *sensor) { this->eh_active_ = sensor; }

  // Methods to update values dynamically (only for registered components)
  void set_sensor_value(const std::string &sensor_name, float value);
  void set_binary_sensor_value(const std::string &sensor_name, bool value);
  void set_number_value(const std::string &number_name, float value);
  void set_select_value(const std::string &select_name, int value);
#if defined(USE_SWITCH)
  void set_switch_value(const std::string &switch_name, bool value);
#endif
  void update_timestamp(uint8_t hour, uint8_t minute);

  // Allow UART command sending for Number/Select control
  bool send_uart_cc_command(uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width = 1);
  bool send_uart_c2_command(uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width = 1);
  void restore_default_settings();
  void save_known_good_profile();
  void restore_known_good_profile();
  void restore_auto_snapshot();
  void apply_time_band();
  void clear_time_band();
  void update_number_cache(const std::string &number_name, float value);
  void update_select_cache(const std::string &select_name, const std::string &value);
  bool stage_time_band_number(const std::string &number_name, float value);
  bool stage_time_band_mode(uint8_t mode);
#if defined(USE_SWITCH)
  void update_switch_cache(const std::string &switch_name, bool value);
  bool set_silent_mode(bool enabled);
#endif


  enum EkkheDDPacket {
    DD_PACKET_START_IDX = 0,
    DD_PACKET_PROBE_FAULT_IDX = 3,
    DD_PACKET_ALARM_IDX = 4,
    DD_PACKET_ALARM2_IDX = 5,
    DD_PACKET_B_IDX     = 6,
    DD_PACKET_A_IDX     = 7,
    DD_PACKET_C_IDX     = 8,
    DD_PACKET_D_IDX     = 10,
    DD_PACKET_E_IDX     = 11,
    DD_PACKET_F_IDX     = 12,
    DD_PACKET_G_IDX     = 13,
    DD_PACKET_H_IDX     = 14,
    DD_PACKET_I_IDX     = 17,
    DD_PACKET_DIG_IDX     = 21,
    DD_PACKET_FAN_RPM_IDX = 26,
    DD_PACKET_J_FW_IDX    = 39,
    DD_PACKET_END       = 40,
    DD_PACKET_SIZE      = 41,
  };

  enum EkhheD2Packet {
    D2_PACKET_START_IDX = 0,
    D2_PACKET_MASK1_IDX = 1,
    D2_PACKET_MASK2_IDX = 2,
    D2_PACKET_MODE_IDX  = 3,
    D2_PACKET_P4_IDX    = 4,
    D2_PACKET_P7_IDX    = 5,
    D2_PACKET_P10_IDX   = 6,
    D2_PACKET_P2_IDX    = 7,
    D2_PACKET_VAC_DAYS  = 8,
    D2_PACKET_P29_IDX   = 9,
    D2_PACKET_P31_IDX   = 10,
    D2_PACKET_P8_IDX    = 11,
    D2_PACKET_P9_IDX    = 12,
    D2_PACKET_AUTO_TTARGET_IDX      = 13,
    D2_PACKET_ECO_TTARGET_IDX       = 14,
    D2_PACKET_BOOST_TTGARGET_IDX    = 15,
    D2_PACKET_ELECTRIC_TTARGET_IDX  = 16,
    D2_PACKET_P1_IDX    = 20,
    D2_PACKET_P32_IDX   = 21,
    D2_PACKET_P3_IDX    = 22,
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
    D2_PACKET_P18_IDX   = 34,
    D2_PACKET_P19_IDX   = 35,
    D2_PACKET_P20_IDX   = 36,
    D2_PACKET_P21_IDX   = 37,
    D2_PACKET_P22_IDX   = 38,
    D2_PACKET_P34_IDX   = 39,
    D2_PACKET_P37_IDX   = 40,
    D2_PACKET_P38_IDX   = 41,
    D2_PACKET_P40_IDX   = 42,
    D2_PACKET_P36_IDX   = 43,
    D2_PACKET_P35_IDX   = 44,
    D2_PACKET_P41_IDX   = 45,
    D2_PACKET_P42_IDX   = 46,
    D2_PACKET_P43_IDX   = 47,
    D2_PACKET_P44_IDX   = 48,
    D2_PACKET_P45_IDX   = 49,
    D2_PACKET_P46_IDX   = 50,
    D2_PACKET_TIME_BAND_FLAG_IDX = 51,
    D2_PACKET_TIME_BAND_START_HOUR_IDX = 52,
    D2_PACKET_TIME_BAND_START_MINUTE_IDX = 53,
    D2_PACKET_TIME_BAND_END_HOUR_IDX = 54,
    D2_PACKET_TIME_BAND_END_MINUTE_IDX = 55,
    D2_PACKET_HOUR_IDX  = 56,
    D2_PACKET_MIN_IDX   = 57,
    D2_PACKET_P47_IDX   = 59,
    D2_PACKET_P48_IDX   = 60,
    D2_PACKET_P49_IDX   = 61,
    D2_PACKET_P50_IDX   = 62,
    D2_PACKET_P51_IDX   = 63,
    D2_PACKET_P52_IDX   = 64,
    D2_PACKET_TIME_BAND_MODE_IDX = 65,
    D2_PACKET_P54_IDX   = 66,
    D2_PACKET_END       = 70,
    D2_PACKET_SIZE      = 71,
  };

  enum EkhheExtendedParameterPacket {
    EXT_PACKET_P53_IDX = 1,
    EXT_PACKET_P71_IDX = 2,
    EXT_PACKET_P58_IDX = 3,
    EXT_PACKET_P59_IDX = 4,
    EXT_PACKET_P56_IDX = 5,
    EXT_PACKET_P57_IDX = 6,
    EXT_PACKET_P60_IDX = 7,
    EXT_PACKET_P61_IDX = 8,
    EXT_PACKET_P62_IDX = 9,
    EXT_PACKET_P63_IDX = 10,
    EXT_PACKET_P64_IDX = 11,
    EXT_PACKET_P65_IDX = 12,
    EXT_PACKET_P55_IDX = 13,
    EXT_PACKET_P66_IDX = 14,
    EXT_PACKET_P67_IDX = 15,
    EXT_PACKET_P68_IDX = 16,
    EXT_PACKET_P69_IDX = 17,
    EXT_PACKET_P70_IDX = 18,
    EXT_PACKET_P72_IDX = 19,
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

  enum EkhheC2Packet {
    C2_PACKET_START_IDX = 0,
    C2_PACKET_END       = 50,
    C2_PACKET_SIZE      = 51,
  };

  enum EkhheCCPacket {
    CC_PACKET_START_IDX = 0,
    CC_PACKET_MASK1_IDX = 1,
    CC_PACKET_MASK2_IDX = 2,
    CC_PACKET_MODE_IDX  = 3,
    CC_PACKET_P4_IDX    = 4,
    CC_PACKET_P7_IDX    = 5,
    CC_PACKET_P10_IDX   = 6,
    CC_PACKET_P2_IDX    = 7,
    CC_PACKET_VAC_DAYS  = 8,
    CC_PACKET_P29_IDX   = 9,
    CC_PACKET_P31_IDX   = 10,
    CC_PACKET_P8_IDX    = 11,
    CC_PACKET_P9_IDX    = 12,
    CC_PACKET_AUTO_TTARGET_IDX      = 13,
    CC_PACKET_ECO_TTARGET_IDX       = 14,
    CC_PACKET_BOOST_TTGARGET_IDX    = 15,
    CC_PACKET_ELECTRIC_TTARGET_IDX  = 16,
    CC_PACKET_P1_IDX    = 20, 
    CC_PACKET_P32_IDX   = 21,
    CC_PACKET_P3_IDX    = 22, 
    CC_PACKET_P30_IDX   = 23,
    CC_PACKET_P25_IDX   = 24,
    CC_PACKET_P26_IDX   = 25,
    CC_PACKET_P27_IDX   = 26,
    CC_PACKET_P28_IDX   = 27,
    CC_PACKET_P12_IDX   = 28,
    CC_PACKET_P14_IDX   = 29,
    CC_PACKET_P24_IDX   = 30,
    CC_PACKET_P16_IDX   = 31,
    CC_PACKET_P23_IDX   = 32,
    CC_PACKET_P17_IDX   = 33, 
    CC_PACKET_P18_IDX   = 34,
    CC_PACKET_P19_IDX   = 35,
    CC_PACKET_P20_IDX   = 36,
    CC_PACKET_P21_IDX   = 37,
    CC_PACKET_P22_IDX   = 38,
    CC_PACKET_P34_IDX   = 39,
    CC_PACKET_P37_IDX   = 40,
    CC_PACKET_P38_IDX   = 41,
    CC_PACKET_P40_IDX   = 42,
    CC_PACKET_P36_IDX   = 43,
    CC_PACKET_P35_IDX   = 44,
    CC_PACKET_P41_IDX   = 45,
    CC_PACKET_P42_IDX   = 46,
    CC_PACKET_P43_IDX   = 47,
    CC_PACKET_P44_IDX   = 48,
    CC_PACKET_P45_IDX   = 49,
    CC_PACKET_P46_IDX   = 50,
    CC_PACKET_TIME_BAND_FLAG_IDX = 51,
    CC_PACKET_TIME_BAND_START_HOUR_IDX = 52,
    CC_PACKET_TIME_BAND_START_MINUTE_IDX = 53,
    CC_PACKET_TIME_BAND_END_HOUR_IDX = 54,
    CC_PACKET_TIME_BAND_END_MINUTE_IDX = 55,
    CC_PACKET_HOUR_IDX  = 57,
    CC_PACKET_MIN_IDX   = 58,
    CC_PACKET_P47_IDX   = 60, 
    CC_PACKET_P48_IDX   = 61, 
    CC_PACKET_P49_IDX   = 62, 
    CC_PACKET_P50_IDX   = 63,
    CC_PACKET_P51_IDX   = 64,
    CC_PACKET_P52_IDX   = 65,
    CC_PACKET_L_FW_IDX  = 66,
    CC_PACKET_TIME_BAND_MODE_IDX = 67,
    CC_PACKET_P54_IDX   = 68,
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
  static constexpr float kFloatPublishEpsilon = 0.5f;
  static constexpr uint32_t kFastPublishMinIntervalMs = 1000;
  static constexpr uint32_t kRuntimeSensorRefreshMs = 5 * 60 * 1000;
  static constexpr uint32_t kSlowPublishRefreshMs = 30 * 60 * 1000;
  static constexpr uint32_t kTimestampRefreshMs = 5 * 60 * 1000;
  static constexpr uint32_t kCycleTimeoutMs = 2000;
  static constexpr uint32_t kFrameReadTimeoutMs = 120;
  static constexpr size_t kRawFrameMaxLen = 71;
  static constexpr size_t kRawFrameBufferSize = 16;
  static constexpr uint8_t kBitPositionNoBitmask = 255;
  static constexpr uint32_t kDefaultTxDelayAfterD2Ms = 75;
  static constexpr uint32_t kMaxTxDelayAfterD2Ms = 250;
  static constexpr uint8_t kTxMaxRepeats = 5;
  static constexpr uint8_t kDeferredTxMax = 8;
  static constexpr uint8_t kTimeBandUiSyncMaxCycles = 3;
  static constexpr uint8_t kTimeBandApplyFlag = 0xFF;
  static constexpr uint8_t kTimeBandClearFlag = 0xFC;
  static constexpr uint8_t kTimeBandMaxMode = 4;
  static constexpr uint8_t kOperationalModeAuto = 0;
  static constexpr uint8_t kOperationalModeEco = 1;
  static constexpr uint8_t kOperationalModeBoost = 2;
  static constexpr uint8_t kOperationalModeElectric = 3;
  static constexpr uint8_t kOperationalModeFan = 4;
  static constexpr uint8_t kOperationalModeVacation = 5;

  static constexpr uint8_t kPacketMaskDD = 1 << 0;
  static constexpr uint8_t kPacketMaskD2 = 1 << 1;
  static constexpr uint8_t kPacketMaskD4 = 1 << 2;
  static constexpr uint8_t kPacketMaskC1 = 1 << 3;
  static constexpr uint8_t kPacketMaskCC = 1 << 4;
  static constexpr uint8_t kPacketMaskC2 = 1 << 5;
  static constexpr uint8_t kRequiredPacketMask = kPacketMaskDD | kPacketMaskD2 | kPacketMaskD4 | kPacketMaskC1 | kPacketMaskCC;
  static constexpr uint8_t kChecksumPacketMask = kPacketMaskDD | kPacketMaskD4 | kPacketMaskC1 | kPacketMaskC2 | kPacketMaskCC;

  // variables for sensors etc.
  std::map<std::string, esphome::sensor::Sensor *> sensors_;
  std::map<std::string, esphome::binary_sensor::BinarySensor *> binary_sensors_;
  std::map<std::string, esphome::number::Number *> numbers_;
  std::map<std::string, DaikinEkhheSelect *> selects_;
#if defined(USE_SWITCH)
  std::map<std::string, switch_::Switch *> switches_;
#endif
#if defined(USE_WATER_HEATER)
  DaikinEkhheWaterHeater *water_heater_ = nullptr;
  bool water_heater_have_temperatures_ = false;
  bool water_heater_have_main_state_ = false;
  bool water_heater_have_last_target_mode_ = false;
  bool water_heater_have_last_non_vacation_mode_ = false;
  bool water_heater_have_last_non_standby_mode_ = false;
  float water_heater_lower_temperature_ = 0.0f;
  float water_heater_upper_temperature_ = 0.0f;
  float water_heater_auto_target_ = 0.0f;
  float water_heater_eco_target_ = 0.0f;
  float water_heater_boost_target_ = 0.0f;
  float water_heater_electric_target_ = 0.0f;
  bool water_heater_power_on_ = false;
  uint8_t water_heater_operational_mode_ = kOperationalModeAuto;
  uint8_t water_heater_display_probe_ = 1;
  uint8_t water_heater_last_target_mode_ = kOperationalModeAuto;
  uint8_t water_heater_last_non_vacation_mode_ = kOperationalModeAuto;
  uint8_t water_heater_last_non_standby_mode_ = kOperationalModeAuto;
#endif
  std::map<std::string, esphome::text_sensor::TextSensor *> text_sensors_;
  text_sensor::TextSensor *timestamp_sensor_ = nullptr;
  text_sensor::TextSensor *known_good_profile_status_sensor_ = nullptr;
  text_sensor::TextSensor *auto_snapshot_status_sensor_ = nullptr;
  binary_sensor::BinarySensor *heating_demand_ = nullptr;
  binary_sensor::BinarySensor *hp_active_ = nullptr;
  binary_sensor::BinarySensor *eh_active_ = nullptr;
  esphome::time::RealTimeClock *clock = nullptr;

  // UART Processing
  uint8_t ekhhe_checksum(const std::vector<uint8_t>& data_bytes);
  void parse_dd_packet(std::vector<uint8_t> buffer);
  void parse_d2_packet(std::vector<uint8_t> buffer);
  void parse_d4_packet(std::vector<uint8_t> buffer);
  void parse_c1_packet(std::vector<uint8_t> buffer);
  void parse_cc_packet(std::vector<uint8_t> buffer);
  void start_uart_cycle();
  void process_packet_set();
  bool packet_set_complete();
  uint8_t read_rx_byte_();
  void reset_rx_frame_();
  void consume_uart_byte_(uint8_t byte, uint32_t now_ms);
  void check_rx_frame_timeout_(uint32_t now_ms);
  void handle_complete_packet_(uint8_t packet_type, const uint8_t *data, size_t length);
  void store_raw_frame_(uint8_t packet_type, const uint8_t *data, size_t length, uint8_t flags);
  void reset_cycle_stats_();
  uint8_t packet_mask_for_start_(uint8_t start_byte) const;
  std::string packet_mask_to_string_(uint8_t mask) const;
  bool should_publish_float_(const std::string &key, float value, std::map<std::string, float> &last_values,
                             std::map<std::string, uint32_t> &last_publish_ms, uint32_t min_interval_ms,
                             float epsilon, uint32_t refresh_ms);
  bool should_publish_bool_(const std::string &key, bool value, std::map<std::string, bool> &last_values,
                            std::map<std::string, uint32_t> &last_publish_ms, uint32_t refresh_ms);
  bool should_publish_text_(const std::string &key, const std::string &value,
                            std::map<std::string, std::string> &last_values,
                            std::map<std::string, uint32_t> &last_publish_ms, uint32_t refresh_ms);
  struct RawFrameEntry;

  void publish_profile_statuses_();
  void publish_profile_status_(bool known_good);
  std::string format_profile_status_(bool known_good) const;
  void load_persistent_profiles_();
  bool capture_current_packet_(uint8_t packet_type, const std::vector<uint8_t> &cache,
                               std::vector<uint8_t> &packet) const;
  bool capture_current_profile_packets_(std::vector<uint8_t> &main_packet,
                                        std::vector<uint8_t> &extended_packet) const;
  bool save_profile_(bool known_good, const std::vector<uint8_t> &main_packet,
                     const std::vector<uint8_t> &extended_packet);
  bool auto_save_snapshot_if_needed_();
  void restore_profile_(bool known_good);
  void update_time_band_state_from_bus_(const std::vector<uint8_t> &buffer, bool d2_packet,
                                        bool force = false);
#if defined(USE_WATER_HEATER)
  void update_water_heater_temperature_cache_(float lower_temperature, float upper_temperature);
  void update_water_heater_main_cache_from_bus_(const std::vector<uint8_t> &buffer, bool d2_packet);
  bool publish_water_heater_state_(bool force = false);
  bool water_heater_target_capable_mode_(uint8_t mode) const;
  uint8_t water_heater_readback_target_mode_() const;
  float water_heater_target_for_mode_(uint8_t mode) const;
  water_heater::WaterHeaterMode water_heater_native_mode_(uint8_t mode) const;
  bool water_heater_native_mode_to_operational_mode_(water_heater::WaterHeaterMode mode,
                                                     uint8_t &operational_mode) const;
  uint8_t water_heater_restore_mode_for_on_() const;
  uint8_t water_heater_restore_mode_for_away_clear_() const;
#endif
  void update_dd_b1_bit_sensors_();
  void set_text_sensor_value_(const std::string &text_sensor_name, const std::string &value);
  const RawFrameEntry *find_latest_frame_by_type_(uint8_t packet_type, size_t &index, bool require_ok) const;
  const RawFrameEntry *find_latest_dd_frame_(size_t &index) const;
  bool is_frame_ok_(const RawFrameEntry &entry) const;

#if defined(USE_SWITCH)
  bool current_operational_mode_(uint8_t &mode) const;
  bool silent_mode_allowed_mode_(uint8_t mode) const;
  void publish_silent_mode_from_latest_packet_();
#endif
  std::string packet_type_to_string_(uint8_t packet_type) const;
  enum class TxPacketKind : uint8_t {
    SINGLE_FIELD,
    SNAPSHOT,
    PROFILE_RESTORE,
    RESTORE_DEFAULTS,
    TIME_BAND,
    WATER_HEATER,
  };
  enum class TxOperationKind : uint8_t {
    NONE,
    SINGLE_FIELD,
    RESTORE_DEFAULTS,
    PROFILE_RESTORE,
    TIME_BAND,
    WATER_HEATER,
  };
  enum class TxPacketFamily : uint8_t {
    MAIN,
    EXTENDED,
  };
  struct TxPacketFamilySpec {
    TxPacketFamily family;
    uint8_t base_packet_type;
    uint8_t write_packet_type;
    uint8_t readback_packet_type;
    uint8_t packet_size;
    const char *label;
  };
  struct SingleFieldTxPayload {
    TxPacketFamily family = TxPacketFamily::MAIN;
    uint8_t index = 0;
    uint8_t value = 0;
    uint8_t bit_position = kBitPositionNoBitmask;
    uint8_t bit_width = 1;
  };
  struct RestoreTxPayload {
    TxPacketFamily family = TxPacketFamily::MAIN;
    bool main_applied = false;
    bool extended_applied = false;
    bool main_write_sent = false;
    bool extended_write_sent = false;
  };
  struct ProfileRestoreTxPayload {
    bool known_good = false;
    TxPacketFamily family = TxPacketFamily::MAIN;
    bool main_applied = false;
    bool extended_applied = false;
    bool main_write_sent = false;
    bool extended_write_sent = false;
  };
  struct TimeBandTxPayload {
    uint8_t flag = 0;
    uint8_t start_hour = 0;
    uint8_t start_minute = 0;
    uint8_t end_hour = 0;
    uint8_t end_minute = 0;
    uint8_t mode = 0;
  };
  struct WaterHeaterTxField {
    const char *name = "";
    uint8_t write_index = 0;
    uint8_t write_value = 0;
    uint8_t write_bit_position = kBitPositionNoBitmask;
    uint8_t write_bit_width = 1;
    uint8_t readback_index = 0;
    uint8_t readback_value = 0;
    uint8_t readback_bit_position = kBitPositionNoBitmask;
    uint8_t readback_bit_width = 1;
  };
  struct WaterHeaterTxPayload {
    TxPacketFamily family = TxPacketFamily::MAIN;
    std::vector<uint8_t> base_packet;
    std::vector<WaterHeaterTxField> fields;
  };
  const TxPacketFamilySpec &tx_packet_family_spec_(TxPacketFamily family) const;
  void send_prebuilt_cd_packet_(TxPacketFamily family, const std::vector<uint8_t> &command, TxPacketKind kind,
                                uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width,
                                uint8_t attempts_sent);
  bool validate_outbound_cd_packet_(TxPacketFamily family, const std::vector<uint8_t> &base_packet,
                                    const std::vector<uint8_t> &command, TxPacketKind kind,
                                    uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width,
                                    std::string &reason);
#if defined(USE_WATER_HEATER)
  bool prepare_water_heater_tx_payload_(WaterHeaterTxPayload &payload, std::string &reason) const;
  bool add_water_heater_tx_field_(WaterHeaterTxPayload &payload, const char *name,
                                  uint8_t write_index, uint8_t write_value,
                                  uint8_t write_bit_position, uint8_t write_bit_width);
  bool water_heater_add_power_field_(WaterHeaterTxPayload &payload, bool power_on);
  bool water_heater_add_mode_field_(WaterHeaterTxPayload &payload, uint8_t mode);
  bool water_heater_add_target_field_(WaterHeaterTxPayload &payload, uint8_t mode, uint8_t target);
  bool build_water_heater_tx_command_(const WaterHeaterTxPayload &payload,
                                      std::vector<uint8_t> &command,
                                      std::string &reason);
  bool water_heater_tx_matches_packet_(const WaterHeaterTxPayload &payload,
                                       const std::vector<uint8_t> &buffer,
                                       const WaterHeaterTxField **first_mismatch = nullptr) const;
  bool water_heater_tx_matches_base_packet_(const WaterHeaterTxPayload &payload,
                                            const std::vector<uint8_t> &buffer,
                                            const WaterHeaterTxField **first_mismatch = nullptr) const;
  bool validate_water_heater_tx_command_(const WaterHeaterTxPayload &payload,
                                         const std::vector<uint8_t> &command,
                                         std::string &reason);
#endif
  bool send_uart_command_(TxPacketFamily family, uint8_t index, uint8_t value,
                          uint8_t bit_position, uint8_t bit_width);
  void send_uart_tx_packet_(TxPacketFamily family, const std::vector<uint8_t> &base_packet, bool apply_change,
                            uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width);
  void send_uart_cc_packet_(const std::vector<uint8_t> &base_packet, bool apply_change,
                            uint8_t index, uint8_t value, uint8_t bit_position, uint8_t bit_width);
  void check_pending_tx_(const std::vector<uint8_t> &buffer);
  bool defer_single_field_tx_(TxPacketFamily family, uint8_t index, uint8_t value,
                              uint8_t bit_position, uint8_t bit_width);
  bool has_deferred_user_tx_(TxPacketFamily family, uint8_t index, uint8_t bit_position) const;
  void flush_deferred_user_tx_();
  bool schedule_tx_after_d2_(TxOperationKind kind, const RawFrameEntry &d2_entry);
  bool tx_d2_schedule_current_(TxOperationKind kind, uint32_t generation) const;
  void run_tx_after_d2_(TxOperationKind kind, uint32_t generation);
  void schedule_queued_tx_from_d2_(const RawFrameEntry &d2_entry);
  void send_restore_defaults_packet_(TxPacketFamily family, const std::vector<uint8_t> &base_packet,
                                     const std::vector<uint8_t> &packet);
  void send_profile_restore_packet_(TxPacketFamily family, const std::vector<uint8_t> &base_packet,
                                    const std::vector<uint8_t> &packet, bool known_good);
  void send_time_band_packet_(const std::vector<uint8_t> &base_packet);
#if defined(USE_WATER_HEATER)
  void send_water_heater_packet_(const std::vector<uint8_t> &base_packet);
#endif
  void request_time_band_tx_(uint8_t flag, uint8_t start_hour, uint8_t start_minute,
                             uint8_t end_hour, uint8_t end_minute, uint8_t mode);
  bool time_band_matches_packet_(const std::vector<uint8_t> &buffer, bool d2_packet,
                                 uint8_t flag, uint8_t start_hour, uint8_t start_minute,
                                 uint8_t end_hour, uint8_t end_minute, uint8_t mode) const;
  bool validate_time_band_request_(uint8_t flag, uint8_t start_hour, uint8_t start_minute,
                                   uint8_t end_hour, uint8_t end_minute, uint8_t mode,
                                   std::string &reason) const;
  void check_pending_restore_(const std::vector<uint8_t> &buffer);
  void schedule_queued_restore_from_d2_(const RawFrameEntry &d2_entry);
  void check_pending_profile_restore_(const std::vector<uint8_t> &buffer);
  void schedule_queued_profile_restore_from_d2_(const RawFrameEntry &d2_entry);
  void check_pending_time_band_(const std::vector<uint8_t> &buffer);
  void schedule_queued_time_band_from_d2_(const RawFrameEntry &d2_entry);
#if defined(USE_WATER_HEATER)
  void check_pending_water_heater_(const std::vector<uint8_t> &buffer);
  void schedule_queued_water_heater_from_d2_(const RawFrameEntry &d2_entry);
#endif
  TxOperationKind active_tx_operation_kind_() const;
  const char *tx_operation_kind_label_(TxOperationKind kind) const;
  TxPacketFamily active_tx_operation_family_() const;
  uint8_t active_tx_readback_packet_type_() const;
  bool tx_operation_active_() const;
  bool single_field_tx_active_() const;
  const SingleFieldTxPayload &single_field_tx_() const;
  bool single_field_tx_matches_(TxPacketFamily family, uint8_t index, uint8_t bit_position) const;
  bool restore_tx_active_() const;
  RestoreTxPayload &restore_tx_();
  const RestoreTxPayload &restore_tx_() const;
  bool profile_restore_tx_active_() const;
  ProfileRestoreTxPayload &profile_restore_tx_();
  const ProfileRestoreTxPayload &profile_restore_tx_() const;
  bool time_band_tx_active_() const;
  TimeBandTxPayload &time_band_tx_();
  const TimeBandTxPayload &time_band_tx_() const;
#if defined(USE_WATER_HEATER)
  bool water_heater_tx_active_() const;
  WaterHeaterTxPayload &water_heater_tx_();
  const WaterHeaterTxPayload &water_heater_tx_() const;
  bool water_heater_tx_busy_() const;
#endif
  bool single_field_tx_busy_() const;
  bool restore_tx_busy_() const;
  bool profile_restore_tx_busy_() const;
  bool time_band_tx_sending_() const;
  bool time_band_tx_busy_() const;
  bool any_write_busy_() const;
  bool any_tx_or_ui_sync_active_() const;
  bool tx_ui_sync_active_() const;
  bool tx_ui_sync_active_(TxOperationKind kind) const;
  void reset_tx_ui_sync_();
  void start_single_field_ui_sync_(const SingleFieldTxPayload &target);
  void start_restore_ui_sync_();
  void start_profile_restore_ui_sync_(bool known_good);
  void start_time_band_ui_sync_(const TimeBandTxPayload &target);
#if defined(USE_WATER_HEATER)
  void start_water_heater_ui_sync_(const WaterHeaterTxPayload &target);
#endif
  void clear_tx_wait_markers_();
  void reset_tx_operation_();
  void reset_tx_lifecycle_(TxOperationKind kind, bool clear_ui_sync);
  void reset_single_field_tx_lifecycle_(bool clear_ui_sync);
  void reset_restore_tx_lifecycle_(bool clear_ui_sync);
  void reset_profile_restore_tx_lifecycle_(bool clear_ui_sync);
  void reset_time_band_tx_lifecycle_(bool clear_ui_sync);
#if defined(USE_WATER_HEATER)
  void reset_water_heater_tx_lifecycle_(bool clear_ui_sync);
#endif
  void reset_pending_restore_();
  void reset_queued_restore_();
  void reset_pending_profile_restore_();
  void reset_queued_profile_restore_();
  void reset_pending_time_band_();
  void reset_queued_time_band_();
#if defined(USE_WATER_HEATER)
  void reset_pending_water_heater_();
  void reset_queued_water_heater_();
#endif
  uint8_t tx_readback_index_(TxPacketFamily family, uint8_t write_index, uint8_t bit_position) const;
  uint8_t tx_readback_bit_position_(TxPacketFamily family, uint8_t write_index, uint8_t bit_position) const;
  uint8_t tx_readback_bit_width_(TxPacketFamily family, uint8_t write_index,
                                 uint8_t bit_position, uint8_t bit_width) const;
  bool field_matches_target_(const std::vector<uint8_t> &buffer, uint8_t index, uint8_t value,
                             uint8_t bit_position, uint8_t bit_width) const;

  std::vector<uint8_t> last_d2_packet_;
  std::vector<uint8_t> last_dd_packet_;
  std::vector<uint8_t> last_cc_packet_;  // Always store CC for sending commands
  std::vector<uint8_t> last_c1_packet_;
  std::vector<uint8_t> last_c2_packet_;
  std::vector<uint8_t> last_d4_packet_;
  std::map<uint8_t, std::vector<uint8_t>> latest_packets_;

  bool uart_active_ = false;
  bool processing_updates_ = false;
  bool uart_tx_active_ = false; // used for SW "flow control" to avoid RS485 bus contention
  bool continuous_rx_ = false;
  unsigned long last_rx_time_ = 0;
  static constexpr uint32_t kAutoSnapshotCooldownMs = 15UL * 60UL * 1000UL;

  uint32_t cycle_start_ms_ = 0;
  uint32_t cycle_bytes_read_ = 0;
  uint32_t cycle_packets_seen_ = 0;
  uint32_t cycle_timeouts_ = 0;
  uint32_t cycle_checksum_errors_ = 0;
  uint8_t cycle_checksum_error_mask_ = 0;
  uint32_t cycle_framing_errors_ = 0;
  uint8_t cycle_framing_error_start_ = 0;
  uint8_t cycle_packet_types_seen_ = 0;
  bool cycle_timeout_logged_ = false;
  bool cycle_publish_allowed_ = true;
  bool cycle_synced_ = false;
  bool rx_frame_active_ = false;
  uint32_t rx_frame_start_ms_ = 0;
  uint8_t rx_frame_type_ = 0;
  uint8_t rx_frame_expected_len_ = 0;
  size_t rx_frame_offset_ = 0;
  uint8_t rx_frame_buffer_[kRawFrameMaxLen] = {};
  struct StoredProfileBlob {
    uint32_t magic = 0;
    uint8_t version = 0;
    uint8_t main_length = 0;
    uint8_t extended_length = 0;
    uint8_t reserved = 0;
    uint32_t saved_at_epoch = 0;
    uint32_t main_data_hash = 0;
    uint32_t extended_data_hash = 0;
    uint8_t main_data[kRawFrameMaxLen] = {};
    uint8_t extended_data[kRawFrameMaxLen] = {};
  };
  struct ProfileState {
    bool valid = false;
    uint8_t main_length = 0;
    uint8_t extended_length = 0;
    uint32_t saved_at_epoch = 0;
    uint32_t main_data_hash = 0;
    uint32_t extended_data_hash = 0;
    uint8_t main_data[kRawFrameMaxLen] = {};
    uint8_t extended_data[kRawFrameMaxLen] = {};
  };
  struct TimeBandState {
    bool initialized = false;
    bool staged_dirty = false;
    uint8_t flag = 0;
    uint8_t start_hour = 0;
    uint8_t start_minute = 0;
    uint8_t end_hour = 0;
    uint8_t end_minute = 0;
    uint8_t mode = 0;
  };
  ESPPreferenceObject known_good_profile_pref_;
  ESPPreferenceObject auto_snapshot_pref_;
  ProfileState known_good_profile_;
  ProfileState auto_snapshot_;
  TimeBandState time_band_state_;
  uint32_t auto_snapshot_last_write_ms_ = 0;

  struct TxOperationState {
    TxOperationKind kind = TxOperationKind::NONE;
    SingleFieldTxPayload single_field;
    RestoreTxPayload restore;
    ProfileRestoreTxPayload profile_restore;
    TimeBandTxPayload time_band;
#if defined(USE_WATER_HEATER)
    WaterHeaterTxPayload water_heater;
#endif
    uint8_t attempts_sent = 0;
    uint32_t last_attempt_d2_seq = 0;
  };
  TxOperationState tx_operation_;
  struct TxUiSyncState {
    TxOperationKind kind = TxOperationKind::NONE;
    SingleFieldTxPayload single_field;
    TimeBandTxPayload time_band;
#if defined(USE_WATER_HEATER)
    WaterHeaterTxPayload water_heater;
#endif
    bool known_good = false;
    bool main_synced = false;
    bool extended_synced = false;
    uint8_t cycles_waited = 0;
  };
  TxUiSyncState tx_ui_sync_;
  struct QueuedTx {
    bool active = false;
    bool scheduled = false;
    TxPacketFamily family = TxPacketFamily::MAIN;
    uint8_t index = 0;
    uint8_t value = 0;
    uint8_t bit_position = kBitPositionNoBitmask;
    uint8_t bit_width = 1;
    uint32_t generation = 0;
    uint32_t request_ms = 0;
    uint32_t anchor_ms = 0;
    uint32_t anchor_seq = 0;
  };
  QueuedTx queued_tx_;
  struct DeferredTx {
    TxPacketFamily family = TxPacketFamily::MAIN;
    uint8_t index = 0;
    uint8_t value = 0;
    uint8_t bit_position = kBitPositionNoBitmask;
    uint8_t bit_width = 1;
  };
  std::vector<DeferredTx> deferred_user_txs_;
  struct QueuedRestore {
    bool active = false;
    bool scheduled = false;
    TxPacketFamily family = TxPacketFamily::MAIN;
    uint32_t generation = 0;
    uint32_t request_ms = 0;
    uint32_t anchor_ms = 0;
    uint32_t anchor_seq = 0;
  };
  QueuedRestore queued_restore_;
  struct QueuedProfileRestore {
    bool active = false;
    bool scheduled = false;
    bool known_good = false;
    TxPacketFamily family = TxPacketFamily::MAIN;
    uint32_t generation = 0;
    uint32_t request_ms = 0;
    uint32_t anchor_ms = 0;
    uint32_t anchor_seq = 0;
  };
  QueuedProfileRestore queued_profile_restore_;
  struct QueuedTimeBandTx {
    bool active = false;
    bool scheduled = false;
    uint32_t generation = 0;
    uint32_t request_ms = 0;
    uint32_t anchor_ms = 0;
    uint32_t anchor_seq = 0;
  };
  QueuedTimeBandTx queued_time_band_tx_;
#if defined(USE_WATER_HEATER)
  struct QueuedWaterHeaterTx {
    bool active = false;
    bool scheduled = false;
    uint32_t generation = 0;
    uint32_t request_ms = 0;
    uint32_t anchor_ms = 0;
    uint32_t anchor_seq = 0;
  };
  QueuedWaterHeaterTx queued_water_heater_tx_;
#endif
  uint32_t tx_request_ms_ = 0;
  uint32_t tx_sent_ms_ = 0;
  bool tx_waiting_for_first_rx_ = false;
  bool tx_waiting_for_first_cc_ = false;
  static constexpr uint8_t kTxUiSyncMaxCycles = 3;
  static constexpr uint8_t kRestoreUiSyncMaxCycles = 3;
  static constexpr uint8_t kProfileUiSyncMaxCycles = 3;

  enum RawFrameFlags : uint8_t {
    RAW_FRAME_CRC_ERROR = 1 << 0,
    RAW_FRAME_TIMEOUT = 1 << 1,
    RAW_FRAME_TRUNCATED = 1 << 2,
    RAW_FRAME_UNKNOWN_TYPE = 1 << 3,
  };

  struct RawFrameEntry {
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t packet_type;
    uint8_t length;
    uint8_t flags;
    uint8_t data[kRawFrameMaxLen];
  };

  RawFrameEntry raw_frames_[kRawFrameBufferSize];
  size_t raw_frame_head_ = 0;
  size_t raw_frame_count_ = 0;
  uint32_t raw_frame_seq_ = 0;
  uint32_t last_frame_profile_ms_ = 0;
  uint8_t last_frame_profile_type_ = 0;
  uint32_t last_cc_profile_ms_ = 0;

  std::map<std::string, float> last_published_sensor_values_;
  std::map<std::string, uint32_t> last_published_sensor_ms_;
  std::map<std::string, float> last_published_number_values_;
  std::map<std::string, uint32_t> last_published_number_ms_;
  std::map<std::string, bool> last_published_binary_values_;
  std::map<std::string, uint32_t> last_published_binary_ms_;
  std::map<std::string, std::string> last_published_select_values_;
  std::map<std::string, uint32_t> last_published_select_ms_;
#if defined(USE_SWITCH)
  std::map<std::string, bool> last_published_switch_values_;
  std::map<std::string, uint32_t> last_published_switch_ms_;
#endif
  std::map<std::string, std::string> last_published_text_values_;
  std::map<std::string, uint32_t> last_published_text_ms_;
  std::string last_published_timestamp_;
  uint32_t last_published_timestamp_ms_ = 0;
  bool last_dd_demand_ = false;
  bool last_hp_active_ = false;
  bool last_eh_active_ = false;
  bool have_last_dd_bits_ = false;

  // Cycle management
  unsigned long last_process_time_ = 0;
  unsigned long update_interval_ = 10000;
  uint32_t tx_delay_after_d2_ms_ = kDefaultTxDelayAfterD2Ms;
};

}  // namespace daikin_ekkhe
}  // namespace esphome
