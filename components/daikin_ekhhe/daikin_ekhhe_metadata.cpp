#include "daikin_ekhhe_metadata.h"

namespace esphome {
namespace daikin_ekkhe {

using namespace daikin_ekhhe;

bool field_is_extended(WritablePacketFamily family) {
  return family == WritablePacketFamily::EXTENDED;
}

float decode_number_field_value(uint8_t value, const NumberFieldSpec &field) {
  if (field.encoding == NumberFieldEncoding::INT8) {
    return static_cast<int8_t>(value);
  }
  return value;
}

float decode_number_field(const std::vector<uint8_t> &buffer, const NumberFieldSpec &field) {
  return decode_number_field_value(buffer[field.index], field);
}

uint8_t encode_number_field(float value, const NumberFieldSpec &field) {
  if (field.encoding == NumberFieldEncoding::INT8) {
    return static_cast<uint8_t>(static_cast<int8_t>(value));
  }
  return static_cast<uint8_t>(value);
}


// Packet definitions
const std::map<uint8_t, uint8_t> PACKET_SIZES = {
    {DD_PACKET_START_BYTE, DaikinEkhheComponent::DD_PACKET_SIZE},
    {D2_PACKET_START_BYTE, DaikinEkhheComponent::D2_PACKET_SIZE},
    {D4_PACKET_START_BYTE, DaikinEkhheComponent::D4_PACKET_SIZE},
    {C1_PACKET_START_BYTE, DaikinEkhheComponent::C1_PACKET_SIZE},
    {C2_PACKET_START_BYTE, DaikinEkhheComponent::C2_PACKET_SIZE},
    {CC_PACKET_START_BYTE, DaikinEkhheComponent::CC_PACKET_SIZE},
    {CD_PACKET_START_BYTE, DaikinEkhheComponent::CD_PACKET_SIZE},
};

const std::map<std::string, NumberFieldSpec> NUMBER_FIELD_SPECS = {
  {P1_LOW_WAT_PROBE_HYST,   {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P1_IDX, NumberFieldEncoding::UINT8}},
  {P2_HEAT_ON_DELAY,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P2_IDX, NumberFieldEncoding::UINT8}},
  {P3_ANTL_SET_T,           {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P3_IDX, NumberFieldEncoding::UINT8}},
  {P4_ANTL_DURATION,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P4_IDX, NumberFieldEncoding::UINT8}},
  {P7_DEFROST_CYCLE_DELAY,  {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P7_IDX, NumberFieldEncoding::UINT8}},
  {P8_DEFR_START_THRES,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P8_IDX, NumberFieldEncoding::INT8}},
  {P9_DEFR_STOP_THRES,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P9_IDX, NumberFieldEncoding::UINT8}},
  {P10_DEFR_MAX_DURATION,   {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P10_IDX, NumberFieldEncoding::UINT8}},
  {P17_HP_START_DELAY_DIG1, {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P17_IDX, NumberFieldEncoding::UINT8}},
  {P18_LOW_WAT_T_DIG1,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P18_IDX, NumberFieldEncoding::UINT8}},
  {P19_LOW_WAT_T_HYST,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P19_IDX, NumberFieldEncoding::UINT8}},
  {P20_SOL_DRAIN_THRES,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P20_IDX, NumberFieldEncoding::UINT8}},
  {P21_LOW_WAT_T_HP_STOP,   {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P21_IDX, NumberFieldEncoding::UINT8}},
  {P22_UP_WAT_T_EH_STOP,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P22_IDX, NumberFieldEncoding::UINT8}},
  {P25_UP_WAT_T_OFFSET,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P25_IDX, NumberFieldEncoding::INT8}},
  {P26_LOW_WAT_T_OFFSET,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P26_IDX, NumberFieldEncoding::INT8}},
  {P27_INLET_T_OFFSET,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P27_IDX, NumberFieldEncoding::INT8}},
  {P28_DEFR_T_OFFSET,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P28_IDX, NumberFieldEncoding::INT8}},
  {P29_ANTL_START_HR,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P29_IDX, NumberFieldEncoding::UINT8}},
  {P30_UP_WAT_T_EH_HYST,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P30_IDX, NumberFieldEncoding::UINT8}},
  {P31_HP_PERIOD_AUTO,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P31_IDX, NumberFieldEncoding::UINT8}},
  {P32_EH_AUTO_TRES,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P32_IDX, NumberFieldEncoding::UINT8}},
  {P34_EEV_SH_PERIOD,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P34_IDX, NumberFieldEncoding::UINT8}},
  {P35_EEV_SH_SETPOINT,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P35_IDX, NumberFieldEncoding::INT8}},
  {P36_EEV_DSH_SETPOINT,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P36_IDX, NumberFieldEncoding::UINT8}},
  {P37_EEV_STEP_DEFR,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P37_IDX, NumberFieldEncoding::UINT8}},
  {P38_EEV_MIN_STEP_AUTO,   {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P38_IDX, NumberFieldEncoding::UINT8}},
  {P40_EEV_INIT_STEP,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P40_IDX, NumberFieldEncoding::UINT8}},
  {P41_AKP1_THRES,          {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P41_IDX, NumberFieldEncoding::INT8}},
  {P42_AKP2_THRES,          {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P42_IDX, NumberFieldEncoding::INT8}},
  {P43_AKP3_THRES,          {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P43_IDX, NumberFieldEncoding::INT8}},
  {P44_EEV_KP1_GAIN,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P44_IDX, NumberFieldEncoding::INT8}},
  {P45_EEV_KP2_GAIN,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P45_IDX, NumberFieldEncoding::INT8}},
  {P46_EEV_KP3_GAIN,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P46_IDX, NumberFieldEncoding::INT8}},
  {P47_MAX_INLET_T_HP,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P47_IDX, NumberFieldEncoding::UINT8}},
  {P48_MIN_INLET_T_HP,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P48_IDX, NumberFieldEncoding::INT8}},
  {P49_EVA_INLET_THRES,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P49_IDX, NumberFieldEncoding::UINT8}},
  {P50_ANTIFREEZE_SET,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P50_IDX, NumberFieldEncoding::UINT8}},
  {P51_EVA_HIGH_SET,        {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P51_IDX, NumberFieldEncoding::UINT8}},
  {P52_EVA_LOW_SET,         {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P52_IDX, NumberFieldEncoding::UINT8}},
  {P54_LOW_PRESS_BYPASS,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P54_IDX, NumberFieldEncoding::UINT8}},
  {ECO_T_TEMPERATURE,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_ECO_TTARGET_IDX, NumberFieldEncoding::UINT8}},
  {AUTO_T_TEMPERATURE,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_AUTO_TTARGET_IDX, NumberFieldEncoding::UINT8}},
  {BOOST_T_TEMPERATURE,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_BOOST_TTGARGET_IDX, NumberFieldEncoding::UINT8}},
  {ELECTRIC_T_TEMPERATURE,  {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_ELECTRIC_TTARGET_IDX, NumberFieldEncoding::UINT8}},
  {VAC_DAYS,                {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_VAC_DAYS, NumberFieldEncoding::UINT8}},

  {P53_EVA_FAN_DEFR_SPEED,  {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P53_IDX, NumberFieldEncoding::UINT8}},
  {P55_EVA_BAND1_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P55_IDX, NumberFieldEncoding::UINT8}},
  {P56_EVA_MAX_ACT_DELTA,   {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P56_IDX, NumberFieldEncoding::UINT8}},
  {P57_EVA_MAX_DEACT_DELTA, {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P57_IDX, NumberFieldEncoding::UINT8}},
  {P59_EVA_FAN_OFF_SPEED,   {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P59_IDX, NumberFieldEncoding::UINT8}},
  {P60_EVA_AIR_DELTA1,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P60_IDX, NumberFieldEncoding::UINT8}},
  {P61_EVA_AIR_DELTA2,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P61_IDX, NumberFieldEncoding::UINT8}},
  {P62_EVA_AIR_DELTA3,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P62_IDX, NumberFieldEncoding::UINT8}},
  {P63_EVA_AIR_DELTA4,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P63_IDX, NumberFieldEncoding::UINT8}},
  {P64_EVA_AIR_DELTA5,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P64_IDX, NumberFieldEncoding::UINT8}},
  {P65_EVA_AIR_DELTA6,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P65_IDX, NumberFieldEncoding::UINT8}},
  {P66_EVA_BAND2_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P66_IDX, NumberFieldEncoding::UINT8}},
  {P67_EVA_BAND3_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P67_IDX, NumberFieldEncoding::UINT8}},
  {P68_EVA_BAND4_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P68_IDX, NumberFieldEncoding::UINT8}},
  {P69_EVA_BAND5_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P69_IDX, NumberFieldEncoding::UINT8}},
  {P70_EVA_BAND6_PROP,      {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P70_IDX, NumberFieldEncoding::UINT8}},
  {P71_EC_FAN_SILENT_RED,   {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P71_IDX, NumberFieldEncoding::UINT8}},
  {P72_EC_FAN_REG_GAIN,     {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P72_IDX, NumberFieldEncoding::UINT8}},
};

const std::map<std::string, SelectFieldSpec> SELECT_FIELD_SPECS = {
  {OPERATIONAL_MODE,     {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MODE_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P12_EXT_PUMP_MODE,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P12_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P14_EVA_BLOWER_TYPE,  {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P14_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P16_SOLAR_MODE_INT,   {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P16_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P23_PV_MODE_INT,      {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P23_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P24_OFF_PEAK_MODE,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_P24_IDX, BIT_POSITION_NO_BITMASK, 1}},
  {P58_EVA_FAN_COMP_OFF, {WritablePacketFamily::EXTENDED, DaikinEkhheComponent::EXT_PACKET_P58_IDX, BIT_POSITION_NO_BITMASK, 1}},

  {POWER_STATUS,          {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 0, 1}},
  {P39_EEV_MODE,          {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 2, 1}},
  {P13_HW_CIRC_PUMP_MODE, {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK1_IDX, 4, 1}},
  {P11_DISP_WAT_T_PROBE,  {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 0, 1}},
  {P15_SAFETY_SW_TYPE,    {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 1, BIT_WIDTH_P15_STEPPED}},
  {P5_DEFROST_MODE,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 2, 1}},
  {P6_EHEATER_DEFROSTING, {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 3, 1}},
  {P33_EEV_CONTROL,       {WritablePacketFamily::MAIN, DaikinEkhheComponent::CC_PACKET_MASK2_IDX, 4, 1}},
};

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

extern const size_t RESTORE_DEFAULT_MAIN_FIELD_COUNT =
    sizeof(RESTORE_DEFAULT_FIELDS) / sizeof(RESTORE_DEFAULT_FIELDS[0]);
extern const size_t RESTORE_DEFAULT_EXTENDED_FIELD_COUNT =
    sizeof(RESTORE_DEFAULT_EXTENDED_FIELDS) / sizeof(RESTORE_DEFAULT_EXTENDED_FIELDS[0]);
extern const size_t RESTORE_DEFAULT_FIELD_COUNT =
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
  const PacketFieldRef &location = use_d2_indices ? field.readback : field.write;
  if (location.index >= buffer.size()) {
    return false;
  }
  if (location.bit_position == BIT_POSITION_NO_BITMASK) {
    return buffer[location.index] == field.value;
  }
  return extract_field_value(buffer, location) ==
         (field.value & field_value_mask(location.index, location.bit_position, location.bit_width));
}

void apply_restore_defaults_to_packet(std::vector<uint8_t> &packet, bool extended) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (field.write.index >= packet.size()) {
      continue;
    }
    apply_field_value(packet, field.write, field.value);
  }
}

bool restore_defaults_match_packet(const std::vector<uint8_t> &buffer, bool use_d2_indices,
                                   bool extended) {
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
                                               uint8_t &current_value, bool extended) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (restore_scope_field_matches(field, buffer, use_d2_indices)) {
      continue;
    }
    const PacketFieldRef &location = use_d2_indices ? field.readback : field.write;
    if (location.index >= buffer.size()) {
      current_value = 0;
    } else if (location.bit_position == BIT_POSITION_NO_BITMASK) {
      current_value = buffer[location.index];
    } else {
      current_value = extract_field_value(buffer, location);
    }
    return &field;
  }
  current_value = 0;
  return nullptr;
}

bool is_restore_scope_field(uint8_t cc_index, uint8_t bit_position, bool extended) {
  size_t count = 0;
  const RestoreFieldSpec *fields = restore_default_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const RestoreFieldSpec &field = fields[i];
    if (field.write.index == cc_index && field.write.bit_position == bit_position) {
      return true;
    }
  }
  return false;
}

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

extern const size_t PROFILE_MANAGED_MAIN_FIELD_COUNT =
    sizeof(PROFILE_MANAGED_FIELDS) / sizeof(PROFILE_MANAGED_FIELDS[0]);
extern const size_t PROFILE_MANAGED_EXTENDED_FIELD_COUNT =
    sizeof(PROFILE_MANAGED_EXTENDED_FIELDS) / sizeof(PROFILE_MANAGED_EXTENDED_FIELDS[0]);
extern const size_t PROFILE_MANAGED_FIELD_COUNT =
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
    if (field.write.index == cc_index && field.write.bit_position == bit_position) {
      return true;
    }
  }
  return false;
}

const ManagedFieldSpec *find_managed_field_by_cc(uint8_t cc_index, uint8_t bit_position) {
  for (const auto &field : PROFILE_MANAGED_FIELDS) {
    if (field.write.index != cc_index) {
      continue;
    }
    if (bit_position == BIT_POSITION_NO_BITMASK && field.write.bit_position == BIT_POSITION_NO_BITMASK) {
      return &field;
    }
    if (bit_position != BIT_POSITION_NO_BITMASK && field.write.bit_position == bit_position) {
      return &field;
    }
  }
  return nullptr;
}


static bool is_p15_stepped_field(uint8_t index, uint8_t bit_position, uint8_t bit_width) {
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

uint8_t extract_field_value(const uint8_t *data, size_t size, const PacketFieldRef &field) {
  return extract_field_value(data, size, field.index, field.bit_position, field.bit_width);
}

uint8_t extract_field_value(const std::vector<uint8_t> &data, const PacketFieldRef &field) {
  return extract_field_value(data.data(), data.size(), field);
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

void apply_field_value(std::vector<uint8_t> &packet, const PacketFieldRef &field, uint8_t value) {
  apply_field_value(packet, field.index, field.bit_position, field.bit_width, value);
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
    const PacketFieldRef &actual_location = use_d2_indices ? field.readback : field.write;
    const uint8_t expected = extract_field_value(profile_data, profile_len, field.write);
    const uint8_t actual = extract_field_value(buffer, actual_location);
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
    const PacketFieldRef &current_location = use_d2_indices ? field.readback : field.write;
    expected_value = extract_field_value(profile_data, profile_len, field.write);
    current_value = extract_field_value(buffer, current_location);
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
    const uint8_t value = extract_field_value(profile_data, profile_len, field.write);
    apply_field_value(packet, field.write, value);
  }
}

bool managed_fields_equal_between_packets(bool extended, const uint8_t *lhs, size_t lhs_len,
                                          const uint8_t *rhs, size_t rhs_len) {
  size_t count = 0;
  const ManagedFieldSpec *fields = profile_managed_fields(extended, count);
  for (size_t i = 0; i < count; ++i) {
    const ManagedFieldSpec &field = fields[i];
    if (extract_field_value(lhs, lhs_len, field.write) !=
        extract_field_value(rhs, rhs_len, field.write)) {
      return false;
    }
  }
  return true;
}

}  // namespace daikin_ekkhe
}  // namespace esphome
