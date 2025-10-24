import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_DEVICE_CLASS,
)
from esphome.const import (
    CONF_ID,
    ICON_EMPTY,
    UNIT_EMPTY,
    UNIT_CELSIUS,
    UNIT_HOUR,
    UNIT_SECOND,
    UNIT_PERCENT,
    UNIT_MINUTE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TIMESTAMP,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_NONE
)


from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
    daikin_ekhhe_ns,
)

from .const import *

DaikinEkhheNumber = daikin_ekhhe_ns.class_(
    "DaikinEkhheNumber", number.Number, cg.Component
)


TYPES = [
    P1_LOW_WAT_PROBE_HYST,
    P2_HEAT_ON_DELAY,
    P3_ANTL_SET_T,
    P4_ANTL_DURATION,
    P7_DEFROST_CYCLE_DELAY,
    P8_DEFR_START_THRES,
    P9_DEFR_STOP_THRES,
    P10_DEFR_MAX_DURATION,
    P17_HP_START_DELAY_DIG1,
    P18_LOW_WAT_T_DIG1,
    P19_LOW_WAT_T_HYST,
    P20_SOL_DRAIN_THRES,
    P21_LOW_WAT_T_HP_STOP,
    P22_UP_WAT_T_EH_STOP,
    P25_UP_WAT_T_OFFSET,
    P26_LOW_WAT_T_OFFSET,
    P27_INLET_T_OFFSET,
    P28_DEFR_T_OFFSET,
    P29_ANTL_START_HR,
    P30_UP_WAT_T_EH_HYST,
    P31_HP_PERIOD_AUTO,
    P32_EH_AUTO_TRES,
    P34_EEV_SH_PERIOD,
    P35_EEV_SH_SETPOINT,
    P36_EEV_DSH_SETPOINT,
    P37_EEV_STEP_DEFR,
    P38_EEV_MIN_STEP_AUTO,
    P40_EEV_INIT_STEP,
    P41_AKP1_THRES,
    P42_AKP2_THRES,
    P43_AKP3_THRES,
    P44_EEV_KP1_GAIN,
    P45_EEV_KP2_GAIN,
    P46_EEV_KP3_GAIN,
    P47_MAX_INLET_T_HP,
    P48_MIN_INLET_T_HP,
    P49_EVA_INLET_THRES,
    P50_ANTIFREEZE_SET,
    P51_EVA_HIGH_SET,
    P52_EVA_LOW_SET,
    AUTO_T_TEMPERATURE,
    ECO_T_TEMPERATURE,
    BOOST_T_TEMPERATURE,
    ELECTRIC_T_TEMPERATURE,
]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
            cv.Optional(P1_LOW_WAT_PROBE_HYST): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=15): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=2): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }),
            cv.Optional(P2_HEAT_ON_DELAY): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=90): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }),
            cv.Optional(P3_ANTL_SET_T): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=75): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=50): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P4_ANTL_DURATION): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=90): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }), 
            cv.Optional(P7_DEFROST_CYCLE_DELAY): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=90): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }), 
            cv.Optional(P8_DEFR_START_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=0): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P9_DEFR_STOP_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber), 
                cv.Optional(CONF_MAX_VALUE, default=30): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=2): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P10_DEFR_MAX_DURATION): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=12): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=3): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }),
            cv.Optional(P17_HP_START_DELAY_DIG1): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=60): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }),
            cv.Optional(P18_LOW_WAT_T_DIG1): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=60): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=20): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P19_LOW_WAT_T_HYST): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=20): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=5): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P20_SOL_DRAIN_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=150): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=100): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P21_LOW_WAT_T_HP_STOP): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=70): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P22_UP_WAT_T_EH_STOP): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=80): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=30): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P25_UP_WAT_T_OFFSET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-25): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P26_LOW_WAT_T_OFFSET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-25): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P27_INLET_T_OFFSET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-25): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P28_DEFR_T_OFFSET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-25): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P29_ANTL_START_HR): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=23): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TIMESTAMP): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_HOUR): cv.string,
            }),
            cv.Optional(P30_UP_WAT_T_EH_HYST): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=20): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=2): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P31_HP_PERIOD_AUTO): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=80): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_MINUTE): cv.string,
            }),
            cv.Optional(P32_EH_AUTO_TRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=20): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=2): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P34_EEV_SH_PERIOD): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=90): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=20): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_DURATION): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_SECOND): cv.string,
            }),
            cv.Optional(P35_EEV_SH_SETPOINT): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=15): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-8): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P36_EEV_DSH_SETPOINT): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=110): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=60): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P37_EEV_STEP_DEFR): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=50): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=5): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P38_EEV_MIN_STEP_AUTO): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=45): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=3): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P40_EEV_INIT_STEP): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=50): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=5): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P41_AKP1_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P42_AKP2_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P43_AKP3_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P44_EEV_KP1_GAIN): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P45_EEV_KP2_GAIN): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P46_EEV_KP3_GAIN): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
            }), 
            cv.Optional(P47_MAX_INLET_T_HP): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=43): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=38): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P48_MIN_INLET_T_HP): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=10): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=-10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P49_EVA_INLET_THRES): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=40): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P50_ANTIFREEZE_SET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=15): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }), 
            cv.Optional(P51_EVA_HIGH_SET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=100): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=60): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_PERCENT): cv.string,
            }), 
            cv.Optional(P52_EVA_LOW_SET): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=60): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=10): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_PERCENT): cv.string,
            }), 
            cv.Optional(ECO_T_TEMPERATURE): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=62): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=43): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }),
            cv.Optional(AUTO_T_TEMPERATURE): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=62): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=43): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }),
            cv.Optional(BOOST_T_TEMPERATURE): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=75): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=43): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }),
            cv.Optional(ELECTRIC_T_TEMPERATURE): number.number_schema(DaikinEkhheNumber).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheNumber),
                cv.Optional(CONF_MAX_VALUE, default=75): cv.float_,
                cv.Optional(CONF_MIN_VALUE, default=43): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.positive_float,
                cv.Optional(CONF_DEVICE_CLASS, default=DEVICE_CLASS_TEMPERATURE): cv.string,
                cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=UNIT_CELSIUS): cv.string,
            }),
        }

    )
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]

        num = await number.new_number(
            conf,
            min_value=conf[CONF_MIN_VALUE],
            max_value=conf[CONF_MAX_VALUE], 
            step=conf[CONF_STEP])
        cg.add(getattr(hub, "register_number")(key, num))
        cg.add(num.set_parent(hub))
        cg.add(num.set_internal_id(key))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)