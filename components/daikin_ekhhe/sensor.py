import esphome.codegen as cg
from esphome.core import EnumValue
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    ICON_EMPTY,
    UNIT_EMPTY,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_MINUTE,
    UNIT_MILLISECOND,
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
    DaikinEkhhe
)

from .const import *

TYPES = [
    # sensor parametersr
    A_LOW_WAT_T_PROBE,
    B_UP_WAT_T_PROBE,
    C_DEFROST_T_PROBE,
    D_SUPPLY_AIR_T_PROBE,
    E_EVA_INLET_T_PROBE,
    F_EVA_OUTLET_T_PROBE,
    G_COMP_GAS_T_PROBE,
    H_SOLAR_T_PROBE,
    I_EEV_STEP,
    J_POWER_FW_VERSION,
    L_UI_FW_VERSION,
]

DEBUG_TYPES = [
    FRAMES_CAPTURED_TOTAL,
    FRAMES_DROPPED_TOTAL,
    FRAMES_TRUNCATED_TOTAL,
    CRC_ERRORS_TOTAL,
    FRAMING_ERRORS_TOTAL,
    BYTES_CAPTURED_TOTAL,
    CYCLE_PARSE_TIME_MS,
    CYCLE_TOTAL_TIME_MS,
    CYCLE_OVER_BUDGET_TOTAL,
]

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
            cv.Optional(A_LOW_WAT_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(B_UP_WAT_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(C_DEFROST_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
                cv.Optional(D_SUPPLY_AIR_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(E_EVA_INLET_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(F_EVA_OUTLET_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(G_COMP_GAS_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(H_SOLAR_T_PROBE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(I_EEV_STEP): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                #device_class=DEVICE_CLASS_NONE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(J_POWER_FW_VERSION): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                #device_class=DEVICE_CLASS_NONE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(L_UI_FW_VERSION): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                #device_class=DEVICE_CLASS_NONE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(FRAMES_CAPTURED_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(FRAMES_DROPPED_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(FRAMES_TRUNCATED_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CRC_ERRORS_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(FRAMING_ERRORS_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(BYTES_CAPTURED_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CYCLE_PARSE_TIME_MS): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLISECOND,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CYCLE_TOTAL_TIME_MS): sensor.sensor_schema(
                unit_of_measurement=UNIT_MILLISECOND,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DURATION,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CYCLE_OVER_BUDGET_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_EMPTY,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]

        sens = await sensor.new_sensor(conf)
        cg.add(hub.register_sensor(key, sens))
        #cg.add(getattr(hub, f"register_sensor")(cg.RawExpression("EKHHE::" + key), sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
    for key in DEBUG_TYPES:
        if key in config:
            conf = config[key]
            sens = await sensor.new_sensor(conf)
            cg.add(hub.register_debug_sensor(key, sens))
