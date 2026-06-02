import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    UNIT_EMPTY,
    UNIT_CELSIUS,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe
)

from .const import *

TYPES = [
    # sensor parameters
    A_LOW_WAT_T_PROBE,
    B_UP_WAT_T_PROBE,
    C_DEFROST_T_PROBE,
    D_SUPPLY_AIR_T_PROBE,
    E_EVA_INLET_T_PROBE,
    F_EVA_OUTLET_T_PROBE,
    G_COMP_GAS_T_PROBE,
    H_SOLAR_T_PROBE,
    I_EEV_STEP,
    FAN_SPEED_RPM,
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
            cv.Optional(FAN_SPEED_RPM): sensor.sensor_schema(
                unit_of_measurement="rpm",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
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
