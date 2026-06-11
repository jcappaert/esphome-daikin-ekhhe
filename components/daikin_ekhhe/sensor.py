import esphome.codegen as cg
from esphome.components import sensor
from esphome.const import (
    UNIT_EMPTY,
    UNIT_CELSIUS,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_EKHHE_ID, DaikinEkhhe

from .const import *
from .schema_helpers import (
    SensorSchemaSpec,
    optional_schema_entries,
    platform_schema,
    sensor_schema,
)

TEMPERATURE_SENSOR_KEYS = [
    A_LOW_WAT_T_PROBE,
    B_UP_WAT_T_PROBE,
    C_DEFROST_T_PROBE,
    D_SUPPLY_AIR_T_PROBE,
    E_EVA_INLET_T_PROBE,
    F_EVA_OUTLET_T_PROBE,
    G_COMP_GAS_T_PROBE,
    H_SOLAR_T_PROBE,
]

SENSOR_SPECS = [
    *[
        SensorSchemaSpec(
            key,
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        )
        for key in TEMPERATURE_SENSOR_KEYS
    ],
    SensorSchemaSpec(
        I_EEV_STEP,
        unit_of_measurement=UNIT_EMPTY,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    SensorSchemaSpec(
        FAN_SPEED_RPM,
        unit_of_measurement="rpm",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
]

TYPES = [spec.key for spec in SENSOR_SPECS]

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    optional_schema_entries(SENSOR_SPECS, sensor_schema),
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]

        sens = await sensor.new_sensor(conf)
        cg.add(hub.register_sensor(key, sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
