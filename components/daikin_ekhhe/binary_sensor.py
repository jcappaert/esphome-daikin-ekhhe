import esphome.codegen as cg
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_PROBLEM, ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
)

from .const import *
from .schema_helpers import (
    BinarySensorSchemaSpec,
    binary_sensor_schema,
    optional_schema_entries,
    platform_schema,
)

BINARY_SENSOR_SPECS = [
    BinarySensorSchemaSpec(DIG1_CONFIG, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    BinarySensorSchemaSpec(DIG2_CONFIG, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    BinarySensorSchemaSpec(DIG3_CONFIG, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    BinarySensorSchemaSpec(
        MASTER_FAULT,
        device_class=DEVICE_CLASS_PROBLEM,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    *[
        BinarySensorSchemaSpec(key, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
        for key in [
            P01_TANK_LOWER_PROBE_FAULT,
            P02_TANK_UPPER_PROBE_FAULT,
            P03_DEFROST_PROBE_FAULT,
            P04_INLET_AIR_PROBE_FAULT,
            P05_EVAPORATOR_INLET_PROBE_FAULT,
            P06_EVAPORATOR_OUTLET_PROBE_FAULT,
            P07_COMPRESSOR_FLOW_PROBE_FAULT,
            P08_SOLAR_COLLECTOR_PROBE_FAULT,
            E01_HIGH_PRESSURE_PROTECTION,
            E02_SOLAR_RECIRCULATION_ALARM,
            E03_ELECTRONIC_FAN_FAULT,
            PA_HEAT_PUMP_TEMP_UNSUITABLE_ALARM,
        ]
    ],
]

TYPES = [spec.key for spec in BINARY_SENSOR_SPECS]

RUNTIME_DD_TYPES = {
    HEATING_DEMAND: "set_heating_demand",
    HP_ACTIVE: "set_hp_active",
    EH_ACTIVE: "set_eh_active",
}

RUNTIME_DD_SPECS = [
    BinarySensorSchemaSpec(key, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
    for key in RUNTIME_DD_TYPES
]

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    {
        **optional_schema_entries(BINARY_SENSOR_SPECS, binary_sensor_schema),
        **optional_schema_entries(RUNTIME_DD_SPECS, binary_sensor_schema),
    },
)

async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]

        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(hub.register_binary_sensor(key, sens))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
    for key, setter_name in RUNTIME_DD_TYPES.items():
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(hub.register_binary_sensor(key, sens))
            cg.add(getattr(hub, setter_name)(sens))
