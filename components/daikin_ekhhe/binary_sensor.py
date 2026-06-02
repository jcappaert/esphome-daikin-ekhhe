import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
)

from .const import *


TYPES =[
    DIG1_CONFIG,
    DIG2_CONFIG,
    DIG3_CONFIG,
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

RUNTIME_DD_TYPES = {
    DD_HEATING_DEMAND: "set_dd_heating_demand",
    HP_ACTIVE: "set_hp_active",
    EH_ACTIVE: "set_eh_active",
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
            cv.Optional(DIG1_CONFIG): binary_sensor.binary_sensor_schema(
                #device_class=DEVICE_CLASS_NONE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(DIG2_CONFIG): binary_sensor.binary_sensor_schema(
                #device_class=DEVICE_CLASS_NONE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(DIG3_CONFIG): binary_sensor.binary_sensor_schema(
                #device_class=DEVICE_CLASS_NONE,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,  		
            ),
            cv.Optional(P01_TANK_LOWER_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P02_TANK_UPPER_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P03_DEFROST_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P04_INLET_AIR_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P05_EVAPORATOR_INLET_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P06_EVAPORATOR_OUTLET_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P07_COMPRESSOR_FLOW_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(P08_SOLAR_COLLECTOR_PROBE_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(E01_HIGH_PRESSURE_PROTECTION): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(E02_SOLAR_RECIRCULATION_ALARM): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(E03_ELECTRONIC_FAN_FAULT): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(PA_HEAT_PUMP_TEMP_UNSUITABLE_ALARM): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(DD_HEATING_DEMAND): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(HP_ACTIVE): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(EH_ACTIVE): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
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
