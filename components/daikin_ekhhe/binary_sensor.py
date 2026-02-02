import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
    DEBUG_COMPONENTS,
)

from .const import *


TYPES =[
    DIG1_CONFIG,
    DIG2_CONFIG,
    DIG3_CONFIG
]

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
            cv.Optional(DD_HEATING_DEMAND): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(DD_HEATING_STAGE1): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(DD_HEATING_STAGE2): binary_sensor.binary_sensor_schema(
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
    if str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        if DD_HEATING_DEMAND in config:
            sens = await binary_sensor.new_binary_sensor(config[DD_HEATING_DEMAND])
            cg.add(hub.register_binary_sensor(DD_HEATING_DEMAND, sens))
            cg.add(hub.set_dd_heating_demand(sens))
        if DD_HEATING_STAGE1 in config:
            sens = await binary_sensor.new_binary_sensor(config[DD_HEATING_STAGE1])
            cg.add(hub.register_binary_sensor(DD_HEATING_STAGE1, sens))
            cg.add(hub.set_dd_heating_stage1(sens))
        if DD_HEATING_STAGE2 in config:
            sens = await binary_sensor.new_binary_sensor(config[DD_HEATING_STAGE2])
            cg.add(hub.register_binary_sensor(DD_HEATING_STAGE2, sens))
            cg.add(hub.set_dd_heating_stage2(sens))
