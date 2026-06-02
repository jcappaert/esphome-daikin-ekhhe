import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import DEVICE_CLASS_TIMESTAMP, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe
from .const import (
    CURRENT_TIME,
    DAIKIN_AUTO_SNAPSHOT_STATUS,
    DAIKIN_KNOWN_GOOD_PROFILE_STATUS,
    J_POWER_FW_VERSION,
    L_UI_FW_VERSION,
)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(CURRENT_TIME): text_sensor.text_sensor_schema(
            device_class=DEVICE_CLASS_TIMESTAMP,
        ),
        cv.Optional(J_POWER_FW_VERSION): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(L_UI_FW_VERSION): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_KNOWN_GOOD_PROFILE_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_AUTO_SNAPSHOT_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])

    if CURRENT_TIME in config:
        sens = await text_sensor.new_text_sensor(config[CURRENT_TIME])
        cg.add(hub.register_timestamp_sensor(sens))
    if J_POWER_FW_VERSION in config:
        sens = await text_sensor.new_text_sensor(config[J_POWER_FW_VERSION])
        cg.add(hub.register_text_sensor(J_POWER_FW_VERSION, sens))
    if L_UI_FW_VERSION in config:
        sens = await text_sensor.new_text_sensor(config[L_UI_FW_VERSION])
        cg.add(hub.register_text_sensor(L_UI_FW_VERSION, sens))
    if DAIKIN_KNOWN_GOOD_PROFILE_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_KNOWN_GOOD_PROFILE_STATUS])
        cg.add(hub.register_known_good_profile_status_sensor(sens))
    if DAIKIN_AUTO_SNAPSHOT_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_AUTO_SNAPSHOT_STATUS])
        cg.add(hub.register_auto_snapshot_status_sensor(sens))
