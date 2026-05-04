import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, DEVICE_CLASS_TIMESTAMP, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe, DEBUG_COMPONENTS
from .const import CURRENT_TIME, DAIKIN_RAW_FRAME_HEX, DAIKIN_RAW_FRAME_META, DAIKIN_UNKNOWN_FIELDS, DAIKIN_FRAME_DIFF, DAIKIN_KNOWN_GOOD_PROFILE_STATUS, DAIKIN_AUTO_SNAPSHOT_STATUS, DAIKIN_DD_B1_TEXT, DAIKIN_DD_B5_TEXT


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(CURRENT_TIME): text_sensor.text_sensor_schema(
            device_class=DEVICE_CLASS_TIMESTAMP,
        ),
        cv.Optional(DAIKIN_RAW_FRAME_HEX): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_RAW_FRAME_META): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_UNKNOWN_FIELDS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_FRAME_DIFF): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_KNOWN_GOOD_PROFILE_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_AUTO_SNAPSHOT_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_DD_B1_TEXT): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(DAIKIN_DD_B5_TEXT): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])

    if CURRENT_TIME in config:
        sens = await text_sensor.new_text_sensor(config[CURRENT_TIME])
        cg.add(hub.register_timestamp_sensor(sens))
    if DAIKIN_RAW_FRAME_HEX in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_RAW_FRAME_HEX])
        cg.add(hub.register_debug_text_sensor(DAIKIN_RAW_FRAME_HEX, sens))
    if DAIKIN_RAW_FRAME_META in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_RAW_FRAME_META])
        cg.add(hub.register_debug_text_sensor(DAIKIN_RAW_FRAME_META, sens))
    if DAIKIN_UNKNOWN_FIELDS in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_UNKNOWN_FIELDS])
        cg.add(hub.register_debug_text_sensor(DAIKIN_UNKNOWN_FIELDS, sens))
    if DAIKIN_FRAME_DIFF in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_FRAME_DIFF])
        cg.add(hub.register_debug_text_sensor(DAIKIN_FRAME_DIFF, sens))
    if DAIKIN_KNOWN_GOOD_PROFILE_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_KNOWN_GOOD_PROFILE_STATUS])
        cg.add(hub.register_known_good_profile_status_sensor(sens))
    if DAIKIN_AUTO_SNAPSHOT_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_AUTO_SNAPSHOT_STATUS])
        cg.add(hub.register_auto_snapshot_status_sensor(sens))
    if DAIKIN_DD_B1_TEXT in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_DD_B1_TEXT])
        cg.add(hub.set_dd_b1_text(sens))
    if DAIKIN_DD_B5_TEXT in config and str(config[CONF_EKHHE_ID]) in DEBUG_COMPONENTS:
        sens = await text_sensor.new_text_sensor(config[DAIKIN_DD_B5_TEXT])
        cg.add(hub.set_dd_b5_text(sens))
