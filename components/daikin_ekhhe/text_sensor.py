import esphome.codegen as cg
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
from .schema_helpers import (
    TextSensorSchemaSpec,
    optional_schema_entries,
    platform_schema,
    text_sensor_schema,
)


TEXT_SENSOR_SPECS = [
    TextSensorSchemaSpec(CURRENT_TIME, device_class=DEVICE_CLASS_TIMESTAMP),
    TextSensorSchemaSpec(J_POWER_FW_VERSION, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    TextSensorSchemaSpec(L_UI_FW_VERSION, entity_category=ENTITY_CATEGORY_DIAGNOSTIC),
    TextSensorSchemaSpec(
        DAIKIN_KNOWN_GOOD_PROFILE_STATUS,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    TextSensorSchemaSpec(
        DAIKIN_AUTO_SNAPSHOT_STATUS,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
]

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    optional_schema_entries(TEXT_SENSOR_SPECS, text_sensor_schema),
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
