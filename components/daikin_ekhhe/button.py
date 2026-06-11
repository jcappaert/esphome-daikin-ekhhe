import esphome.codegen as cg
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns
from .const import (
    DAIKIN_RESTORE_AUTO_SNAPSHOT,
    DAIKIN_RESTORE_DEFAULT_SETTINGS,
    DAIKIN_RESTORE_KNOWN_GOOD_PROFILE,
    DAIKIN_SAVE_KNOWN_GOOD_PROFILE,
    DAIKIN_APPLY_TIME_BAND,
    DAIKIN_CLEAR_TIME_BAND,
)
from .schema_helpers import (
    ButtonSchemaSpec,
    button_schema,
    optional_schema_entries,
    platform_schema,
)

DaikinEkhheActionButton = daikin_ekhhe_ns.class_(
    "DaikinEkhheActionButton", button.Button, cg.Component
)

BUTTON_SPECS = [
    ButtonSchemaSpec(
        DAIKIN_RESTORE_DEFAULT_SETTINGS,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:backup-restore",
    ),
    ButtonSchemaSpec(
        DAIKIN_SAVE_KNOWN_GOOD_PROFILE,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:content-save-check",
    ),
    ButtonSchemaSpec(
        DAIKIN_RESTORE_KNOWN_GOOD_PROFILE,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:restore",
    ),
    ButtonSchemaSpec(
        DAIKIN_RESTORE_AUTO_SNAPSHOT,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:restore-clock",
    ),
    ButtonSchemaSpec(
        DAIKIN_APPLY_TIME_BAND,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:clock-check-outline",
    ),
    ButtonSchemaSpec(
        DAIKIN_CLEAR_TIME_BAND,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:clock-remove-outline",
    ),
]

BUTTON_ACTIONS = {
    DAIKIN_RESTORE_DEFAULT_SETTINGS: "RESTORE_DEFAULT_SETTINGS",
    DAIKIN_SAVE_KNOWN_GOOD_PROFILE: "SAVE_KNOWN_GOOD_PROFILE",
    DAIKIN_RESTORE_KNOWN_GOOD_PROFILE: "RESTORE_KNOWN_GOOD_PROFILE",
    DAIKIN_RESTORE_AUTO_SNAPSHOT: "RESTORE_AUTO_SNAPSHOT",
    DAIKIN_APPLY_TIME_BAND: "APPLY_TIME_BAND",
    DAIKIN_CLEAR_TIME_BAND: "CLEAR_TIME_BAND",
}

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    optional_schema_entries(
        BUTTON_SPECS, lambda spec: button_schema(DaikinEkhheActionButton, spec)
    ),
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])

    for spec in BUTTON_SPECS:
        if spec.key not in config:
            continue
        conf = config[spec.key]
        action = BUTTON_ACTIONS[spec.key]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression(
                f"esphome::daikin_ekkhe::DaikinEkhheActionButton::Action::{action}"
            ),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))
