import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns, DEBUG_COMPONENTS
from .const import (
    DAIKIN_RESTORE_CC_SNAPSHOT,
    DAIKIN_RESTORE_DEFAULT_SETTINGS,
    DAIKIN_SAVE_CC_SNAPSHOT,
)

DaikinEkhheActionButton = daikin_ekhhe_ns.class_(
    "DaikinEkhheActionButton", button.Button, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(DAIKIN_RESTORE_DEFAULT_SETTINGS): button.button_schema(
            DaikinEkhheActionButton, entity_category=ENTITY_CATEGORY_CONFIG
        ),
        cv.Optional(DAIKIN_SAVE_CC_SNAPSHOT): button.button_schema(
            DaikinEkhheActionButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(DAIKIN_RESTORE_CC_SNAPSHOT): button.button_schema(
            DaikinEkhheActionButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])

    if DAIKIN_RESTORE_DEFAULT_SETTINGS in config:
        conf = config[DAIKIN_RESTORE_DEFAULT_SETTINGS]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression("esphome::daikin_ekkhe::DaikinEkhheActionButton::Action::RESTORE_DEFAULT_SETTINGS"),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))

    if str(config[CONF_EKHHE_ID]) not in DEBUG_COMPONENTS:
        return

    if DAIKIN_SAVE_CC_SNAPSHOT in config:
        conf = config[DAIKIN_SAVE_CC_SNAPSHOT]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression("esphome::daikin_ekkhe::DaikinEkhheActionButton::Action::SAVE_SNAPSHOT"),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))

    if DAIKIN_RESTORE_CC_SNAPSHOT in config:
        conf = config[DAIKIN_RESTORE_CC_SNAPSHOT]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression("esphome::daikin_ekkhe::DaikinEkhheActionButton::Action::RESTORE_SNAPSHOT"),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))
