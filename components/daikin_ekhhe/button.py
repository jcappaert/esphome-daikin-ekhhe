import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns, DEBUG_COMPONENTS
from .const import DAIKIN_SAVE_CC_SNAPSHOT, DAIKIN_RESTORE_CC_SNAPSHOT

DaikinEkhheDebugButton = daikin_ekhhe_ns.class_(
    "DaikinEkhheDebugButton", button.Button, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(DAIKIN_SAVE_CC_SNAPSHOT): button.button_schema(
            DaikinEkhheDebugButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        cv.Optional(DAIKIN_RESTORE_CC_SNAPSHOT): button.button_schema(
            DaikinEkhheDebugButton, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    if str(config[CONF_EKHHE_ID]) not in DEBUG_COMPONENTS:
        return

    if DAIKIN_SAVE_CC_SNAPSHOT in config:
        conf = config[DAIKIN_SAVE_CC_SNAPSHOT]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression("esphome::daikin_ekkhe::DaikinEkhheDebugButton::Action::SAVE_SNAPSHOT"),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))

    if DAIKIN_RESTORE_CC_SNAPSHOT in config:
        conf = config[DAIKIN_RESTORE_CC_SNAPSHOT]
        var = cg.new_Pvariable(
            conf[CONF_ID],
            cg.RawExpression("esphome::daikin_ekkhe::DaikinEkhheDebugButton::Action::RESTORE_SNAPSHOT"),
        )
        await button.register_button(var, conf)
        cg.add(var.set_parent(hub))
