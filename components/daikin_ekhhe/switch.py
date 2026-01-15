import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns
from .const import DAIKIN_DEBUG_FREEZE

DaikinEkhheDebugSwitch = daikin_ekhhe_ns.class_(
    "DaikinEkhheDebugSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(DAIKIN_DEBUG_FREEZE): switch.switch_schema(
            DaikinEkhheDebugSwitch, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    if DAIKIN_DEBUG_FREEZE in config:
        conf = config[DAIKIN_DEBUG_FREEZE]
        sw = await switch.new_switch(conf)
        cg.add(hub.register_debug_switch(sw))
        cg.add(sw.set_parent(hub))
