import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns
from .const import DAIKIN_DEBUG_FREEZE, SILENT_MODE

DaikinEkhheSwitch = daikin_ekhhe_ns.class_(
    "DaikinEkhheSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(SILENT_MODE): switch.switch_schema(
            DaikinEkhheSwitch, entity_category=ENTITY_CATEGORY_CONFIG
        ),
        cv.Optional(DAIKIN_DEBUG_FREEZE): switch.switch_schema(
            switch.Switch, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    if SILENT_MODE in config:
        conf = config[SILENT_MODE]
        sw = await switch.new_switch(conf)
        cg.add(hub.register_switch(SILENT_MODE, sw))
        cg.add(sw.set_parent(hub))
        cg.add(sw.set_internal_id(SILENT_MODE))
