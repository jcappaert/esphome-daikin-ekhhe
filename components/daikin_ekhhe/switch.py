import esphome.codegen as cg
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns
from .const import SILENT_MODE
from .schema_helpers import (
    SwitchSchemaSpec,
    optional_schema_entries,
    platform_schema,
    switch_schema,
)

DaikinEkhheSwitch = daikin_ekhhe_ns.class_(
    "DaikinEkhheSwitch", switch.Switch, cg.Component
)

SWITCH_SPECS = [
    SwitchSchemaSpec(SILENT_MODE, entity_category=ENTITY_CATEGORY_CONFIG),
]

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    optional_schema_entries(
        SWITCH_SPECS, lambda spec: switch_schema(DaikinEkhheSwitch, spec)
    ),
)


async def to_code(config):
    if SILENT_MODE not in config:
        return

    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    conf = config[SILENT_MODE]
    sw = await switch.new_switch(conf)
    cg.add(hub.register_switch(SILENT_MODE, sw))
    cg.add(sw.set_parent(hub))
    cg.add(sw.set_internal_id(SILENT_MODE))
