import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import (
    CONF_ID,
    CONF_OPTIONS,
    ENTITY_CATEGORY_DIAGNOSTIC
)

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
    daikin_ekhhe_ns,
)

from .const import *

CODEOWNERS = ["@jcappaert"]

TYPES = [
    POWER_STATUS,
    OPERATIONAL_MODE,
    P5_DEFROST_MODE,
    P6_EHEATER_DEFROSTING,
    P11_DISP_WAT_T_PROBE,
    P12_EXT_PUMP_MODE,
    P13_HW_CIRC_PUMP_MODE,
    P14_EVA_BLOWER_TYPE,
    P15_SAFETY_SW_TYPE,
    P16_SOLAR_MODE_INT,
    P23_PV_MODE_INT,
    P24_OFF_PEAK_MODE,
    P33_EEV_CONTROL, 
    P39_EEV_MODE,
]


DaikinEkhheSelect = daikin_ekhhe_ns.class_(
    "DaikinEkhheSelect", select.Select, cg.Component
    )
DaikinEkhheDebugSelect = daikin_ekhhe_ns.class_(
    "DaikinEkhheDebugSelect", select.Select, cg.Component
    )

DEBUG_PACKET_OPTIONS = ["latest", "DD", "D2", "D4", "C1", "CC"]


# taken from tuya select
def ensure_option_map(value):
    cv.check_not_templatable(value)
    option = cv.All(cv.int_range(0, 2**8 - 1))
    mapping = cv.All(cv.string_strict)
    options_map_schema = cv.Schema({option: mapping})
    value = options_map_schema(value)

    all_values = list(value.keys())
    unique_values = set(value.keys())
    if len(all_values) != len(unique_values):
        raise cv.Invalid("Mapping values must be unique.")

    return value




CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
            cv.Optional(POWER_STATUS): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Standby', 1: 'On'}): ensure_option_map
            }),
            cv.Optional(OPERATIONAL_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Auto', 1: 'Eco', 2: 'Boost', 3: 'Electric', 4: 'Fan', 5: 'Vacation'}): ensure_option_map
            }),
            cv.Optional(P5_DEFROST_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Compressor-stop', 1: 'Hot-gas'}): ensure_option_map
            }),
            cv.Optional(P6_EHEATER_DEFROSTING): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Off', 1: 'On'}): ensure_option_map
            }),
            cv.Optional(P11_DISP_WAT_T_PROBE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Lower', 1: 'Upper'}): ensure_option_map
            }),
            cv.Optional(P12_EXT_PUMP_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Always-off', 1: 'Hot water recirculation', 2: 'Thermal Solar System'}): ensure_option_map
            }),
            cv.Optional(P13_HW_CIRC_PUMP_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'With heat pump', 1: 'Always on'}): ensure_option_map
            }),
            cv.Optional(P14_EVA_BLOWER_TYPE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'EC', 1: 'AC', 2: 'AC double speed', 3: 'EC dynamic speed control'}): ensure_option_map
            }),
            cv.Optional(P15_SAFETY_SW_TYPE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'NC', 1: 'NO'}): ensure_option_map
            }),
            cv.Optional(P16_SOLAR_MODE_INT): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Permanently deactivated', 1: 'DIG1', 2: 'direct control'}): ensure_option_map
            }),
            cv.Optional(P23_PV_MODE_INT): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Permanently deactivated', 1: 'activated'}): ensure_option_map
            }),
            cv.Optional(P24_OFF_PEAK_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Permanently deactivated', 1: 'activated with eco', 2: 'activated with auto'}): ensure_option_map
            }),
            cv.Optional(P33_EEV_CONTROL): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'Permanently deactivated', 1: 'activated'}): ensure_option_map
            }),
            cv.Optional(P39_EEV_MODE): select.select_schema(DaikinEkhheSelect).extend({
                cv.GenerateID(): cv.declare_id(DaikinEkhheSelect),
                cv.Optional(CONF_OPTIONS, default={0: 'automatic', 1: 'manual'}): ensure_option_map
            }),
            cv.Optional(DAIKIN_DEBUG_PACKET): select.select_schema(
                DaikinEkhheDebugSelect, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
        }
    )
)


async def setup_conf(config, key, hub):
    if key in config:
        conf = config[key]
        options_map = conf[CONF_OPTIONS]
        sens = await select.new_select(conf, options=list(options_map.values()))

        cpp_map_expr = cg.RawExpression(
            "std::map<std::string, int>{"
            + ", ".join([f'{{"{v}", {k}}}' for k, v in options_map.items()])
            + "}"
        )

        cg.add(sens.set_select_mappings(cpp_map_expr))
        #cg.add(sens.set_select_mappings(list(options_map.keys())))
        #cg.add(sens.set_select_mappings(options_map))
        cg.add(getattr(hub, f"register_select")(key, sens))
        cg.add(sens.set_parent(hub))
        cg.add(sens.set_internal_id(key))



async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
    if DAIKIN_DEBUG_PACKET in config:
        conf = config[DAIKIN_DEBUG_PACKET]
        sens = await select.new_select(conf, options=DEBUG_PACKET_OPTIONS)
        cg.add(hub.register_debug_select(sens))
        cg.add(sens.set_parent(hub))
