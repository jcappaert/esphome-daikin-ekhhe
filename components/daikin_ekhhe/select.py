import esphome.codegen as cg
from esphome.components import select
from esphome.const import CONF_OPTIONS, ENTITY_CATEGORY_CONFIG

from . import (
    CONF_EKHHE_ID,
    DaikinEkhhe,
    daikin_ekhhe_ns,
)
from .const import *
from .schema_helpers import (
    SelectSchemaSpec,
    optional_schema_entries,
    platform_schema,
    select_schema,
)

CODEOWNERS = ["@jcappaert"]

DaikinEkhheSelect = daikin_ekhhe_ns.class_(
    "DaikinEkhheSelect", select.Select, cg.Component
)

SELECT_SPECS = [
    SelectSchemaSpec(POWER_STATUS, {0: "Standby", 1: "On"}),
    SelectSchemaSpec(
        OPERATIONAL_MODE,
        {
            0: "Auto",
            1: "Eco",
            2: "Boost",
            3: "Electric",
            4: "Fan",
            5: "Vacation",
        },
    ),
    SelectSchemaSpec(P5_DEFROST_MODE, {0: "Compressor-stop", 1: "Hot-gas"}),
    SelectSchemaSpec(P6_EHEATER_DEFROSTING, {0: "Off", 1: "On"}),
    SelectSchemaSpec(P11_DISP_WAT_T_PROBE, {0: "Lower", 1: "Upper"}),
    SelectSchemaSpec(
        P12_EXT_PUMP_MODE,
        {0: "Always-off", 1: "Hot water recirculation", 2: "Thermal Solar System"},
    ),
    SelectSchemaSpec(P13_HW_CIRC_PUMP_MODE, {0: "With heat pump", 1: "Always on"}),
    SelectSchemaSpec(
        P14_EVA_BLOWER_TYPE,
        {0: "EC", 1: "AC", 2: "AC double speed", 3: "EC dynamic speed control"},
    ),
    SelectSchemaSpec(
        P15_SAFETY_SW_TYPE,
        {0: "NC", 1: "NO", 2: "Low pressure selection switch"},
    ),
    SelectSchemaSpec(
        P16_SOLAR_MODE_INT,
        {0: "Permanently deactivated", 1: "DIG1", 2: "direct control"},
    ),
    SelectSchemaSpec(
        P23_PV_MODE_INT,
        {0: "Permanently deactivated", 1: "activated"},
    ),
    SelectSchemaSpec(
        P24_OFF_PEAK_MODE,
        {0: "Permanently deactivated", 1: "activated with eco", 2: "activated with auto"},
    ),
    SelectSchemaSpec(
        P33_EEV_CONTROL,
        {0: "Permanently deactivated", 1: "activated"},
    ),
    SelectSchemaSpec(P39_EEV_MODE, {0: "automatic", 1: "manual"}),
    SelectSchemaSpec(P58_EVA_FAN_COMP_OFF, {0: "Disabled", 1: "Enabled", 2: "Auto"}),
    SelectSchemaSpec(
        TIME_BAND_MODE,
        {0: "Auto", 1: "Eco", 2: "Boost", 3: "Electric", 4: "Fan"},
        entity_category=ENTITY_CATEGORY_CONFIG,
    ),
]

TYPES = [spec.key for spec in SELECT_SPECS]

CONFIG_SCHEMA = platform_schema(
    CONF_EKHHE_ID,
    DaikinEkhhe,
    optional_schema_entries(
        SELECT_SPECS, lambda spec: select_schema(DaikinEkhheSelect, spec)
    ),
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
        cg.add(getattr(hub, f"register_select")(key, sens))
        cg.add(sens.set_parent(hub))
        cg.add(sens.set_internal_id(key))


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    for key in TYPES:
        await setup_conf(config, key, hub)
