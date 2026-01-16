import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor
from esphome.const import (
    CONF_ID
)

CONF_UPDATE_INTERVAL = "update_interval"
CONF_MODE = "mode"
DEBUG_COMPONENTS = set()

CODEOWNERS = ["@jcappaert"]

DEPENDENCIES = ["uart"]

daikin_ekhhe_ns = cg.esphome_ns.namespace("daikin_ekkhe")
DaikinEkhhe = daikin_ekhhe_ns.class_("DaikinEkhheComponent", cg.Component, uart.UARTDevice)
MULTI_CONF = True

CONF_EKHHE_ID = "ekhhe_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DaikinEkhhe),
            cv.Optional(CONF_UPDATE_INTERVAL, default=10): cv.positive_int,
            cv.Optional(CONF_MODE, default="production"): cv.one_of("production", "debug", lower=True),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    update_interval_ms = config[CONF_UPDATE_INTERVAL] * 1000 
    cg.add(var.set_update_interval(update_interval_ms))
    debug_mode = config[CONF_MODE] == "debug"
    cg.add_build_flag(f"-DDAIKIN_EKHHE_DEBUG={1 if debug_mode else 0}")
    if debug_mode:
        DEBUG_COMPONENTS.add(str(config[CONF_ID]))
