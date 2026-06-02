import logging

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import (
    CONF_ID
)

CONF_UPDATE_INTERVAL = "update_interval"
CONF_MODE = "mode"
CONF_CONTINUOUS_RX = "continuous_rx"
CONF_TX_SEND_CALIBRATION = "tx_send_calibration"
_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@jcappaert"]

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor", "number", "select", "sensor", "text_sensor", "time"]

daikin_ekhhe_ns = cg.esphome_ns.namespace("daikin_ekkhe")
DaikinEkhhe = daikin_ekhhe_ns.class_("DaikinEkhheComponent", cg.Component, uart.UARTDevice)
MULTI_CONF = True

CONF_EKHHE_ID = "ekhhe_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DaikinEkhhe),
            cv.Optional(CONF_UPDATE_INTERVAL, default=10): cv.positive_int,
            cv.Optional(CONF_MODE): cv.one_of("production", "debug", lower=True),
            cv.Optional(CONF_CONTINUOUS_RX, default=False): cv.boolean,
            cv.Optional(CONF_TX_SEND_CALIBRATION, default=75): cv.int_range(min=0, max=250),
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
    cg.add(var.set_tx_delay_after_d2_ms(config[CONF_TX_SEND_CALIBRATION]))
    cg.add(var.set_continuous_rx(config[CONF_CONTINUOUS_RX]))
    if CONF_MODE in config:
        _LOGGER.warning(
            "The daikin_ekhhe 'mode' option is deprecated and no longer changes "
            "component behavior; remove it from your YAML when convenient."
        )
