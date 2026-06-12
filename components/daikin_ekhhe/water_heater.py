import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import water_heater
from esphome.const import CONF_ID

from . import CONF_EKHHE_ID, DaikinEkhhe, daikin_ekhhe_ns

CONF_CURRENT_TEMPERATURE_SOURCE = "current_temperature_source"

DaikinEkhheWaterHeater = daikin_ekhhe_ns.class_(
    "DaikinEkhheWaterHeater", water_heater.WaterHeater
)

WaterHeaterCurrentTemperatureSource = daikin_ekhhe_ns.enum(
    "WaterHeaterCurrentTemperatureSource"
)
CURRENT_TEMPERATURE_SOURCES = {
    "display": WaterHeaterCurrentTemperatureSource.WATER_HEATER_TEMP_SOURCE_DISPLAY,
    "upper": WaterHeaterCurrentTemperatureSource.WATER_HEATER_TEMP_SOURCE_UPPER,
    "lower": WaterHeaterCurrentTemperatureSource.WATER_HEATER_TEMP_SOURCE_LOWER,
}

CONFIG_SCHEMA = water_heater.water_heater_schema(DaikinEkhheWaterHeater).extend(
    {
        cv.GenerateID(CONF_EKHHE_ID): cv.use_id(DaikinEkhhe),
        cv.Optional(CONF_CURRENT_TEMPERATURE_SOURCE, default="display"): cv.enum(
            CURRENT_TEMPERATURE_SOURCES, lower=True
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_EKHHE_ID])
    var = cg.new_Pvariable(config[CONF_ID])

    await water_heater.register_water_heater(var, config)

    cg.add(var.set_parent(hub))
    cg.add(
        var.set_current_temperature_source(
            config[CONF_CURRENT_TEMPERATURE_SOURCE]
        )
    )
    cg.add(hub.register_water_heater(var))
