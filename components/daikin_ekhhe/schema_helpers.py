from dataclasses import dataclass
from typing import Any, Mapping, Optional

import esphome.config_validation as cv
from esphome.components import (
    binary_sensor,
    button,
    number,
    select,
    sensor,
    switch,
    text_sensor,
)
from esphome.const import (
    CONF_DEVICE_CLASS,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_OPTIONS,
    CONF_STEP,
    CONF_UNIT_OF_MEASUREMENT,
)


@dataclass(frozen=True)
class NumberSchemaSpec:
    key: str
    min_value: float
    max_value: float
    step: float = 1
    device_class: Optional[str] = None
    unit_of_measurement: Optional[str] = None
    entity_category: Optional[str] = None


@dataclass(frozen=True)
class SelectSchemaSpec:
    key: str
    options: Mapping[int, str]
    entity_category: Optional[str] = None


@dataclass(frozen=True)
class SensorSchemaSpec:
    key: str
    unit_of_measurement: Optional[str] = None
    accuracy_decimals: Optional[int] = None
    device_class: Optional[str] = None
    state_class: Optional[str] = None
    entity_category: Optional[str] = None


@dataclass(frozen=True)
class BinarySensorSchemaSpec:
    key: str
    device_class: Optional[str] = None
    entity_category: Optional[str] = None


@dataclass(frozen=True)
class TextSensorSchemaSpec:
    key: str
    device_class: Optional[str] = None
    entity_category: Optional[str] = None


@dataclass(frozen=True)
class ButtonSchemaSpec:
    key: str
    entity_category: Optional[str] = None
    icon: Optional[str] = None


@dataclass(frozen=True)
class SwitchSchemaSpec:
    key: str
    entity_category: Optional[str] = None


def ensure_option_map(value: Mapping[int, str]) -> Mapping[int, str]:
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


def platform_schema(hub_conf_key: str, hub_cls: Any, entries: Mapping[Any, Any]) -> cv.Schema:
    return cv.Schema(
        {
            cv.GenerateID(hub_conf_key): cv.use_id(hub_cls),
            **entries,
        }
    )


def optional_schema_entries(specs: list[Any], schema_factory: Any) -> dict[Any, Any]:
    return {cv.Optional(spec.key): schema_factory(spec) for spec in specs}


def number_schema(entity_cls: Any, spec: NumberSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    schema = {
        cv.GenerateID(): cv.declare_id(entity_cls),
        cv.Optional(CONF_MAX_VALUE, default=spec.max_value): cv.float_,
        cv.Optional(CONF_MIN_VALUE, default=spec.min_value): cv.float_,
        cv.Optional(CONF_STEP, default=spec.step): cv.positive_float,
    }
    if spec.device_class is not None:
        schema[cv.Optional(CONF_DEVICE_CLASS, default=spec.device_class)] = cv.string
    if spec.unit_of_measurement is not None:
        schema[cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=spec.unit_of_measurement)] = cv.string

    return number.number_schema(entity_cls, **schema_kwargs).extend(schema)


def select_schema(entity_cls: Any, spec: SelectSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    return select.select_schema(entity_cls, **schema_kwargs).extend(
        {
            cv.GenerateID(): cv.declare_id(entity_cls),
            cv.Optional(CONF_OPTIONS, default=dict(spec.options)): ensure_option_map,
        }
    )


def sensor_schema(spec: SensorSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.unit_of_measurement is not None:
        schema_kwargs["unit_of_measurement"] = spec.unit_of_measurement
    if spec.accuracy_decimals is not None:
        schema_kwargs["accuracy_decimals"] = spec.accuracy_decimals
    if spec.device_class is not None:
        schema_kwargs["device_class"] = spec.device_class
    if spec.state_class is not None:
        schema_kwargs["state_class"] = spec.state_class
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    return sensor.sensor_schema(**schema_kwargs)


def binary_sensor_schema(spec: BinarySensorSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.device_class is not None:
        schema_kwargs["device_class"] = spec.device_class
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    return binary_sensor.binary_sensor_schema(**schema_kwargs)


def text_sensor_schema(spec: TextSensorSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.device_class is not None:
        schema_kwargs["device_class"] = spec.device_class
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    return text_sensor.text_sensor_schema(**schema_kwargs)


def button_schema(entity_cls: Any, spec: ButtonSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category
    if spec.icon is not None:
        schema_kwargs["icon"] = spec.icon

    return button.button_schema(entity_cls, **schema_kwargs)


def switch_schema(entity_cls: Any, spec: SwitchSchemaSpec) -> cv.Schema:
    schema_kwargs = {}
    if spec.entity_category is not None:
        schema_kwargs["entity_category"] = spec.entity_category

    return switch.switch_schema(entity_cls, **schema_kwargs)
