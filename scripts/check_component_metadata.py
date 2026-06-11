#!/usr/bin/env python3
"""Validate component metadata that is mirrored across Python and C++ source."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PY_CONST_PATH = REPO_ROOT / "components" / "daikin_ekhhe" / "const.py"
CPP_CONST_PATH = REPO_ROOT / "components" / "daikin_ekhhe" / "daikin_ekhhe_const.h"
COMPONENT_DIR = REPO_ROOT / "components" / "daikin_ekhhe"
PRODUCTION_FIXTURE_PATH = REPO_ROOT / ".github" / "fixtures" / "production" / "ci.yaml"

CPP_STRING_RE = re.compile(
    r"static\s+const\s+std::string\s+([A-Z][A-Z0-9_]*)\s*=\s*\"([^\"]*)\""
)

SPEC_CLASS_NAMES = {
    "BinarySensorSchemaSpec",
    "ButtonSchemaSpec",
    "NumberSchemaSpec",
    "SelectSchemaSpec",
    "SensorSchemaSpec",
    "SwitchSchemaSpec",
    "TextSensorSchemaSpec",
}

PLATFORM_SOURCE_PATHS = [
    COMPONENT_DIR / "binary_sensor.py",
    COMPONENT_DIR / "button.py",
    COMPONENT_DIR / "number.py",
    COMPONENT_DIR / "select.py",
    COMPONENT_DIR / "sensor.py",
    COMPONENT_DIR / "switch.py",
    COMPONENT_DIR / "text_sensor.py",
]

REQUIRED_PRODUCTION_FIXTURE_KEYS = {
    "sensor": ["A_LOW_WAT_T_PROBE", "FAN_SPEED_RPM"],
    "runtime binary sensor": ["HEATING_DEMAND", "HP_ACTIVE", "EH_ACTIVE"],
    "fault binary sensor": ["MASTER_FAULT", "P01_TANK_LOWER_PROBE_FAULT"],
    "main number": ["P1_LOW_WAT_PROBE_HYST"],
    "extended number": ["P53_EVA_FAN_DEFR_SPEED", "P72_EC_FAN_REG_GAIN"],
    "time-band number/select": ["TIME_BAND_START_HOUR", "TIME_BAND_MODE"],
    "select": ["OPERATIONAL_MODE", "P58_EVA_FAN_COMP_OFF"],
    "switch": ["SILENT_MODE"],
    "button": ["DAIKIN_RESTORE_DEFAULT_SETTINGS", "DAIKIN_APPLY_TIME_BAND"],
    "text sensor": [
        "J_POWER_FW_VERSION",
        "L_UI_FW_VERSION",
        "DAIKIN_KNOWN_GOOD_PROFILE_STATUS",
    ],
}

# These keys are intentionally Python-only because Python wires them through
# dedicated setters or action enums instead of passing a string key into C++.
PYTHON_ONLY_CONSTANTS = {
    "DAIKIN_AUTO_SNAPSHOT_STATUS",
    "DAIKIN_KNOWN_GOOD_PROFILE_STATUS",
    "DAIKIN_RESTORE_AUTO_SNAPSHOT",
    "DAIKIN_RESTORE_DEFAULT_SETTINGS",
    "DAIKIN_RESTORE_KNOWN_GOOD_PROFILE",
    "DAIKIN_SAVE_KNOWN_GOOD_PROFILE",
    "EH_ACTIVE",
    "HEATING_DEMAND",
    "HP_ACTIVE",
}


def load_python_constants(path: Path) -> dict[str, str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    constants: dict[str, str] = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name):
            continue
        name = node.targets[0].id
        if not name.isupper() or not isinstance(node.value, ast.Constant):
            continue
        if isinstance(node.value.value, str):
            constants[name] = node.value.value
    return constants


def load_cpp_constants(path: Path) -> dict[str, str]:
    constants: dict[str, str] = {}
    for match in CPP_STRING_RE.finditer(path.read_text(encoding="utf-8")):
        constants[match.group(1)] = match.group(2)
    return constants


def is_constant_name(name: str) -> bool:
    return name.isupper() and not name.startswith("CONF_")


def node_location(path: Path, node: ast.AST) -> str:
    line = getattr(node, "lineno", "?")
    return f"{path}:{line}"


def call_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    return None


def literal_number(node: ast.AST) -> float | int | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
        return node.value
    if (
        isinstance(node, ast.UnaryOp)
        and isinstance(node.op, ast.USub)
        and isinstance(node.operand, ast.Constant)
        and isinstance(node.operand.value, (int, float))
    ):
        return -node.operand.value
    return None


def validate_constant_ref(
    name: str,
    constants: dict[str, str],
    errors: list[str],
    context: str,
) -> None:
    if is_constant_name(name) and name not in constants:
        errors.append(f"{context}: unknown component constant {name}")


def validate_number_spec(path: Path, node: ast.Call, errors: list[str]) -> None:
    keywords = {keyword.arg: keyword.value for keyword in node.keywords if keyword.arg}
    min_value = literal_number(keywords.get("min_value", ast.Constant(None)))
    max_value = literal_number(keywords.get("max_value", ast.Constant(None)))
    step = literal_number(keywords.get("step", ast.Constant(1)))

    if min_value is None:
        errors.append(f"{node_location(path, node)}: NumberSchemaSpec missing literal min_value")
    if max_value is None:
        errors.append(f"{node_location(path, node)}: NumberSchemaSpec missing literal max_value")
    if step is None:
        errors.append(f"{node_location(path, node)}: NumberSchemaSpec step must be a literal number")
    if min_value is not None and max_value is not None and min_value > max_value:
        errors.append(
            f"{node_location(path, node)}: NumberSchemaSpec min_value {min_value} "
            f"is greater than max_value {max_value}"
        )
    if step is not None and step <= 0:
        errors.append(f"{node_location(path, node)}: NumberSchemaSpec step must be positive")


def validate_select_spec(path: Path, node: ast.Call, errors: list[str]) -> None:
    if len(node.args) < 2 or not isinstance(node.args[1], ast.Dict):
        errors.append(f"{node_location(path, node)}: SelectSchemaSpec options must be a dict literal")
        return

    option_ids: list[int] = []
    labels: list[str] = []
    for key_node, value_node in zip(node.args[1].keys, node.args[1].values):
        if not isinstance(key_node, ast.Constant) or not isinstance(key_node.value, int):
            errors.append(f"{node_location(path, node)}: select option id must be an integer literal")
            continue
        if key_node.value < 0 or key_node.value > 255:
            errors.append(
                f"{node_location(path, node)}: select option id {key_node.value} "
                "is outside uint8 range"
            )
        option_ids.append(key_node.value)

        if not isinstance(value_node, ast.Constant) or not isinstance(value_node.value, str):
            errors.append(f"{node_location(path, node)}: select option label must be a string literal")
            continue
        labels.append(value_node.value)

    duplicate_ids = sorted({option_id for option_id in option_ids if option_ids.count(option_id) > 1})
    duplicate_labels = sorted({label for label in labels if labels.count(label) > 1})
    if duplicate_ids:
        errors.append(f"{node_location(path, node)}: duplicate select option ids {duplicate_ids}")
    if duplicate_labels:
        errors.append(f"{node_location(path, node)}: duplicate select option labels {duplicate_labels}")


def validate_platform_metadata(constants: dict[str, str], errors: list[str]) -> None:
    for path in PLATFORM_SOURCE_PATHS:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and call_name(node.func) in SPEC_CLASS_NAMES:
                spec_name = call_name(node.func)
                if not node.args:
                    errors.append(f"{node_location(path, node)}: {spec_name} missing key argument")
                    continue
                key_arg = node.args[0]
                if isinstance(key_arg, ast.Name):
                    if key_arg.id != "key":
                        validate_constant_ref(
                            key_arg.id,
                            constants,
                            errors,
                            f"{node_location(path, key_arg)}: {spec_name} key",
                        )
                else:
                    errors.append(
                        f"{node_location(path, key_arg)}: {spec_name} key must be a component constant"
                    )

                if spec_name == "NumberSchemaSpec":
                    validate_number_spec(path, node, errors)
                elif spec_name == "SelectSchemaSpec":
                    validate_select_spec(path, node, errors)

            if isinstance(node, ast.List):
                for element in node.elts:
                    if isinstance(element, ast.Name):
                        validate_constant_ref(
                            element.id,
                            constants,
                            errors,
                            f"{node_location(path, element)}: list item",
                        )

            if isinstance(node, ast.Dict):
                for key_node in node.keys:
                    if isinstance(key_node, ast.Name):
                        validate_constant_ref(
                            key_node.id,
                            constants,
                            errors,
                            f"{node_location(path, key_node)}: dict key",
                        )


def validate_fixture_coverage(constants: dict[str, str], errors: list[str]) -> None:
    fixture = PRODUCTION_FIXTURE_PATH.read_text(encoding="utf-8")
    for group_name, constant_names in REQUIRED_PRODUCTION_FIXTURE_KEYS.items():
        for constant_name in constant_names:
            value = constants.get(constant_name)
            if value is None:
                errors.append(f"Fixture coverage references unknown constant {constant_name}")
                continue
            if re.search(rf"^\s+{re.escape(value)}\s*:", fixture, re.MULTILINE) is None:
                errors.append(
                    f"{PRODUCTION_FIXTURE_PATH}: production fixture does not cover "
                    f"{group_name} key {constant_name} ({value})"
                )


def main() -> int:
    python_constants = load_python_constants(PY_CONST_PATH)
    cpp_constants = load_cpp_constants(CPP_CONST_PATH)
    errors: list[str] = []

    for name, cpp_value in sorted(cpp_constants.items()):
        py_value = python_constants.get(name)
        if py_value is None:
            errors.append(f"{CPP_CONST_PATH}: C++ constant {name} is missing from const.py")
            continue
        if py_value != cpp_value:
            errors.append(
                f"{name} value mismatch: const.py={py_value!r}, "
                f"daikin_ekhhe_const.h={cpp_value!r}"
            )

    mirrored_python_constants = set(python_constants) - PYTHON_ONLY_CONSTANTS
    for name in sorted(mirrored_python_constants - set(cpp_constants)):
        errors.append(
            f"{PY_CONST_PATH}: Python constant {name} is missing from daikin_ekhhe_const.h "
            "or must be added to PYTHON_ONLY_CONSTANTS with a reason"
        )

    stale_python_only = sorted(PYTHON_ONLY_CONSTANTS - set(python_constants))
    for name in stale_python_only:
        errors.append(f"PYTHON_ONLY_CONSTANTS contains unknown constant {name}")

    validate_platform_metadata(python_constants, errors)
    validate_fixture_coverage(python_constants, errors)

    if errors:
        print("Component metadata validation failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(
        "Component metadata validation passed: "
        f"{len(cpp_constants)} mirrored constants checked, "
        f"{len(PYTHON_ONLY_CONSTANTS)} Python-only constants allowed, "
        f"{len(PLATFORM_SOURCE_PATHS)} platform files checked."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
