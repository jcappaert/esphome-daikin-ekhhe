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

CPP_STRING_RE = re.compile(
    r"static\s+const\s+std::string\s+([A-Z][A-Z0-9_]*)\s*=\s*\"([^\"]*)\""
)

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

    if errors:
        print("Component metadata validation failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print(
        "Component metadata validation passed: "
        f"{len(cpp_constants)} mirrored constants checked, "
        f"{len(PYTHON_ONLY_CONSTANTS)} Python-only constants allowed."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
