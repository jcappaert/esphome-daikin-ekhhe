# Development

This page documents the repository workflow for component development and validation.

## Repository Layout

```text
components/daikin_ekhhe/   ESPHome external component
docs/                      User and engineering documentation
examples/                  Public example YAML files
.github/fixtures/          CI validation configs
.github/workflows/ci.yml   GitHub Actions workflow
```

Public examples are designed for humans to copy from. CI fixtures are designed for automated validation and may intentionally be smaller or use local component paths.

## Component Module Map

The C++ component keeps one public `DaikinEkhheComponent` class, with method implementations split by ownership:

- `daikin_ekhhe.cpp`: top-level setup, loop, config logging, checksum helpers, and small coordinator glue.
- `daikin_ekhhe_metadata.cpp`: runtime field descriptors, default/profile descriptor helpers, and lookup helpers.
- `daikin_ekhhe_profiles.cpp`: known-good profile, auto snapshot, restore defaults, and preference persistence behavior.
- `daikin_ekhhe_entities.cpp`: ESPHome entity registration, state publishing, cached UI updates, and entity control adapters.
- `daikin_ekhhe_parse.cpp`: complete-frame parsing and conversion from raw frame values into component state.
- `daikin_ekhhe_rx.cpp`: UART receive buffering, frame assembly, cycle collection, and completed-frame handoff.
- `daikin_ekhhe_tx.cpp`: write admission, TX scheduling, packet construction, retries, readback checks, and write completion.
- `daikin_ekhhe_internal.h`: internal helper types and declarations shared by the split modules.

When adding code, prefer the module that owns the behavior rather than growing the coordinator. Keep single-file helpers file-local with `static` or an anonymous namespace unless another module genuinely needs them.

## Local Validation

The GitHub Actions workflow validates and compiles two fixture configurations:

- `.github/fixtures/minimal/ci.yaml`
- `.github/fixtures/production/ci.yaml`

Useful local commands:

```bash
python3 scripts/check_component_metadata.py
esphome config .github/fixtures/minimal/ci.yaml
esphome compile .github/fixtures/minimal/ci.yaml
esphome config .github/fixtures/production/ci.yaml
esphome compile .github/fixtures/production/ci.yaml
```

Run at least the relevant fixture when changing optional entities, schema, packet write logic, receive-cycle behavior, or recovery/profile features.

`scripts/check_component_metadata.py` is intentionally dependency-light. It can run before ESPHome is installed and checks mirrored Python/C++ constants, schema table references, number ranges, select options, and representative production fixture coverage.

## Adding Entities And Parameters

Most user-facing entities are defined in small Python spec tables under `components/daikin_ekhhe/`.

- Add sensors in `sensor.py` with `SensorSchemaSpec`.
- Add binary sensors in `binary_sensor.py` with `BinarySensorSchemaSpec`.
- Add text sensors in `text_sensor.py` with `TextSensorSchemaSpec`.
- Add buttons in `button.py` with `ButtonSchemaSpec` and a matching action enum mapping.
- Add switches in `switch.py` with `SwitchSchemaSpec`.
- Add numbers in `number.py` with `NumberSchemaSpec`, including min/max/step and units.
- Add selects in `select.py` with `SelectSchemaSpec`, using stable integer values and unique labels.

For any key that is also used by C++, add the same string constant to both `const.py` and `daikin_ekhhe_const.h`. If Python wires a key through a dedicated setter or enum action and C++ does not need the string key, add it to the explicit allowlist in `scripts/check_component_metadata.py`.

Runtime packet mapping lives in C++ descriptors and helper tables:

- Use number/select field descriptors in `daikin_ekhhe_metadata.cpp` for single-parameter writes.
- Use restore/profile field descriptors in `daikin_ekhhe_metadata.cpp` for bulk restore and profile operations.
- Keep packet offsets, bit positions, bit widths, signedness, defaults, and readback locations easy to audit.
- Avoid combining unrelated concepts into one broad table; schema metadata and runtime packet metadata serve different jobs.

When adding or changing an entity:

1. Update the Python schema table.
2. Update C++ constants and runtime descriptors if the value is read, written, restored, or published by C++.
3. Add representative coverage to `.github/fixtures/production/ci.yaml`.
4. Update `docs/configuration.md` if users can configure or observe the new entity.
5. Run `python3 scripts/check_component_metadata.py` plus the relevant ESPHome config/compile checks.

## CI Notes

The workflow caches Python dependencies and PlatformIO packages to keep ESPHome compilation reasonably fast. The metadata validator runs before installing ESPHome so simple table mistakes fail quickly. The minimal fixture is especially important because it catches accidental hard dependencies on optional entities.
