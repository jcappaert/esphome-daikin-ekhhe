# Development

This page documents the repository workflow for component development, validation, and protocol work.

## Repository Layout

```text
components/daikin_ekhhe/   ESPHome external component
docs/                      User and engineering documentation
examples/                  Public example YAML files
.github/fixtures/          CI validation configs
.github/workflows/ci.yml   GitHub Actions workflow
```

Public examples are designed for humans to copy from. CI fixtures are designed for automated validation and may intentionally be smaller or use local component paths.

## Local Validation

The GitHub Actions workflow validates and compiles two fixture configurations:

- `.github/fixtures/minimal/ci.yaml`
- `.github/fixtures/production/ci.yaml`

Useful local commands:

```bash
esphome config .github/fixtures/minimal/ci.yaml
esphome compile .github/fixtures/minimal/ci.yaml
esphome config .github/fixtures/production/ci.yaml
esphome compile .github/fixtures/production/ci.yaml
```

Run at least the relevant fixture when changing optional entities, schema, packet write logic, receive-cycle behavior, or recovery/profile features.

## Protocol Work

Protocol notes live in [protocol](protocol.md). Keep that file focused on durable findings, confidence levels, packet families, and implementation implications.

Use protocol-lab tooling for raw packet capture, unknown-field comparison, and packet diffing. The ESPHome component should only grow decoded, user-facing entities or operational logs that are useful on installed devices.

When adding newly decoded fields:

- Update Python entity definitions and C++ packet mappings together.
- Add or update CI fixture coverage for optional entity compilation.
- Update `docs/configuration.md` if the user-facing entity set changes.
- Update `docs/protocol.md` if the finding changes packet-family understanding.

## CI Notes

The workflow caches Python dependencies and PlatformIO packages to keep ESPHome compilation reasonably fast. The minimal fixture is especially important because it catches accidental hard dependencies on optional entities.
