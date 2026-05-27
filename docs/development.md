# Development

This page documents the repository workflow for component development, validation, and protocol debugging.

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

The GitHub Actions workflow validates and compiles three fixture configurations:

- `.github/fixtures/minimal/ci.yaml`
- `.github/fixtures/production/ci.yaml`
- `.github/fixtures/debug/ci.yaml`

Useful local commands:

```bash
esphome config .github/fixtures/minimal/ci.yaml
esphome compile .github/fixtures/minimal/ci.yaml
esphome config .github/fixtures/production/ci.yaml
esphome compile .github/fixtures/production/ci.yaml
esphome config .github/fixtures/debug/ci.yaml
esphome compile .github/fixtures/debug/ci.yaml
```

Run at least the relevant fixture when changing optional entities, schema, packet write logic, or debug-only features.

## Debug Mode

Set the component to debug mode to enable raw packet inspection entities:

```yaml
daikin_ekhhe:
  - id: daikin_component
    update_interval: 10
    mode: debug
```

Optional debug entities include:

- `daikin_raw_frame_hex`
- `daikin_raw_frame_meta`
- `daikin_unknown_fields`
- `daikin_frame_diff`
- `daikin_debug_packet`
- `daikin_debug_freeze`
- bus health counters such as `frames_captured_total`, `crc_errors_total`, and `cycle_parse_time_ms`

By default, debug mode still respects `update_interval`. Add `continuous_rx: true` only when you need uninterrupted packet capture.

## Protocol Work

Protocol notes live in [protocol](protocol.md). Keep that file focused on durable findings, confidence levels, packet families, and implementation implications.

When adding newly decoded fields:

- Update Python entity definitions and C++ packet mappings together.
- Add or update CI fixture coverage for optional entity compilation.
- Update `docs/configuration.md` if the user-facing entity set changes.
- Update `docs/protocol.md` if the finding changes packet-family understanding.

## CI Notes

The workflow caches Python dependencies and PlatformIO packages to keep ESPHome compilation reasonably fast. The minimal fixture is especially important because it catches accidental hard dependencies on optional entities.
