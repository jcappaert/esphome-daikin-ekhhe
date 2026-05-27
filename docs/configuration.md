# Configuration

All entities are optional. Start with a small config, confirm stable reads, then add writable settings gradually.

## Examples

- [`examples/minimal.yaml`](../examples/minimal.yaml): recommended starting point with core readings and operating controls.
- [`examples/production.yaml`](../examples/production.yaml): broader normal-user setup exposing most supported entities.
- [`examples/debug.yaml`](../examples/debug.yaml): diagnostic setup with raw packet and bus-health entities.

## Component Setup

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/jcappaert/esphome-daikin-ekhhe
      ref: main
    components: [daikin_ekhhe]

uart:
  rx_pin: GPIO20
  tx_pin: GPIO21
  baud_rate: 9600
  parity: NONE
  stop_bits: 1
  id: my_uart

daikin_ekhhe:
  - id: daikin_component
    update_interval: 10
    mode: production
```

## Component Options

| Option | Default | Description |
| --- | ---: | --- |
| `update_interval` | `10` | Idle polling interval in seconds. Writes temporarily keep RX active until confirmation and UI sync complete. |
| `mode` | `production` | Use `production` for normal operation or `debug` to enable debug-only entities and logging paths. |
| `continuous_rx` | `false` | Debug-only option. When true in debug mode, RX keeps running continuously instead of respecting `update_interval`. |
| `tx_send_calibration` | `75` | Delay in milliseconds used when scheduling write packets. Most users should leave this at the default unless diagnosing write failures. |

The optional `tx_send_calibration` number entity can expose this timing in Home Assistant for testing. Changes made through the number entity apply immediately but are not persisted unless you also update YAML.

## Core Entity Groups

### Sensors

The component exposes known probe and runtime readings:

- `low_water_temp_probe`
- `upper_water_temp_probe`
- `defrost_temp_probe`
- `supply_air_temp_probe`
- `evaporator_inlet_gas_temp_probe`
- `evaporator_outlet_gas_temp_probe`
- `compressor_discharge_gas_temp_probe`
- `solar_collector_temp_probe`
- `eev_opening_step`

### Binary Sensors

Normal binary sensors:

- `dig1_config`
- `dig2_config`
- `dig3_config`
- `hp_active`
- `eh_active`

Debug-only binary sensor:

- `dd_heating_demand`

### Selects

Normal selects:

- `power_status`
- `operational_mode`
- `defrosting_mode`
- `elec_heater_during_defrosting`
- `display_water_probe_temp`
- `external_pump_mode`
- `hw_circ_pump_mode`
- `evaporator_blower_type`
- `safety_flow_switch_type`
- `solar_mode_integration`
- `pv_mode_integration`
- `off_peak_working_mode`
- `eev_control`
- `eev_control_mode`
- `evaporator_fan_compressor_off_mode`

Debug-only select:

- `daikin_debug_packet`

### Numbers

Number entities include writable numeric installer parameters, target temperatures, vacation days, and optional timing calibration.

Supported settings currently include `P1-P4`, `P7-P10`, `P17-P22`, `P25-P32`, `P34-P38`, `P40-P57`, and `P59-P72`. `P5`, `P6`, `P11-P16`, `P23`, `P24`, `P33`, `P39`, and `P58` are exposed as selects.

Target temperature numbers:

- `auto_target_temperature`
- `eco_target_temperature`
- `boost_target_temperature`
- `electric_target_temperature`

Other optional numbers:

- `vacation_days`
- `tx_send_calibration`

### Buttons

Maintenance and recovery buttons:

- `daikin_restore_default_settings`
- `daikin_save_known_good_profile`
- `daikin_restore_known_good_profile`
- `daikin_restore_auto_snapshot`

Read [operations](operations.md) before enabling or pressing restore buttons.

### Text Sensors

Normal and diagnostic text sensors:

- `current_time`
- `power_board_firmware_version`
- `ui_firmware_version`
- `daikin_known_good_profile_status`
- `daikin_auto_snapshot_status`

Debug-only text sensors:

- `daikin_raw_frame_hex`
- `daikin_raw_frame_meta`
- `daikin_unknown_fields`
- `daikin_frame_diff`
- `dd_b1_text_sensor`
- `dd_b5_text_sensor`

## Debug Mode

Enable debug mode on the component:

```yaml
daikin_ekhhe:
  - id: daikin_component
    update_interval: 10
    mode: debug
```

Debug mode enables raw frame inspection entities when they are also declared in YAML. By default `update_interval` is still respected in debug mode. Set `continuous_rx: true` only when you need continuous bus capture for troubleshooting or reverse engineering.

## Production Defaults

For normal use:

- Use `mode: production`.
- Leave `continuous_rx` unset.
- Leave `tx_send_calibration` at its default unless write confirmation is unreliable.
- Prefer the minimal example first, then add entities from the production example as needed.
