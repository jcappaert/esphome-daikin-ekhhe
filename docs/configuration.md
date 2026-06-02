# Configuration

All entities are optional. Start with a small config, confirm stable reads, then add writable settings gradually.

## Examples

- [`examples/minimal.yaml`](../examples/minimal.yaml): recommended starting point with core readings and operating controls.
- [`examples/full.yaml`](../examples/full.yaml): broader normal-user setup exposing most supported entities.

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
```

## Component Options

| Option | Default | Description |
| --- | ---: | --- |
| `update_interval` | `10` | Idle polling interval in seconds. Higher values can reduce CPU load on more constrained ESP variants. Writes temporarily keep RX active until confirmation and UI sync complete. |
| `continuous_rx` | `false` | When true, RX starts another cycle immediately after each complete cycle instead of waiting for `update_interval`. Writes and restore/profile actions keep RX active as needed even when this is false. |
| `mode` | none | Deprecated compatibility shim. Accepted legacy values are ignored at runtime; remove this option from new YAML. |
| `tx_send_calibration` | `75` | Delay in milliseconds used when scheduling write packets. This might need to be calibrated for your specific ESP32 chip if parameters writes are not working. |

The optional `tx_send_calibration` number entity can expose this timing as a Number. Changes made through the number entity apply immediately but are not persisted unless you also update YAML.

## Entity Groups

Entities are optional and are declared under the normal ESPHome platforms (`sensor`, `binary_sensor`, `select`, `number`, `button`, and `text_sensor`). The groups below are organized by device functionality instead of YAML platform.

### Temperature And Runtime Readings

Useful read-only values for monitoring the heat pump:

- `low_water_temp_probe`
- `upper_water_temp_probe`
- `defrost_temp_probe`
- `supply_air_temp_probe`
- `evaporator_inlet_gas_temp_probe`
- `evaporator_outlet_gas_temp_probe`
- `compressor_discharge_gas_temp_probe`
- `solar_collector_temp_probe`
- `eev_opening_step`
- `fan_speed_rpm`

### Basic Operation

Common day-to-day controls and state indicators:

- `power_status`
- `operational_mode`
- `silent_mode`
- `hp_active`
- `eh_active`
- `dd_heating_demand`
- `vacation_days`

`silent_mode` can be enabled only while the unit is in Auto, Eco, or Boost mode. Disabling it is allowed from any mode so the setting can be cleared safely if the operating mode changes.

`dd_heating_demand`, `hp_active`, and `eh_active` are semantic status indicators decoded from the runtime status packet; user-facing names should describe heat demand and active equipment rather than packet bytes.

### Fault And Alarm Indicators

Read-only binary sensors for confirmed display faults and alarms:

- `p01_tank_lower_probe_fault`
- `p02_tank_upper_probe_fault`
- `p03_defrost_probe_fault`
- `p04_inlet_air_probe_fault`
- `p05_evaporator_inlet_probe_fault`
- `p06_evaporator_outlet_probe_fault`
- `p07_compressor_flow_probe_fault`
- `p08_solar_collector_probe_fault`
- `e01_high_pressure_protection`
- `e02_solar_recirculation_alarm`
- `e03_electronic_fan_fault`
- `pa_heat_pump_temperature_unsuitable_alarm`

These indicators mirror observed display states. Use the Daikin display and
manual as the primary diagnostic reference when a fault is active.

`E04` and `E08` are not exposed yet. They are intentionally omitted because
they have not been reproducibly observed on this unit in a bus-visible form:
`E04` may not be present on all manual/hardware variants, and `E08` appears to
be display-local or otherwise not reported as a confirmed status bit on the
captured display bus.

### Target Temperatures

Writable target temperatures for the operating modes:

- `auto_target_temperature`
- `eco_target_temperature`
- `boost_target_temperature`
- `electric_target_temperature`

### Digital Input Configuration

Configuration and status related to the digital inputs:

- `dig1_config`
- `dig2_config`
- `dig3_config`
- `dig1_hp_start_delay`
- `dig1_low_water_temp_threshold`

### Installer Parameters

The component exposes known installer parameters as either numbers or selects depending on how the parameter behaves in the Daikin UI.

Writable numeric parameters currently include:

- `P1-P4`
- `P7-P10`
- `P17-P22`
- `P25-P32`
- `P34-P38`
- `P40-P57`
- `P59-P72`

Menu-style parameters currently include:

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

The menu-style list corresponds to `P5`, `P6`, `P11-P16`, `P23`, `P24`, `P33`, `P39`, and `P58`.

### Firmware And Clock

Diagnostic identity and time values:

- `current_time`
- `power_board_firmware_version`
- `ui_firmware_version`

### Recovery And Maintenance

Recovery controls and status entities:

- `daikin_restore_default_settings`
- `daikin_save_known_good_profile`
- `daikin_restore_known_good_profile`
- `daikin_restore_auto_snapshot`
- `daikin_known_good_profile_status`
- `daikin_auto_snapshot_status`

Read [operations](operations.md) before enabling or pressing restore buttons.

### TX Timing

Optional timing calibration:

- `tx_send_calibration`

This can be useful if read values work but writes repeatedly fail on your specific ESP32 or RS485 setup.

### Protocol Capture

ESPHome entities are intended for supported runtime state and controls. Raw packet inspection, unknown-field comparison, and packet diffing should be done with protocol-lab tooling outside the production ESPHome component. Use `continuous_rx: true` only when an installed node intentionally needs uninterrupted receive cycles during normal operation.
