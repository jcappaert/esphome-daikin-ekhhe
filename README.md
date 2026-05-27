# ESPHome Daikin EKHHE

[![CI](https://github.com/jcappaert/esphome-daikin-ekhhe/actions/workflows/ci.yml/badge.svg)](https://github.com/jcappaert/esphome-daikin-ekhhe/actions/workflows/ci.yml)
[![License: GPLv3](https://img.shields.io/badge/license-GPLv3-blue.svg)](LICENSE)
[![ESPHome external component](https://img.shields.io/badge/ESPHome-external_component-00B9D8.svg)](https://esphome.io/components/external_components/)

ESPHome external component for Daikin EKHHE domestic hot water heat pumps, also known as Altherma M HW.

The component exposes known heat-pump readings, operating controls, installer parameters, target temperatures, recovery buttons, and debug tools through ESPHome and Home Assistant.

## Status And Safety

This project is experimental and reverse engineered. It has been tested primarily with an `EKHHE260PCV37`.

Writes can change installer parameters on the heat pump. Incorrect settings, protocol bugs, bad wiring, or timing issues may misconfigure or damage your device. Use this component only if you are comfortable validating settings on the physical unit and recovering from a bad configuration.

Recommended safety steps:

- Start from [`examples/minimal.yaml`](examples/minimal.yaml).
- Confirm read-only sensors are stable before enabling many writable entities.
- Save a known-good profile before making changes.
- Keep the original Daikin display connected and available for verification.
- Read [operations and recovery](docs/operations.md) before using restore buttons or broad parameter writes.

## Features

- Temperature and runtime sensors for the known EKHHE probe values.
- Binary status indicators for digital inputs, heat-pump activity, and electric-heater activity.
- Select entities for power state, operating mode, and known menu-style parameters.
- Number entities for known numeric installer parameters, target temperatures, and TX timing calibration.
- Persistent known-good and automatic recovery snapshots for managed settings.
- Restore-defaults button for documented default values in the supported main settings block.
- Debug mode with raw packet capture, frame metadata, packet diffs, and bus health counters.
- Support for both known write packet families:
  - Main settings: `CC` base, `CD` write, `D2` readback.
  - Extended settings: `C1` base, `C2` write, `D4` readback.

## Hardware

You need an ESP32 with a UART connected through an RS485 transceiver to the display bus. The original display must remain connected.

Typical wiring uses:

- RS485 `A`, `B`, and `GND` tapped from the display bus connector.
- ESP32 UART RX/TX connected to the RS485 transceiver.
- A suitable 5 V or 3.3 V power arrangement for your ESP32 board.

See [hardware](docs/hardware.md) for wiring notes and safety cautions.

## Quick Start

Add this repository as an ESPHome external component:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/jcappaert/esphome-daikin-ekhhe
      ref: main
    components: [daikin_ekhhe]
```

Configure the UART and component:

```yaml
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

Then add only the entities you want:

```yaml
sensor:
  - platform: daikin_ekhhe
    low_water_temp_probe:
      name: "Lower water temperature"
    upper_water_temp_probe:
      name: "Upper water temperature"

select:
  - platform: daikin_ekhhe
    power_status:
      name: "Power status"
    operational_mode:
      name: "Operational mode"
```

For complete starting points, use:

- [`examples/minimal.yaml`](examples/minimal.yaml): small everyday setup.
- [`examples/production.yaml`](examples/production.yaml): broader normal-user setup.
- [`examples/debug.yaml`](examples/debug.yaml): diagnostic and reverse-engineering setup.

## Documentation

- [Hardware](docs/hardware.md): RS485 wiring, display-bus tap, and installation cautions.
- [Configuration](docs/configuration.md): YAML options, examples, entity groups, debug mode, and TX timing.
- [Operations](docs/operations.md): writes, retries, profiles, restore defaults, and troubleshooting.
- [Protocol](docs/protocol.md): reverse-engineered packet families, cycle timing, and write behavior.
- [Development](docs/development.md): local validation, CI fixtures, and debugging workflow.

## Screenshots

Current screenshots are being refreshed.

Planned image slots:

- Home Assistant overview with the minimal entity set.
- Production entity set with grouped settings.
- Debug mode packet inspection entities.

## Credits

Protocol reverse engineering builds on earlier work by [lorbetzki/Daikin-EKHHE](https://github.com/lorbetzki/Daikin-EKHHE) plus live captures and testing from this project.
