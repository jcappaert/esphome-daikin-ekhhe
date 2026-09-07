# Companion PCB Examples

These configurations target a custom ESP32-C3 companion PCB for the Daikin
EKHHE display/controller bus. The companion PCB is not publicly released yet, so
these files are mainly intended for boards supplied directly to early users.

The board uses an onboard RS485 transceiver and an optional two-channel CT clamp
analog frontend.

## Files

- `factory.yaml`: provisioning image for boards supplied pre-flashed. It starts
  a setup access point, supports captive portal upload, supports Improv over
  USB serial, and keeps RS485 transmit disabled.
- `minimal.yaml`: normal starting configuration for the companion PCB.
- `ct-clamps.yaml`: companion PCB configuration with the two CT clamp inputs
  enabled.

## Pin Mapping

| Function | ESP32-C3 pin |
| --- | --- |
| RS485 RX | `GPIO5` |
| RS485 TX | `GPIO4` |
| RS485 driver enable | `GPIO2` |
| CT1 ADC | `GPIO0` |
| CT2 ADC | `GPIO1` |

`GPIO2` is an ESP32-C3 strapping pin. The examples set
`ignore_strapping_warning: true` where that pin is used.

The factory image is intentionally bus-passive. It should be replaced with
`minimal.yaml`, `ct-clamps.yaml`, or a user-specific configuration before normal
operation.

The setup access point in `factory.yaml` is intentionally open for local
provisioning; do not leave the factory image installed as the normal operating
firmware.
