# Hardware

This component listens and writes on the RS485 bus between the Daikin EKHHE control board and the original display. The ESPHome device must share the bus; it does not replace the display.

## Typical Setup

- ESP32 board. The examples use an ESP32-C3 devkit.
- UART-to-RS485 transceiver module.
- Connection to the display bus `A`, `B`, and `GND` lines connected to the main board on CN23.
- Power for the ESP32 board from a suitable local supply.

The original display should remain connected because it is an active participant in the bus protocol and remains the best way to verify settings locally.

## Wiring Notes

Some installations may also be able to source low-voltage power from nearby connectors such as `CN21` or `CN22`, but you must verify this on your own unit and board revision before using it and make sure it's appropriate for your ESP32 board.

Typical UART settings:

```yaml
uart:
  rx_pin: GPIO20
  tx_pin: GPIO21
  baud_rate: 9600
  parity: NONE
  stop_bits: 1
  id: my_uart
```

If you receive no valid frames, try these checks first:

- Confirm ESP32 RX/TX are connected to the RS485 module correctly.
- Confirm the RS485 module direction control or enable line, if present, is wired or configured correctly.
- Swap `A` and `B` if the adapter labels appear inverted.
- Confirm ESP32 ground is connected to bus ground.
- Confirm the original display still works normally.

## Safety Notes

Power down the unit before modifying wiring. Avoid loose splices, floating grounds, and unsupported power taps. This project is reverse engineered and can write installer settings. Wiring mistakes or incorrect settings can misconfigure or damage the device.

## Related Work

Some of the initial protocol and hardware investigation was published by [lorbetzki/Daikin-EKHHE](https://github.com/lorbetzki/Daikin-EKHHE).
