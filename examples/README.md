# Examples

The example configurations in this directory are intended as starting points:

- `minimal.yaml`: small everyday setup with the native water heater entity, key readings, and basic controls.
- `full.yaml`: broader normal-user setup exposing the native water heater entity plus most supported detailed entities, including optional time-band controls.
- `companion-pcb/`: configurations for a custom ESP32-C3 companion PCB that is not publicly released yet.

Use the examples as references, then copy only the entities you want into your own ESPHome node. The native water heater entity is the easiest Home Assistant dashboard control; the detailed entities remain useful for direct parameters, diagnostics, profiles, and recovery actions. The top-level examples assume an ESP32-C3 with UART on `GPIO20`/`GPIO21`; the companion PCB examples use their documented board pin mapping. Adjust pins, board, Wi-Fi, and node names for your own hardware.
