# Daikin EKHHE esphome external component
This is an esphome external component for the Daikin EKHHE series of domestic hot water heat pumps, also known as the Altherma M HW. 

The component exposes all of the known sensors and settings to the user. 

So far, it's been tested only with an EKHHE260PCV37 model. 

This component has some basic functionality now but still needs to be further developed to be fully functional. 

**!! WARNING !!**: Use at your own risk, has the potential to change the properties of your device and might even damage it if inappropriate settings are applied. 

## YAML setup
In the YAML you can enter which sensors/numbers/selects etc. you want to use. In the example yaml, all available entities are listed, you can remove what you don't need. 


Other than this, the only customization available for the module is:
* update_interval
* mode (production or debug)
* continuous_rx (debug only)

`update_interval` sets the normal polling interval in seconds when the component is idle.

`mode: debug` enables extra raw-frame logging and optional debug entities.

`continuous_rx: true` is an additional debug-only option that keeps RX running continuously after each parsed cycle for
reverse engineering. By default it is `false`, so even in debug mode `update_interval` is respected unless a write is
in flight.

## Debug / Reverse Engineering
There is a debug mode that enables internal raw UART capture and optional Home Assistant entities for inspection. These
entities are only active when `mode: debug` is set on the component.

### Debug entities (optional)
Text sensors:
* daikin_raw_frame_hex
* daikin_raw_frame_meta
* daikin_unknown_fields
* daikin_frame_diff

Sensors:
* frames_captured_total
* frames_dropped_total
* frames_truncated_total
* crc_errors_total
* framing_errors_total
* bytes_captured_total
* cycle_parse_time_ms
* cycle_total_time_ms
* cycle_over_budget_total

Controls:
* daikin_debug_packet (select: latest, DD, D2, D4, C1, CC)
* daikin_debug_freeze (switch)
* daikin_save_cc_snapshot (button)
* daikin_restore_cc_snapshot (button)

Snapshot text sensor:
* daikin_cc_snapshot_hex

### Example YAML files
See `example-production.yaml` and `example-debug.yaml` in the repository root.

If all goes well, you should get something like this in the UI (there are a lot of paramters and variables ...):
![esphome UI example](https://github.com/jcappaert/esphome-daikin-ekhhe/blob/main/images/ekhhe_all.PNG)

## Hardware 
You will need the esp32 UART hooked up to a UART/RS485 converter, connected to A/B/GND in CN23. This will need to be spliced in somehow as the display needs to remain connected. You can tap 5V for an esp32 board from CN21 or CN22. Some information also [here](https://github.com/lorbetzki/Daikin-EKHHE) by lorbetzki. 

## Reverse Engineering
A lot of the protocol reverse engineering has been done by lorbetzki [here](https://github.com/lorbetzki/Daikin-EKHHE).

## RX/TX behavior
The component listens on the UART/RS485 bus and processes a repeating read cycle. Each cycle collects a set of packet
types (DD, D2, D4, C1, CC). Frames are assembled by detecting a start byte and then reading the expected length from
the packet size table. Packets that require a checksum are validated before they are accepted into the cycle set.

When all required packets are present, parsing runs and sensors are updated. The latest valid CC packet is always
stored because it is also the base for writes.

When idle, RX follows `update_interval`. A write request bypasses that idle wait: it immediately starts an RX cycle if
needed and then waits for the next observed D2 packet.

For TX, the component reuses the last received CC packet, changes a single byte/bit (or sends a snapshot), rewrites the
checksum, and transmits a CD packet. The write is scheduled relative to the observed D2 packet rather than being sent
immediately.

Writes are confirmed from subsequent D2 readback, not from the transmit itself. If the requested value is not yet
present in D2, the component retries on later cycles, up to a small fixed maximum. If the value still does not apply,
it logs a warning (`TX not applied`). If it eventually applies after retries, it also logs a warning so that
non-first-try writes are visible in the logs.

While a write is pending, and for a short UI-sync phase immediately after D2 confirms success, the component keeps RX
alive even when `continuous_rx` is disabled. During that UI-sync phase, stale CC updates for the target field are
suppressed until CC also reflects the applied value, so Home Assistant does not briefly jump back to the old value. If
CC does not catch up after a few cycles, the UI-sync phase times out and normal polling resumes.

## CC snapshot / restore (RAM only)
In debug mode you can store the latest valid CC frame in RAM and later re-send it as a CD packet. The snapshot is
volatile (not persisted to flash) and displayed as hex via `daikin_cc_snapshot_hex`. If no valid CC frame is available,
the snapshot is marked as EMPTY and restore does nothing.

## TODO
Some main TODOs to get to full functionality are:

* Find where in the packets the following items are:
  - Protection and fault codes
  - Silent mode  
