# Daikin EKHHE esphome external component
This is an esphome external component for the Daikin EKHHE series of hot water domestic heatpumps, also known as the Altherma M HW. 

The component exposes all of the known sensors and settings to the user. 

So far, it's been tested only with an EKHHE260PCV37 model. 

This component has some basic functionality now but still needs to be further developed to be fully functional. 

**!! WARNING !!**: Use at your own risk, has the potential to change the properies of your device and might even damage it if inappropriate settings are applied. 

## YAML setup
In the YAML you can enter which sensors/numbers/selects etc. you want to use. In the example yaml, all available entitiesare listed, you can remove what you don't need. 


Other than this, the only customization available for the module is:
* update_interval
* mode (production or debug)

Which sets the interval at which all entities update in seconds. 

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

### Example YAML (debug)
```yaml
daikin_ekhhe:
  - id: daikin_component
    update_interval: 10
    mode: debug

text_sensor:
  - platform: daikin_ekhhe
    daikin_raw_frame_hex:
      name: "Daikin Raw Frame Hex"
    daikin_raw_frame_meta:
      name: "Daikin Raw Frame Meta"
    daikin_unknown_fields:
      name: "Daikin Unknown Fields"
    daikin_frame_diff:
      name: "Daikin Frame Diff"

sensor:
  - platform: daikin_ekhhe
    frames_captured_total:
      name: "Daikin Frames Captured"
    frames_dropped_total:
      name: "Daikin Frames Dropped"
    frames_truncated_total:
      name: "Daikin Frames Truncated"
    crc_errors_total:
      name: "Daikin CRC Errors"
    framing_errors_total:
      name: "Daikin Framing Errors"
    bytes_captured_total:
      name: "Daikin Bytes Captured"
    cycle_parse_time_ms:
      name: "Daikin Cycle Parse Time"
    cycle_total_time_ms:
      name: "Daikin Cycle Total Time"
    cycle_over_budget_total:
      name: "Daikin Cycle Over Budget"

select:
  - platform: daikin_ekhhe
    daikin_debug_packet:
      name: "Daikin Debug Packet"

switch:
  - platform: daikin_ekhhe
    daikin_debug_freeze:
      name: "Daikin Debug Freeze"
```

If all goes well, you should get something like this in the UI (there are a lot of paramters and variables ...):
![esphome UI example](https://github.com/jcappaert/esphome-daikin-ekhhe/blob/main/images/ekhhe_all.PNG)

## Hardware 
You will need the esp32 UART hooked up to a UART/RS485 converter, connected to A/B/GND in CN23. This will need to be spliced in somehow as the display needs to remain connected. You can tap 5V for an esp32 board from CN21 or CN22. Some information also [here](https://github.com/lorbetzki/Daikin-EKHHE) by lorbetzki. 

## Reverse Engineering
A lot of the protocol reverse engineering has been done by lorbetzki [here](https://github.com/lorbetzki/Daikin-EKHHE).

## TODO
Some main TODOs to get to full functionality are:

* Implement better TX UART flow control and match the Daikin protocol as indicate by lorbetzki [here](https://github.com/lorbetzki/Daikin-EKHHE/discussions/2#discussioncomment-12176862).
* Find where in the packets the following items are:
  - Protection and fault codes
  - Silent mode  
