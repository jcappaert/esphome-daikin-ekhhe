# Protocol

This document summarizes the current reverse-engineering model for the Daikin EKHHE / Altherma M HW display bus. It is a working protocol reference, not an official Daikin specification.

## Protocol Overview

The bus appears to be a structured conversation between the display/UI board and the main control board. The control board publishes state and readback packets, and the UI normally answers with packets that mirror or acknowledge that state. When a setting changes, the UI-side packet in that part of the cycle is replaced by a write packet until the main control board reports the changed value back.

Bus observations:

- The bus has a repeating multi-packet cycle.
- `D2` and `CC` carry closely related main settings/state.
- `D4` and `C1` carry closely related extended settings/state.
- `Cx` packets are display/UI-originated and `Dx` packets are control-board-originated.
- UI-originated main-setting writes use `CD` in the normal UI response slot.
- UI-originated extended-setting writes use `C2` with the same payload schema as `C1`.
- Writes are confirmed by later controller readback, not by seeing the transmit packet itself.
- The UI appears to repeat write packets once per cycle until readback reflects the requested value.
- `CC` is the UI's normal response to `D2`, while `CD` is the modified response used for pending main-setting writes.
- `C1` is the UI's normal extended-state packet, while `C2` is the modified packet used for pending extended-setting writes.

## Bus Unknowns

The following areas still need reverse engineering:

- Detailed status flags.
- Detailed equipment/component state beyond the currently decoded fan RPM.
- Fault codes, error codes, and protection codes.
- Status-only fields in `DD`.
- Remaining unknown or partially decoded bitmasks.
- Device identity fields such as model/type, serial number, or additional software versions if present.

## Packet Families

The currently useful mental model is to group packets by ownership and purpose. `Dx` packets come from the main control board and are the authoritative readback source. `Cx` packets come from the display/UI side. In idle operation, the UI-side packets echo or return the current values. During a setting change, the UI-side packet for that family changes into a write packet.

| Family | Control/readback packet | Idle UI packet | UI write packet | Payload size |
| --- | --- | --- | --- | ---: |
| Status/runtime | `DD` | none known | none known | 41 |
| Main settings | `D2` | `CC` | `CD` | 71 |
| Extended settings | `D4` | `C1` | `C2` | 51 |

The implementation uses these families when selecting the packet to send and the packet to trust for confirmation.

## Observed Idle Cycle

With continuous RX enabled, the stable cycle is roughly 1.49 to 1.50 seconds. A commonly observed order is:

```text
D4 -> DD -> C1 -> D2 -> CC -> D4
```

Typical inter-packet timings from captures:

| Transition | Approximate timing |
| --- | ---: |
| `D4 -> DD` | 490 ms |
| `DD -> C1` | 112 to 116 ms |
| `C1 -> D2` | 415 to 420 ms |
| `D2 -> CC` | 138 to 143 ms |
| `CC -> D4` | 337 to 341 ms |

## Main-Block Writes: `CC` / `CD` / `D2`

The main settings block is the best understood write path. In idle operation, the main control board sends `D2` and the UI responds with `CC`. When the UI changes a main-block setting, that normal response becomes `CD`. The changed value is not considered applied until it comes back from the control board in a later `D2`.

Normal idle behavior:

```text
... D2 -> CC ...
```

Main-setting write behavior:

```text
... D2 -> CD -> D4 -> DD -> C1 -> D2 -> ...
```

Observed write behavior:

- `CD` appears in the normal `D2 -> Cx` response slot.
- Repeated `CD` packets are separated by normal cycle traffic.
- The UI does not appear to send `CD -> CD -> CD` back-to-back.
- Burst length is variable; captures have shown two, four, and five-cycle bursts.

The current model is that the UI sends one `CD` per cycle while an edit is pending, then stops once `D2` readback reflects the requested value.

## Extended-Block Writes: `C1` / `C2` / `D4`

The extended settings block follows the same principle with a different packet family. `D4` is the control-board readback packet and `C1` is the normal UI-side packet. When the UI changes an extended setting, the UI-side packet becomes `C2`. The changed value is confirmed against a later `D4`.

Known extended write-packet properties:

- `C2` has start byte `0xC2`.
- `C2` length is 51 bytes.
- `C2` uses the same checksum model as the other checksum-protected packets.
- `C2` uses the same payload schema as `C1`.
- Extended settings read back through `D4`.

Known extended-family fields:

- `P53`
- `P55-P72`

Known exception:

- `P54` is in the main `D2`/`CC`/`CD` family even though it sits numerically among extended parameters.

## Current ESPHome Write Strategy

The component treats writes as temporary UI-side responses:

1. Wait for the relevant bus phase.
2. Build a write packet from the latest valid UI-side base packet.
3. Change the requested field only.
4. Recompute checksum.
5. Transmit the appropriate write packet.
6. Confirm against the matching control/readback packet.
7. Retry on later cycles until readback matches or the retry limit is reached.

For now, both `CD` and `C2` writes use the same `tx_send_calibration` timing option. If future captures show that the extended family needs materially different timing, a separate calibration should be added rather than guessed.
