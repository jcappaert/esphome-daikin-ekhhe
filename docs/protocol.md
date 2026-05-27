# Protocol

This document summarizes the current reverse-engineering model for the Daikin EKHHE / Altherma M HW display bus. It is a working protocol reference, not an official Daikin specification.

## Confidence Levels

Strong observations:

- The bus has a repeating multi-packet cycle.
- `D2` and `CC` carry closely related main settings/state.
- `D4` and `C1` carry closely related extended settings/state.
- UI-originated main-setting writes use `CD` in the normal UI response slot.
- UI-originated extended-setting writes use `C2` with the same payload schema as `C1`.
- Writes are confirmed by later controller readback, not by seeing the transmit packet itself.

Moderate observations:

- `Cx` packets are display/UI-originated and `Dx` packets are control-board-originated.
- The UI appears to repeat write packets once per cycle until readback reflects the requested value.
- `CC` is the UI's normal response to `D2`, while `CD` is the modified response used for pending main-setting writes.
- `C1` is the UI's normal extended-state packet, while `C2` is the modified packet used for pending extended-setting writes.

Still open:

- Full fault and protection-code decoding.
- Some status-only fields in `DD`.
- Whether all units and firmware revisions use identical timing margins.

## Packet Families

| Family | Control/readback packet | Idle UI packet | UI write packet | Payload size |
| --- | --- | --- | --- | ---: |
| Status/runtime | `DD` | none known | none known | 41 |
| Main settings | `D2` | `CC` | `CD` | 71 |
| Extended settings | `D4` | `C1` | `C2` | 51 |

The implementation uses these families when selecting a write packet and readback confirmation packet.

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

Some captures showed slight ordering variation, so implementation should reason in terms of packet family and readback packet rather than one rigid global sequence.

## Main-Block Writes: `CC` / `CD` / `D2`

Normal idle behavior:

```text
... D2 -> CC ...
```

Main-setting write behavior:

```text
... D2 -> CD -> D4 -> DD -> C1 -> D2 -> ...
```

Observed UI behavior:

- `CD` appears in the normal `D2 -> Cx` response slot.
- Repeated `CD` packets are separated by normal cycle traffic.
- The UI does not appear to send `CD -> CD -> CD` back-to-back.
- Burst length is variable; captures have shown two, four, and five-cycle bursts.

The current model is that the UI sends one `CD` per cycle while an edit is pending, then stops once `D2` readback reflects the requested value.

## Extended-Block Writes: `C1` / `C2` / `D4`

Targeted captures confirmed a second write packet family:

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

## Firmware Version Fields

Manual display readout showed firmware values `J = U14` and `L = U20` on the tested unit. Current best field matches:

| UI field | Meaning | Packet byte | Published value |
| --- | --- | --- | --- |
| `J` | Power-board firmware version | `DD[39]` | `U<DD[39]>` |
| `L` | UI firmware version | `CC[66]` | `U<CC[66]>` |

These are identity-style fields rather than controllable settings. Cross-device captures are useful for extra confirmation.

## Implementation Implications

- Do not treat successful UART transmit as successful setting application.
- Confirm main writes against `D2`.
- Confirm extended writes against `D4`.
- Keep the packet family in pending TX state.
- Keep RX active during pending writes and short UI-sync windows.
- Avoid writing unrelated bytes from stale base packets.
- Treat fault/protection/status decoding as separate reverse-engineering work.
