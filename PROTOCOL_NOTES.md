# Daikin EKHHE Protocol Notes

Last updated: 2026-05-26

This note summarizes the current reverse-engineering findings from live UART/RS485 captures in `mode: debug`.
It is meant as a working protocol reference, not a final spec.

## Confidence levels

Some observations below are very strong, others are still hypotheses:

* Strong:
  * The steady-state bus has a repeating multi-packet cycle.
  * `D2` and `CC` are closely related state packets.
  * A UI-driven change causes `CD` to appear in the normal `D2 -> Cx` response slot.
  * UI-driven `CD` packets repeat across multiple cycles, not back-to-back within one cycle.
* Moderate:
  * `CC` looks like the UI's normal response to `D2`.
  * `CD` looks like the UI's "pending edit" response in the same slot.
  * The UI likely keeps sending `CD` until readback shows the requested value.
* Still open:
  * Whether all `Cx` packets are UI-originated and all `Dx` packets are control-board-originated.
  * Whether repeated `CD` packets within one burst are always byte-identical.
  * Which write failures are timing-related vs controller/state validation related.

## Observed steady-state cycle

With continuous RX enabled, the stable inter-packet pattern is:

1. `D4`
2. about `490-491 ms` later: `DD`
3. about `112-116 ms` later: `C1`
4. about `415-420 ms` later: `D2`
5. about `138-143 ms` later: `CC`
6. about `337-341 ms` later: next `D4`

So the practical idle loop is:

`D4 -> DD -> C1 -> D2 -> CC -> D4`

The full loop is roughly `1.49-1.50 s`.


## Why `CC` and `CD` matter

The component field maps already suggest `D2` and `CC` describe essentially the same logical state family.

On the wire:

* idle behavior: `... D2 -> CC ...`
* UI-active behavior: `... D2 -> CD ...`

That makes `CC` and `CD` look like alternate responses in the same slot.

## Physical UI behavior during writes

When the physical UI changes a parameter, observed behavior is:

* one `CD` appears per cycle
* `CD` lands in the normal `D2 -> CC` slot
* `CD` is not sent back-to-back as `CD -> CD -> CD`
* there is normal cycle traffic between repeated `CD`s

Typical UI-active pattern:

`... D2 -> CD -> D4 -> DD -> (sometimes C1) -> D2 -> CD ...`

When the burst ends, the bus returns to:

`... D2 -> CC ...`

## UI `CD` burst timing

Observed UI-generated `CD` timing:

* `D2 -> CD` is typically about `138-141 ms`
* this is effectively the same slot where idle traffic would carry `CC`

Observed burst length is not fixed. Captures showed bursts of about:

* `2` cycles
* `4` cycles
* `5` cycles

So the UI does not appear to send a single perfect `CD`, nor a fixed-count blind burst. The more likely model is:

* send one `CD` in the `D2 -> Cx` slot each cycle
* keep doing so while the edit is pending
* stop once normal readback state reflects the requested value

This "stop on readback" behavior is still inferred rather than directly proven, but it matches the variable burst
lengths well.

## Implications for ESPHome TX

The most useful protocol takeaway is:

* writing is not just "send a one-shot command whenever the bus is quiet"
* the real UI appears to occupy a specific response slot after `D2`
* a better mental model is "temporarily behave like the UI-side responder"

That means a write strategy should be thought of as:

1. observe `D2`
2. answer in the `D2 -> Cx` slot with `CD`
3. check a later `D2` for readback
4. repeat on later cycles until readback matches or a max-attempt rule is reached



## Practical current summary

Best current protocol model:

* control board emits `D2`
* UI normally responds with `CC`
* UI responds with `CD` instead of `CC` while a write is pending
* UI repeats one `CD` per cycle until the requested value comes back in readback state
* bus then falls back to normal `CC`

## Extended-block `C2` write behavior

Targeted protocol-lab captures later confirmed a second UI-side write packet:

* `C2` has start byte `0xC2`, length `51`, and the same checksum model as the
  other known packets.
* `C2` uses the same payload schema as `C1`.
* `C2` appears when the display/UI changes extended-block parameters that are
  read back in `D4`.
* `P53` and `P55-P72` are in this extended `D4`/`C1`/`C2` family.
* `P54` is the exception in that numeric range: it belongs to the main
  `D2`/`CC`/`CD` family.

The current write-family model is:

| Family | Control/readback packet | Idle UI packet | UI write packet | Payload size |
| --- | --- | --- | --- | ---: |
| Main block | `D2` | `CC` | `CD` | 71 |
| Extended block | `D4` | `C1` | `C2` | 51 |

## Firmware version fields

Manual UI readout showed firmware values `J = U14` and `L = U20`. Current best
matches from stable captured bytes are:

| UI field | Meaning | Packet byte | Published value |
| --- | --- | --- | --- |
| `J` | Power-board firmware version | `DD[39]` | `U<DD[39]>` |
| `L` | UI firmware version | `CC[66]` | `U<CC[66]>` |

These are stable identity-style fields rather than controllable parameters, so
cross-device captures remain useful for extra confirmation.

Observed targeted `C2` evidence:

* During a P53/P55 display edit, checksum-valid `C2` frames appeared.
* `C2[1]` tracked P53 and `C2[13]` tracked P55.
* The following `D4` echoed the changed extended values.
* Main-block `CD` frames can appear during the same edit window, but they do
  not carry the changed extended value.

## CD vs C2 timing baseline

The bus cycle is not perfectly fixed, and both of these idle orders have been
observed in clean captures:

* `D4 -> DD -> CC -> D2 -> C1`
* `D4 -> DD -> C1 -> D2 -> CC`

For implementation, the safer abstraction is therefore not "hard-code one
global packet order", but "replace/respond with the UI-side packet for the
field's packet family and confirm against the matching control-board packet".

Useful timing anchors from captures and ESPHome TX logs:

* Full cycle length is typically about `1.49-1.50 s`.
* The UI-side response slot after the relevant control packet is typically on
  the order of `138-155 ms`.
* Current ESPHome TX scheduling is D2-anchored and has been seen sending around
  `150 ms` after `D2` when the event loop fires cleanly.
* For current `CD` writes, confirmation is expected on a later `D2`, typically
  about one cycle after the send (`~1.4 s` in logs).
* For future `C2` writes, confirmation should be checked against `D4`; depending
  on the observed order, the first useful `D4` can arrive much sooner after the
  send than the next `D2`.

Implementation implication:

* Keep the existing D2-anchored send calibration for the first `C2` PR.
* Do not assume `CD` and `C2` share the same readback packet.
* Store the intended packet family in pending TX state so `CD` confirms against
  `D2` and `C2` confirms against `D4`.
* If hardware testing shows that `C2` needs a materially different send delay,
  add a separate extended TX calibration as a follow-up rather than guessing now.
