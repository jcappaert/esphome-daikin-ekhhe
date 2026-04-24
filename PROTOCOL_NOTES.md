# Daikin EKHHE Protocol Notes

Last updated: 2026-04-24

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


