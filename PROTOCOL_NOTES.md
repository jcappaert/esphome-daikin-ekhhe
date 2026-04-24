# Daikin EKHHE Protocol Notes

Last updated: 2026-04-24

This note summarizes the current reverse-engineering findings from live UART/RS485 captures in `mode: debug`.
It is meant as a working protocol reference, not a final spec.

## Confidence levels

Some observations below are very strong, others are still hypotheses:

* Strong:
  * The steady-state bus has a repeating multi-packet cycle, not a simple 10 second poll.
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

Important note:

* Earlier "10 second cycle" conclusions were mostly an artifact of non-continuous polling.
* Once RX was kept active continuously, the real repeating cycle became visible.

## Packet ownership model

Current best model:

* `D2` is a control-board state/update packet.
* `CC` is the normal UI-side response in the `D2 -> Cx` slot.
* `CD` is the UI-side "edited state" response in that same slot when a setting change is pending.

This is strongest for the `D2 / CC / CD` family. It is not yet proven as a universal rule for every `C*` and `D*`
packet type.

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

## Important caveat about timing experiments

Several timing experiments were run in the component:

* `DD`-anchored scheduling
* `D2`-anchored scheduling
* various nominal offsets after `D2`

One important bug was found along the way:

* the code originally checked current-cycle `CC` presence using `latest_packets_`
* but `latest_packets_` was cleared on cycle reset
* that allowed "late" sends to happen even after `CC` had already arrived

That bug explained why some earlier timings appeared to work even though they were actually transmitting after the
intended slot.

## CRC / framing observation

A recurring checksum error that appeared in earlier non-continuous captures largely disappeared once continuous RX was
enabled.

Current interpretation:

* that earlier checksum issue was more likely a synchronization / partial-cycle capture artifact
* it does not currently look like a stable, always-present bad frame in the steady-state protocol

## Open questions

Still worth investigating:

* whether repeated `CD` packets in one UI burst are byte-identical
* whether some `operational_mode` values are controller-state dependent even when timing is correct
* whether ESPHome should imitate the UI more closely by sending one `CD` per cycle until readback matches
* whether the UI also changes any non-`CD` traffic during a write burst

## Practical current summary

Best current protocol model:

* control board emits `D2`
* UI normally responds with `CC`
* UI responds with `CD` instead of `CC` while a write is pending
* UI repeats one `CD` per cycle until the requested value comes back in readback state
* bus then falls back to normal `CC`

That model has held up better than any "single magic transmit timestamp" theory so far.
