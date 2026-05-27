# Operations

This page covers behavior that matters once the component is installed: polling, writes, retries, recovery snapshots, restore defaults, and troubleshooting.

## Polling

When idle, the component listens according to `update_interval`. A normal read cycle collects the known packet set and then publishes configured entities.

During writes, the component temporarily keeps RX active even if `continuous_rx` is disabled. This lets it observe readback confirmation and then wait briefly for the display-originated packet to catch up, reducing Home Assistant UI churn.

## Writes

Writes are confirmed from controller readback, not from transmit success alone. A setting write is considered applied only when the matching control-board packet reports the requested value.

The component uses two known write families:

| Setting family | Base packet | Transmit packet | Readback packet |
| --- | --- | --- | --- |
| Main settings | `CC` | `CD` | `D2` |
| Extended settings | `C1` | `C2` | `D4` |

The component schedules writes relative to the observed bus cycle and retries later cycles when readback does not match. The maximum retry count is currently 5 attempts. If a write succeeds only after more than one attempt, a warning is logged so timing or state-validation issues remain visible.

## Recovery Profiles

The component has two persistent profile slots in ESP flash:

| Profile | How it is saved | Intended use |
| --- | --- | --- |
| Known-good profile | Only when `daikin_save_known_good_profile` is pressed | Trusted manual recovery point |
| Auto snapshot | Automatically before normal single-field writes, with cooldown and diff checks | Recent pre-write undo point |

The auto snapshot is rate-limited to reduce flash writes. At the time of writing, it is stored at most once per 15 minutes and only when the managed fields differ from the stored auto snapshot.

Profile status text sensors report whether each slot is empty or valid:

- `daikin_known_good_profile_status`
- `daikin_auto_snapshot_status`

## Restore Defaults

The `daikin_restore_default_settings` button sends one managed restore packet rather than looping through individual fields.

Current restore-defaults scope:

- `P1-P52`
- `auto_target_temperature`
- `eco_target_temperature`
- `boost_target_temperature`
- `electric_target_temperature`

Runtime fields outside that scope, such as current operating mode, power state, clock values, and vacation days, are preserved from the latest base packet.

Extended settings `P53` and `P55-P72` are writable individually through the extended packet family, but they are not part of the current restore-defaults batch. `P54` is a main-family setting but is also outside the current restore-defaults batch because the documented batch scope is `P1-P52` plus target temperatures.

Only press the restore-defaults button when you intend to rewrite many installer parameters at once.

## Recommended Workflow

1. Flash a minimal configuration.
2. Confirm read-only sensors update normally.
3. Add the operating mode and target temperature controls.
4. Save a known-good profile after verifying the device configuration locally.
5. Add additional installer parameters only as needed.
6. After changing installer parameters, verify important settings on the physical display.

## Troubleshooting

### No values update

- Check RS485 `A`/`B` orientation.
- Check common ground.
- Check UART pins and baud settings.
- Confirm the original display still works.
- Try `mode: debug` with raw frame metadata enabled.

### Values update but writes fail

- Confirm the device is in a state where that setting is allowed to change.
- Confirm Home Assistant automations are not rapidly writing multiple settings.
- Expose `tx_send_calibration` temporarily and test nearby timing values.
- Use debug mode and check for `TX not applied` warnings.
- Start from the minimal example if the current YAML exposes many writable entities.

### Home Assistant briefly jumps back to old values

Some delay is normal. The component confirms controller readback first, then waits for the UI-side packet to reflect the applied value. If `update_interval` is long, UI refreshes may feel slower outside write windows.

### Restore buttons do nothing

- Confirm at least one valid base packet has been captured.
- Confirm no other write or restore is already active.
- Check the profile status text sensors for `VALID`.
- Enable debug logging and look for restore scheduling or confirmation warnings.
