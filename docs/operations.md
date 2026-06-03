# Operations

This page covers behavior that matters once the component is installed: polling, writes, retries, recovery snapshots, restore defaults, and troubleshooting.

## Polling

When idle, the component listens according to `update_interval`. A normal read cycle collects the known bus state and then publishes configured entities.

During writes, the component temporarily keeps RX active even if `continuous_rx` is disabled. This lets it observe readback confirmation and then wait briefly for the display state to catch up, reducing Home Assistant UI churn.

## Writes

Writes are confirmed from controller readback, not from transmit success alone. A setting write is considered applied only when the heat pump reports the requested value back.

The bus cycle is roughly 1.5 seconds. In normal conditions a write should usually be confirmed within one cycle, so expect a setting to settle in about 1 to 3 seconds. The component keeps listening during this window, even if the normal idle `update_interval` is longer.

If the first attempt is not reflected in readback, the component retries on later cycles. The current retry limit is 5 attempts, so a write failure may take roughly 7 to 10 seconds to become final. A successful write that needed more than one attempt logs a warning; a write that never applies logs `TX not applied`.

After controller readback confirms the new value, Home Assistant may still briefly show the old value until the display-side state catches up. The component keeps RX active for a short UI-sync window to suppress most of that churn. Treat the value as truly settled once it remains stable after the next visible refresh, or after roughly another 1 to 2 seconds.

## Multiple Writes And Automations

The component is designed around one active write at a time. If Home Assistant sends several changes at once, the first write starts and later writes may be ignored or delayed depending on timing. This can happen when an automation updates operating mode and target temperature together, or when a dashboard sends several entity changes in quick succession.

Recommended Home Assistant patterns:

- Prefer changing one heat-pump entity at a time.
- In scripts or automations, add a short delay between writes.
- Use at least 2 to 3 seconds between ordinary setting changes.
- Use 8 to 10 seconds between changes when testing timing-sensitive settings or after a retry warning.
- Avoid periodic automations that rewrite unchanged values.
- If an automation controls multiple values, check current state first and write only values that actually need changing.

If you need a robust multi-step automation, make it state-driven: send one change, wait until the entity reports the requested state, then send the next change.

## Recovery Profiles

The component has two persistent profile slots in ESP flash:

| Profile | How it is saved | Intended use |
| --- | --- | --- |
| Known-good profile | Only when `daikin_save_known_good_profile` is pressed | Trusted manual recovery point |
| Auto snapshot | Automatically before normal single-field writes, with cooldown and diff checks | Recent pre-write undo point |

The auto snapshot is rate-limited to reduce flash writes. At the time of writing, it is stored at most once per 15 minutes and only when the managed fields differ from the stored auto snapshot.

Each profile is one logical recovery point for the supported managed settings. Internally, saving a profile requires a recent complete read of both settings areas. If either area has not been captured yet, saving is skipped instead of storing a partial recovery point.

Profile status text sensors report whether each slot is empty or valid:

- `daikin_known_good_profile_status`
- `daikin_auto_snapshot_status`

A valid profile status reports both stored lengths, for example `VALID main=71 extended=51`. If a previously saved profile was created by an older component version, it may show `EMPTY`; save a fresh known-good profile after verifying the device settings.

Restoring a profile is a broad operation. The component restores the supported managed settings in two internal stages and reports success only after both stages are confirmed by device readback. If only one stage applies, the restore logs a partial-failure warning so you can retry or verify settings on the physical display.

## Restore Defaults

The `daikin_restore_default_settings` button restores documented datasheet/manual defaults for the supported settings. It uses managed batch writes rather than looping through individual fields one by one.

Current restore-defaults scope:

- `P1-P72`
- `auto_target_temperature`
- `eco_target_temperature`
- `boost_target_temperature`
- `electric_target_temperature`

Runtime fields outside that scope, such as current operating mode, power state, clock values, and vacation days, are preserved from the latest observed device state.

Like profile restore, restore-defaults is applied in two internal stages and is considered successful only when the device reports all supported defaults back. A partial failure means some settings may have changed while others did not, so check the warning logs and verify critical parameters on the display before continuing.

Only press the restore-defaults button when you intend to rewrite many installer parameters at once.

## Fault Indicators

The optional fault/alarm binary sensors are read-only indicators decoded from
the unit status frames. They are intended to make Home Assistant dashboards and
automations aware of confirmed display states; they do not replace the Daikin
display, installer manual, or normal troubleshooting procedure.

Confirmed indicators currently include `P01` through `P08`, `E01`, `E02`,
`E03`, and `PA`. During inlet-air probe failures, the unit may briefly show
`PA` before settling into `P04`; exposing both sensors lets Home Assistant show
that transition instead of hiding it.

`E04` and `E08` are intentionally not included at this time. They are not just
missing from the entity list; they have not been reproduced with a reliable
bus-visible status bit. In particular, `E04` may depend on hardware/manual
variant behavior, and `E08` may be local to the display when communication is
lost.

## Log Expectations

At the default `INFO` level, normal operation should be fairly quiet. You should see successful write and restore messages, plus Home Assistant state updates from ESPHome itself.

At `WARN` level, pay attention to:

- `TX not applied`: the device did not report the requested value after all retry attempts.
- Write applied after more than one attempt: the change worked, but timing or state conditions were not ideal.
- Restore/profile warnings: the restore was blocked, missing recent source data, missing a stored profile, failed one internal stage, or failed confirmation.

At `DEBUG` level, expect more operational timing detail:

- TX scheduling and confirmation timing.

For ordinary use, `INFO` is usually enough. Use `DEBUG` while tuning `tx_send_calibration` or diagnosing write failures.

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
- Temporarily set `continuous_rx: true` if you need to confirm the node can keep up with every bus cycle.

### Values update but writes fail

- Confirm the device is in a state where that setting is allowed to change.
- Confirm Home Assistant automations are not rapidly writing multiple settings.
- Increase automation delays and try again.
- Expose `tx_send_calibration` temporarily and test nearby timing values.
- Set the logger to `DEBUG` and check for `TX not applied` warnings.
- Start from the minimal example if the current YAML exposes many writable entities.

### Home Assistant briefly jumps back to old values

Some delay is normal. The component confirms controller readback first, then waits for the display-side state to reflect the applied value. If `update_interval` is long, UI refreshes may feel slower outside write windows.

### Restore buttons do nothing

- Confirm the device has completed at least one normal read cycle since boot.
- Confirm no other write or restore is already active.
- Check the profile status text sensors for `VALID`.
- Enable debug logging and look for restore scheduling or confirmation warnings.

### Restore reports a partial failure

- Wait for the next normal refresh and check whether the setting values are stable.
- Retry once if the bus was busy or Home Assistant sent other writes at the same time.
- If the same restore stage fails repeatedly, expose `tx_send_calibration` and test nearby timing values.
- Verify critical installer parameters on the physical display before relying on the restored profile/defaults.
