# Restore Default Settings Roadmap

Context:
- Target device: EKHHE-PCV37
- Scope to restore in one action:
  - `P1-P52`
  - mode target temperatures for `ECO`, `AUTOMATIC`, `BOOST`, `ELECTRIC`
- Explicit product decision:
  - include `P33-P46` even though the manual marks them "not usable for this device"
- Transport decision:
  - restore should be sent as a single `CD` packet built from the latest valid `CC`
  - do not loop through individual setting writes

Source of defaults:
- Manual: `EKHHE-PCV37_User, Installations and Maintenance Manual_English_2024.09.pdf`
- `P1-P52` defaults from pages 28-30
- mode target temperature defaults from page 23:
  - `ECO = 55`
  - `AUTOMATIC = 55`
  - `BOOST = 55`
  - `ELECTRIC = 55`

## Ticket 1: Button and Schema
- Add a new non-debug button entity:
  - `restore_default_settings`
- Keep existing debug snapshot buttons unchanged
- Make the new button available outside `mode: debug`

Acceptance:
- YAML can declare the new button in production or debug mode
- Existing debug buttons still work as before

## Ticket 2: Default Packet Builder
- Add a single internal default-value map for:
  - all requested byte-backed numbers
  - all requested signed values
  - all requested full-byte selects
  - all requested bitmask-backed selects
- Build a restore packet from `last_cc_packet_`
- Overwrite only the requested fields
- Preserve non-scope fields such as:
  - current power state
  - current operating mode
  - current clock values
  - vacation days

Acceptance:
- Component can construct one full `CD` payload containing all requested defaults

## Ticket 3: Batch Restore TX/Confirm State
- Add dedicated restore state separate from single-field `pending_tx_`
- Reuse current D2-anchored scheduling for sending
- Confirm success from subsequent `D2` readback by checking all restored fields
- Retry the same full packet up to the existing max repeat count
- Warn on failure or success after retries

Acceptance:
- Restore can complete or fail as one batch operation
- Normal single-setting writes still behave as before

## Ticket 4: Documentation and Validation
- Update README with:
  - new button
  - single-packet restore behavior
  - warning that it rewrites many installer settings
- Run config + compile validation

Acceptance:
- Docs reflect the new feature
- Build passes
