# rusEFI wideband: changes on top of upstream

High-level overview of what the `prod-2026-01` branch of
[rusefi/wideband](https://github.com/rusefi/wideband) carries on top of
[mck1117/wideband](https://github.com/mck1117/wideband).

- **Branch point:** `acd0463` "CAN RX: filter out not ours packets by WB_BL_HEADER (#343)", 2025-12-13
- **Branch head:** `27e115b`, 2026-01-24
- **Size:** 47 commits, 43 files changed (most of the line count is regenerated firmware image headers)
- Upstream `master` has moved 41 commits past the branch point; none of those are included here.

## CAN protocol (`for_rusefi/wideband_can.h`, `firmware/can.cpp`)

These changes affect the ECU side, since `wideband_can.h` is shared with rusEFI firmware.

- **Hardware ID addressing.** Each board reads its strap pins once at boot and remembers the result as a hardware ID (`BoardGetHwId()`). Commands are now addressed by this ID instead of the configurable CAN index, so a controller can still be reached after its index has been changed.
- **Bootloader entry (`WB_BL_ENTER`)** requires 1 data byte: `0xFF` (all) or the target hardware ID. The legacy zero-length form is no longer accepted.
- **Set index (`WB_MSG_SET_INDEX`)** requires 2 bytes: new index and target hardware ID. The legacy 1-byte form is no longer accepted.
- **New ping command (`WB_MSG_PING`, opcode 6).** Broadcast or addressed by hardware ID; the reply (`PongData`, extended ID) carries hardware ID, protocol version, and firmware build date.
- **New set-sensor-type command (`WB_MSG_SET_SENS_TYPE`, opcode 7).** Addressed by hardware ID; stores the sensor type in flash.
- **`SensorType` enum moved into `wideband_can.h`** (namespace `wbo`) so the ECU shares the definition.
- **New status codes:** `NotAllowed` (heating not allowed) and `CanSilent` (defined, not yet reported). `isStatusError()` now only treats the real sensor faults as errors, not these two.
- **Diagnostic frame reports `NotAllowed`** whenever the ECU has not allowed heating.
- **Lambda is always sent**, even when not valid; consumers must check the `Valid` flag. Previously an invalid lambda was sent as 0.
- **`Valid` byte carries sensor type flags:** bit 1 = LSU 4.9, bit 2 = FAE LSU 4.9.
- **ECU status timeout.** If no ECU status frame arrives for 250 ms, heater-allow falls back to `Unknown` and the remote battery voltage to 0.
- **Battery voltage from the ECU is trusted as-is.** The "below 5 V is bogus, assume 14 V" substitution is removed.
- `CanTxTyped` sets DLC from the payload size and supports extended IDs.

## Sensor support

- **FAE LSU 4.9 sensor type** (`SensorType::LSU49_FAE`). Uses the LSU 4.9 lambda curve, temperature table, heater targets, and ESR drive, with its own pump PID gains (kP 10 instead of 50).
- Pump PID configuration is now switchable at runtime, so a sensor type change takes effect without reboot.
- The F0 module now honors the sensor type stored in configuration (it was hardcoded to LSU 4.9).
- Sensor type is selectable in both TunerStudio ini files.

## Heater control

- **Supply voltage is validated even when the ECU allows heating.** This covers an ECU powered from USB with "force O2 heating" enabled and no battery voltage.
- **Hysteresis and stabilization timer:** between the off (8.5 V) and on (9.5 V) thresholds heating is not started but the stabilization timer is not reset; an explicit `NotAllowed` from the ECU resets it.
- **Supply voltage source is fixed per board** rather than switching on CAN state: boards with a heater input divider use the local measurement; boards without one rely on the ECU's voltage over CAN.
- **Standalone auto-start removed.** Boards without local voltage sensing no longer pretend to see 13 V after 5 seconds, so they will not heat without an ECU on the bus.
- Heater voltage filtering moved from the dual-board ports into common sampling code; `Battery` vs `Heater` naming cleaned up (`HEATER_SUPPLY_*`).
- **Green LED blinks very slowly (2 s)** while heating is not allowed.

## Configuration storage (`firmware/boards/port.h`)

- **Per-channel CAN settings** for up to 4 AFR and 4 EGT channels: rusEFI data and diagnostic frame enables, rusEFI index, AemNet enable and ID offset. The single `CanIndexOffset` field is retired.
- rusEFI data/diagnostic transmit enables and per-channel index are in effect. The AemNet and EGT fields are stored and exposed in the dual-channel TunerStudio ini, but the firmware on this branch does not read them yet.
- **Configuration tag bumped to `0xDEADBE02`**: existing stored settings are discarded and defaults loaded on first boot after update.
- `LoadDefaults()` is now common to all boards.
- **F0 module: strap pins only seed the default index.** Once a valid configuration is in flash, the stored index wins over the straps.
- Strap-pin reading extracted to `firmware/shared/strap_pin.cpp` for reuse; F1 boards also compute a hardware ID.
- `f1_dual_rev1`: `ID_SEL1` pin corrected from 13 to 12.

## Build, CI, and release

- ARM toolchain updated from 11.3.rel1 to 14.3.rel1.
- New workflow builds the F0 image and auto-commits the generated `for_rusefi/wideband_image.h` (commit step is still keyed to the `prod-2025-05` branch name).
- New manual workflow to bump the `libfirmware` submodule.
- Firmware and test workflows run on pushes to `master` and `prod-*` only, plus PRs and manual dispatch.
- Unit tests build with coverage on Linux and publish a gcovr HTML report as an artifact (not on macOS).
- F0 image header includes the build date and time; `build_defs.h` exposes the build date as integers.
- `USE_OPENBLT` and `TS_ENABLED` checks made safe for boards that do not define them.
- Release candidate image added under `release/rc/`; `for_rusefi/wideband_image.h` regenerated.
- Heater state machine tests updated for the stabilization timer.

## Tried and reverted

A `libfirmware` update and a `getTimeNowNt()` fix were both applied and then reverted; the submodule is at the same revision as upstream.

## Loose ends noticed during review

- `wideband_dual.ini` defines `RusEfiIdc0` and `AemNetIdx0`/`AemNetIdx1`, but dialogs and tooltips reference `RusEfiIdx0` and `AemNetIdOffset0`/`AemNetIdOffset1`.
- `wideband_dual.ini`: the EGT dialog is titled "CAN AFR Settings".
- `SendCanEgtForChannel()` is declared in `can.h` with no implementation on this branch.
- The set-sensor-type handler does not check that the hardware supports the requested sensor (marked TODO in code).
