# Changelog

All notable changes to `golfcart_vehicle_interface` are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

> ROS note: ament's native `catkin_generate_changelog` expects `CHANGELOG.rst`.
> This package documents in Markdown to stay consistent with `README.md` and the
> rest of the repository's docs.

## [Unreleased]

Aligns the TX path with the **ROOTS Drive-by-Wire 使用說明** vendor manual
(`2026-06-28 VCU manual`). The manual specifies that ROOTS actuates braking from
`Ads_Vcu_Target_Deceleration` only — stroke and pressure setpoints are ignored —
and that deceleration takes priority over speed (the motor cuts torque whenever
deceleration is commanded).

### Changed

- **Brake commands now use deceleration, not pressure.** ROOTS ignores
  `Ads_Vcu_Target_Stroke` / `Ads_Vcu_Target_Pressure`; both are now sent as `0`
  and braking is driven entirely by `Ads_Vcu_Target_Deceleration`
  (`src/can_io.rs`, `build_frames`).
- **Safety-brake** now commands `SAFETY_BRAKE_DECEL_MPS2 = 3.0 m/s²`
  (segment-3 ≈ 100 % per the manual's segmented brake map) instead of
  `4.0 MPa` of pressure. The `Ads_Vcu_Veh_Estop` bit is still asserted as a
  redundant max-deceleration path.
- **Gear-shift settle brake** now uses deceleration. Parameter
  `shift_brake_pressure_mpa` (0.7 MPa) renamed to `shift_brake_decel_mps2`
  (default `3.0 m/s²`); `GearShiftConfig.brake_pressure_mpa` →
  `brake_decel_mps2`. The previous pressure value did nothing on ROOTS, so the
  gearbox-settle brake was a no-op before this change.
- **Speed setpoint is now a non-negative magnitude.** `Ads_Vcu_Target_Speed` is
  specified for `[0, 25] km/h` with direction selected by the gear, so the
  subscriber now clamps `|velocity|` to
  `[0, min(max_speed_mps, 6.944, DBC max)]` (`ROOTS_MAX_SPEED_MPS = 6.944 m/s`
  ≈ 25 km/h) instead of passing a signed value in `[-max_speed, max_speed]`
  (`src/node.rs`).

### Fixed

- **Build against the current vendor DBC revision** (`Roots_can_test` drop,
  2026-07). The vendor renamed all messages with bus-direction prefixes
  (`RX_ADS_VCU_*` / `TX_VCU_ADS_*`, from the VCU's point of view), which made
  `dbc-codegen` emit `RxAdsVcu*` / `TxVcuAds*` types and broke the build.
  `build.rs` now strips the prefixes from `BO_` lines before codegen so the
  crate keeps its `AdsVcu*` / `VcuAds*` type names. Encoded frames verified
  byte-exact against a `cantools` encode of the same vendor DBC for all four
  TX messages.
- **Motor torque is cut while decelerating.** When a planner deceleration is
  active (`target_deceleration_mps2 > 0` while driving), the motor enable,
  throttle, acceleration, and speed fields are all zeroed so the motor and brake
  no longer fight on the bus — mirroring the VCU's internal decel-priority
  behaviour described in the manual. Gear-enable stays on so the VCU always
  knows the requested gear.

### Known limitations

See the [Known limitations](README.md#known-limitations) section of the README.
Carried forward and still open:

- `Veh_Chksum` algorithm not provided by the vendor — every TX frame sends `0`
  (`checksum_stub()`). Blocker for real-vehicle bring-up if the VCU validates it.
- Segmented brake dead-zone: deceleration requests below `1.2 m/s²` do not
  actuate the ROOTS brake at all, so gentle planner stops coast. Not yet
  remapped — needs a tuning decision.

## [0.1.0] - initial

- Rust (`rclrs`) CAN bridge for the Turing Drive `CAX_ADS_CAN.dbc` protocol:
  four `ADS_VCU_*` TX frames, four `VCU_ADS_*` RX frames, four-state TX FSM,
  control/report watchdogs, fault latch, diagnostics roll-up, socket reopen.
