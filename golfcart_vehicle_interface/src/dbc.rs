//! Generated DBC bindings + thin domain wrappers.
//!
//! The frame layout, bit-packing, and physical-unit scaling all come from
//! `dbc-codegen` parsing `CAX_ADS_CAN.dbc` at build time. We only define
//! Rust enums for fields we want to talk about by name (gear, blinker,
//! subsystem state) and a placeholder checksum.

#[allow(
    dead_code,
    clippy::all,
    non_camel_case_types,
    unused_imports,
    unused_qualifications,
    missing_docs
)]
pub mod messages {
    include!(concat!(env!("OUT_DIR"), "/dbc_messages.rs"));
}

// `mock_vcu` reuses this file via `mod dbc;` but only touches the VCU_ADS_*
// types, while the production binary uses the ADS_VCU_* side. Suppress the
// per-binary unused-import noise rather than splitting the re-export.
#[allow(unused_imports)]
pub use messages::{
    AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, Messages, VcuAdsBrk, VcuAdsEps, VcuAdsMtr,
    VcuAdsVehicle,
};

// ============================================================================
// DBC physical-range limits used to clamp Autoware setpoints before encode.
// Numeric values come from the vendor DBC and are emitted by `build.rs` into
// `$OUT_DIR/dbc_limits.rs` so they never appear in source. The constants
// here are the same names downstream code already depends on; only their
// definition has moved.
// ============================================================================

include!(concat!(env!("OUT_DIR"), "/dbc_limits.rs"));

// ============================================================================
// Domain enums (mapped to/from raw u8/bool fields in the generated structs).
// Encoded values match the DBC `VAL_` tables in CAX_ADS_CAN.dbc.
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Gear {
    #[default]
    Parking = 0,
    Drive = 1,
    Neutral = 2,
    Reverse = 3,
}

impl Gear {
    pub fn to_raw(self) -> u8 {
        self as u8
    }
    pub fn from_raw(v: u8) -> Self {
        match v & 0x3 {
            1 => Self::Drive,
            2 => Self::Neutral,
            3 => Self::Reverse,
            _ => Self::Parking,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum MotorMode {
    Pedal,
    Speed,
}

impl MotorMode {
    /// DBC `Ads_Vcu_Mtr_Mode` is a 1-bit field: 0=Pedal, 1=Speed.
    pub fn as_bool(self) -> bool {
        matches!(self, Self::Speed)
    }
}

impl Default for MotorMode {
    fn default() -> Self {
        Self::Speed
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum BrakeMode {
    Invalid = 0,
    Stroke = 1,
    Pressure = 2,
}

impl BrakeMode {
    pub fn to_raw(self) -> u8 {
        self as u8
    }
}

impl Default for BrakeMode {
    fn default() -> Self {
        Self::Pressure
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
#[allow(dead_code)]
pub enum EpsMode {
    Invalid = 0,
    FrontWheel = 1,
    OppositePhase = 2,
    InPhase = 3,
}

impl EpsMode {
    pub fn to_raw(self) -> u8 {
        self as u8
    }
}

impl Default for EpsMode {
    fn default() -> Self {
        Self::FrontWheel
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BlinkerCtrl {
    #[default]
    Off = 0,
    Left = 1,
    Right = 2,
    Hazard = 3,
}

impl BlinkerCtrl {
    pub fn to_raw(self) -> u8 {
        self as u8
    }
    #[allow(dead_code)]
    pub fn from_raw(v: u8) -> Self {
        match v & 0x3 {
            1 => Self::Left,
            2 => Self::Right,
            3 => Self::Hazard,
            _ => Self::Off,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SubsystemState {
    #[default]
    Invalid = 0,
    Manual = 1,
    Autonomous = 2,
    RemoteControl = 3,
}

impl SubsystemState {
    pub fn from_raw(v: u8) -> Self {
        match v & 0x3 {
            1 => Self::Manual,
            2 => Self::Autonomous,
            3 => Self::RemoteControl,
            _ => Self::Invalid,
        }
    }
}

/// Placeholder checksum — Turing Drive has not provided the algorithm.
/// Replace once the spec is available; every TX frame currently writes 0.
pub fn checksum_stub() -> u8 {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    // Tests stay structural: build via the generated constructors and
    // re-parse via the generated decoders. No raw-byte assertions — those
    // would re-encode the proprietary wire format in source.

    #[test]
    fn motor_speed_round_trips() {
        let speed = -2.5_f32;
        let accel = 1.234_f32;
        let mtr = AdsVcuMtr::new(
            true,
            true,
            MotorMode::Speed.as_bool(),
            Gear::Drive.to_raw(),
            0.0,
            accel,
            speed,
            checksum_stub(),
        )
        .unwrap();
        let parsed = AdsVcuMtr::try_from(mtr.raw().as_slice()).unwrap();
        assert!((parsed.ads_vcu_target_speed() - speed).abs() < 0.01);
        assert!((parsed.ads_vcu_target_acceleration() - accel).abs() < 0.01);
        assert_eq!(parsed.ads_vcu_target_gear_raw(), Gear::Drive.to_raw());
    }

    #[test]
    fn eps_signed_angle_round_trips() {
        let angle = -10.0_f32;
        let eps = AdsVcuEps::new(
            true,
            EpsMode::FrontWheel.to_raw(),
            angle,
            5.0,
            checksum_stub(),
        )
        .unwrap();
        let parsed = AdsVcuEps::try_from(eps.raw().as_slice()).unwrap();
        assert!((parsed.ads_vcu_target_tire_angle() - angle).abs() < 0.01);
    }

    #[test]
    fn motor_status_field_access() {
        // Round-trip a VCU_ADS_MTR via the generated constructor so we
        // exercise decode without hand-crafting a payload.
        let mtr = VcuAdsMtr::new(2, 42, Gear::Drive.to_raw(), 1.5).unwrap();
        let parsed = VcuAdsMtr::try_from(mtr.raw().as_slice()).unwrap();
        assert_eq!(parsed.vcu_ads_motor_state_raw(), 2);
        assert_eq!(parsed.vcu_ads_throttle_position_raw(), 42);
        assert_eq!(
            Gear::from_raw(parsed.vcu_ads_gear_position_raw()),
            Gear::Drive,
        );
        assert!((parsed.vcu_ads_vehicle_speed() - 1.5).abs() < 0.01);
    }
}
