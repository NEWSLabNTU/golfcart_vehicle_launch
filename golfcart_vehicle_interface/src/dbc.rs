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

/// `Vcu_Ads_Vehicle_Speed` is in km/h, whatever the DBC's "m/s" unit says
/// (confirmed on the cart 2026-10-28: the drive log stepped 0, 0.999, 1.999, ...
/// 5.000, whole units each 0.001 short, which is integer km/h). Every consumer
/// of the VCU speed goes through [`vcu_speed_mps`] so no site can forget it.
pub const VCU_SPEED_KMH_TO_MPS: f64 = 1.0 / 3.6;

/// The VCU floors its speed to whole km/h and sends each one 0.001 short
/// (0.999, 1.999, ...). A reading of k km/h therefore means a true speed in
/// [k, k + 1), mean k + 0.5: against NDT on 2026-10-28 the floored report read
/// 0.42 m/s in Drive and 0.28 m/s in Reverse where NDT measured 0.52 and 0.42,
/// and adding 0.5 km/h (0.14 m/s) lands both within 0.04 m/s. Without it the
/// EKF prediction ran slow and trailed NDT by about 0.2 m while moving.
///
/// Applied only to a reading that sits on a whole number (within 0.002), so it
/// switches itself off if the vendor ever sends finer steps. Zero stays zero: it
/// means anything under 1 km/h, which has no useful midpoint.
pub fn midpoint_of_floored_kmh(kmh: f64) -> f64 {
    let mag = kmh.abs();
    let whole = mag.round();
    if whole >= 1.0 && (mag - whole).abs() <= 0.002 {
        kmh.signum() * (whole + 0.5)
    } else {
        kmh
    }
}

/// The speed *setpoint* is in km/h too, whatever `Ads_Vcu_Target_Speed`'s "m/s"
/// label says. On 2026-10-28 the cart sat at 0 km/h for targets of 0.25 to 1.25
/// and began to creep, reading 0.999 km/h, only once 1.5 had been held for about
/// 3 s: a km/h number sent in what we thought was m/s. Everything we send goes
/// through [`vcu_target_speed_raw`], as everything we read goes through
/// [`vcu_speed_mps`].
pub const VCU_SPEED_MPS_TO_KMH: f64 = 3.6;

/// Planner speed in m/s as the number the VCU wants in `Ads_Vcu_Target_Speed`.
pub fn vcu_target_speed_raw(speed_mps: f32) -> f32 {
    (speed_mps as f64 * VCU_SPEED_MPS_TO_KMH) as f32
}

/// The VCU speed report in m/s, unsigned: the sign comes from the gear.
#[allow(dead_code)]
pub fn vcu_speed_mps(m: &VcuAdsMtr) -> f32 {
    (midpoint_of_floored_kmh(m.vcu_ads_vehicle_speed() as f64) * VCU_SPEED_KMH_TO_MPS) as f32
}

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
    fn vcu_speed_is_kmh_converted_to_mps() {
        // The cart's dashboard showed 5 km/h when the report read 5.000; the
        // floored reading stands for [5, 6) km/h, so the report is 5.5 km/h.
        let mtr = VcuAdsMtr::new(2, 0, Gear::Drive.to_raw(), 5.0).unwrap();
        let parsed = VcuAdsMtr::try_from(mtr.raw().as_slice()).unwrap();
        assert!((vcu_speed_mps(&parsed) - 5.5 / 3.6).abs() < 0.005);
        assert!((vcu_speed_mps(&parsed) - 1.528).abs() < 0.005);
    }

    #[test]
    fn the_setpoint_goes_out_in_kmh() {
        assert!((vcu_target_speed_raw(0.75) - 2.7).abs() < 1e-5);
        assert!((vcu_target_speed_raw(1.5) - 5.4).abs() < 1e-5);
        assert_eq!(vcu_target_speed_raw(0.0), 0.0);
    }

    #[test]
    fn floored_reading_gets_half_a_step_back() {
        // The 2026-10-28 drive log values: 0.999 and 1.999 are 1 and 2 km/h floored.
        assert!((midpoint_of_floored_kmh(0.999) - 1.5).abs() < 1e-9);
        assert!((midpoint_of_floored_kmh(1.999) - 2.5).abs() < 1e-9);
        assert!((midpoint_of_floored_kmh(-1.999) + 2.5).abs() < 1e-9);
    }

    #[test]
    fn zero_and_finer_readings_are_left_alone() {
        assert_eq!(midpoint_of_floored_kmh(0.0), 0.0);
        assert_eq!(midpoint_of_floored_kmh(0.4), 0.4); // under 1 km/h: no midpoint
        assert_eq!(midpoint_of_floored_kmh(2.37), 2.37); // not a whole step: the vendor sends finer values
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
