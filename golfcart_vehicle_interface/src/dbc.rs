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

pub use messages::{
    AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, CanError, Messages, VcuAdsBrk, VcuAdsEps,
    VcuAdsMtr, VcuAdsVehicle,
};

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

    #[test]
    fn motor_speed_mode_round_trips() {
        let mtr = AdsVcuMtr::new(
            true,                 // motor_en
            true,                 // gear_en
            MotorMode::Speed.as_bool(),
            Gear::Drive.to_raw(), // target_gear
            0.0,                  // throttle %
            1.234,                // accel m/s²
            -2.5,                 // speed m/s
            checksum_stub(),
        )
        .unwrap();
        let buf = mtr.raw();
        // byte 0: motor_en | gear_en<<1 | mode<<2 | gear<<3 = 0b0000_1111
        assert_eq!(buf[0], 0b0000_1111);
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 1234);
        assert_eq!(i16::from_le_bytes([buf[4], buf[5]]), -2500);
    }

    #[test]
    fn eps_signed_angle() {
        let eps = AdsVcuEps::new(
            true,
            EpsMode::FrontWheel.to_raw(),
            -10.0, // tire angle deg
            5.0,   // tire angular speed deg/s
            checksum_stub(),
        )
        .unwrap();
        let buf = eps.raw();
        assert_eq!(buf[0], 0b0000_0011);
        assert_eq!(i16::from_le_bytes([buf[1], buf[2]]), -5000);
        assert_eq!(buf[3] as i8, 25);
    }

    #[test]
    fn motor_status_decode() {
        // state=2(Auto), throttle=42, gear=1(D), speed=1.5 m/s -> 1500 raw
        let mut buf = [0u8; 8];
        buf[0] = 2;
        buf[1] = 42;
        buf[2] = 1;
        buf[3..5].copy_from_slice(&1500i16.to_le_bytes());
        let m = VcuAdsMtr::try_from(buf.as_slice()).unwrap();
        assert_eq!(m.vcu_ads_motor_state_raw(), 2);
        assert_eq!(m.vcu_ads_throttle_position_raw(), 42);
        assert_eq!(Gear::from_raw(m.vcu_ads_gear_position_raw()), Gear::Drive);
        assert!((m.vcu_ads_vehicle_speed() - 1.5).abs() < 1e-6);
    }
}
