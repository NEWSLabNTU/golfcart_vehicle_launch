//! Encoder/decoder for the CAX_ADS_CAN protocol.
//!
//! Frame IDs and signal layouts mirror `CAX_ADS_CAN.dbc` at the repository
//! root. All multi-byte signals are little-endian. Multi-byte signed signals
//! use two's-complement.
//!
//! Checksum algorithm is a placeholder — Turing Drive has not provided the
//! spec. Every transmit frame currently writes `0x00` into the checksum byte.

pub const ID_ADS_VCU_MTR: u32 = 117;
pub const ID_ADS_VCU_BRK: u32 = 104;
pub const ID_ADS_VCU_EPS: u32 = 101;
pub const ID_ADS_VCU_VEHICLE: u32 = 1087;

pub const ID_VCU_ADS_BRK: u32 = 256;
pub const ID_VCU_ADS_MTR: u32 = 257;
pub const ID_VCU_ADS_EPS: u32 = 258;
pub const ID_VCU_ADS_VEHICLE: u32 = 259;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Gear {
    Parking = 0,
    Drive = 1,
    Neutral = 2,
    Reverse = 3,
}

impl Gear {
    pub fn from_raw(v: u8) -> Self {
        match v & 0x3 {
            1 => Gear::Drive,
            2 => Gear::Neutral,
            3 => Gear::Reverse,
            _ => Gear::Parking,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MotorMode {
    Pedal = 0,
    Speed = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BrakeMode {
    Invalid = 0,
    Stroke = 1,
    Pressure = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EpsMode {
    Invalid = 0,
    FrontWheel = 1,
    OppositePhase = 2,
    InPhase = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BlinkerCtrl {
    Off = 0,
    Left = 1,
    Right = 2,
    Hazard = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SubsystemState {
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

// ============================================================================
// TX commands (ADS → VCU)
// ============================================================================

#[derive(Debug, Clone, Copy)]
pub struct AdsVcuMtr {
    pub motor_en: bool,
    pub gear_en: bool,
    pub mode: MotorMode,
    pub gear: Gear,
    /// Throttle pedal position, 0..100 %. Used in pedal mode.
    pub throttle_pct: f32,
    /// Target longitudinal acceleration, 0..65.535 m/s².
    pub acceleration_mps2: f32,
    /// Target longitudinal speed, ±32.767 m/s. Used in speed mode.
    pub speed_mps: f32,
}

impl AdsVcuMtr {
    pub fn encode(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = (self.motor_en as u8)
            | ((self.gear_en as u8) << 1)
            | ((self.mode as u8) << 2)
            | (((self.gear as u8) & 0x3) << 3);
        buf[1] = scale_u8(self.throttle_pct, 0.4, 0.0, 255.0);
        let acc = scale_u16(self.acceleration_mps2, 0.001, 0.0, 65.535);
        buf[2..4].copy_from_slice(&acc.to_le_bytes());
        let spd = scale_i16(self.speed_mps, 0.001, -32.768, 32.767);
        buf[4..6].copy_from_slice(&spd.to_le_bytes());
        buf[7] = checksum_stub(ID_ADS_VCU_MTR, &buf[..7]);
        buf
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdsVcuBrk {
    pub brk_en: bool,
    pub mode: BrakeMode,
    pub stroke_mm: f32,
    pub pressure_mpa: f32,
    pub deceleration_mps2: f32,
    pub pedal_pct: u8,
}

impl AdsVcuBrk {
    pub fn encode(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = (self.brk_en as u8) | (((self.mode as u8) & 0x3) << 1);
        buf[1] = scale_u8(self.stroke_mm, 0.1, 0.0, 20.0);
        buf[2] = scale_u8(self.pressure_mpa, 0.05, 0.0, 12.75);
        buf[3] = scale_u8(self.deceleration_mps2, 0.05, 0.0, 12.75);
        buf[4] = self.pedal_pct;
        buf[7] = checksum_stub(ID_ADS_VCU_BRK, &buf[..7]);
        buf
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdsVcuEps {
    pub eps_en: bool,
    pub mode: EpsMode,
    /// Target tire angle in degrees, ±65.534.
    pub tire_angle_deg: f32,
    /// Target tire angular speed in deg/s, ±25.4.
    pub tire_ang_speed_dps: f32,
}

impl AdsVcuEps {
    pub fn encode(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = (self.eps_en as u8) | (((self.mode as u8) & 0x3) << 1);
        let ang = scale_i16(self.tire_angle_deg, 0.002, -65.536, 65.534);
        buf[1..3].copy_from_slice(&ang.to_le_bytes());
        buf[3] = scale_i8(self.tire_ang_speed_dps, 0.2, -25.6, 25.4) as u8;
        buf[7] = checksum_stub(ID_ADS_VCU_EPS, &buf[..7]);
        buf
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdsVcuVehicle {
    pub auto_en: bool,
    pub ads_status: u8, // 0 Error, 1 Running
    pub vin_req: bool,
    pub estop: bool,
    pub blinker: BlinkerCtrl,
    pub headlight: bool,
    pub turn_right_prompt: bool,
    pub turn_left_prompt: bool,
    pub backup_prompt: bool,
    pub auto_prompt: bool,
    pub door_ctrl: u8, // 0 invalid, 1 close, 2 open
    pub rolling_counter: u8,
}

impl AdsVcuVehicle {
    pub fn encode(&self) -> [u8; 8] {
        let mut buf = [0u8; 8];
        buf[0] = (self.auto_en as u8)
            | (((self.ads_status) & 0x3) << 1)
            | ((self.vin_req as u8) << 3)
            | ((self.estop as u8) << 4)
            | (((self.blinker as u8) & 0x3) << 5)
            | ((self.headlight as u8) << 7);
        buf[1] = (self.turn_right_prompt as u8)
            | ((self.turn_left_prompt as u8) << 1)
            | ((self.backup_prompt as u8) << 2)
            | ((self.auto_prompt as u8) << 3)
            | (((self.door_ctrl) & 0x3) << 4);
        buf[6] = self.rolling_counter;
        buf[7] = checksum_stub(ID_ADS_VCU_VEHICLE, &buf[..7]);
        buf
    }
}

// ============================================================================
// RX status (VCU → ADS)
// ============================================================================

#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)] // Fields exposed for future telemetry; not all consumed yet.
pub struct VcuAdsBrk {
    pub state: u8,
    pub position_pct: u8, // 255 = invalid
    pub stroke_mm: u8,    // 255 = invalid
    pub pressure_mpa: f32,
}

impl VcuAdsBrk {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        Some(Self {
            state: buf[0] & 0x3,
            position_pct: buf[1],
            stroke_mm: buf[2],
            pressure_mpa: buf[3] as f32 * 0.05,
        })
    }
}

#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
pub struct VcuAdsMtr {
    pub state: u8,
    pub throttle_pct: u8, // 255 = invalid
    pub gear: Gear,
    pub speed_mps: f32,
}

impl Default for Gear {
    fn default() -> Self {
        Gear::Parking
    }
}

impl VcuAdsMtr {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 5 {
            return None;
        }
        let speed_raw = i16::from_le_bytes([buf[3], buf[4]]);
        Some(Self {
            state: buf[0] & 0x3,
            throttle_pct: buf[1],
            gear: Gear::from_raw(buf[2]),
            speed_mps: speed_raw as f32 * 0.001,
        })
    }
}

#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
pub struct VcuAdsEps {
    pub state: u8,
    pub tire_angle_deg: f32,
    pub tire_ang_speed_dps: f32,
}

impl VcuAdsEps {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        let ang_raw = i16::from_le_bytes([buf[1], buf[2]]);
        let dps_raw = buf[3] as i8;
        Some(Self {
            state: buf[0] & 0x3,
            tire_angle_deg: ang_raw as f32 * 0.002,
            tire_ang_speed_dps: dps_raw as f32 * 0.2,
        })
    }
}

#[derive(Debug, Clone, Copy, Default)]
#[allow(dead_code)]
pub struct VcuAdsVehicle {
    pub rolling_counter: u8,
    pub driving_state: SubsystemState,
    pub estop: bool,
    pub blinker: u8, // 0 off, 1 L, 2 R, 3 hazard
    pub headlight: bool,
    pub doors: u8, // 0 closed, 1 open, 2 timeout
    pub err_sys: bool,
    pub err_mtr: bool,
    pub err_eps: bool,
    pub err_brk: bool,
}

impl Default for SubsystemState {
    fn default() -> Self {
        SubsystemState::Invalid
    }
}

impl VcuAdsVehicle {
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < 3 {
            return None;
        }
        let b1 = buf[1];
        let b2 = buf[2];
        Some(Self {
            rolling_counter: buf[0],
            driving_state: SubsystemState::from_raw(b1),
            estop: (b1 >> 2) & 0x1 != 0,
            blinker: (b1 >> 3) & 0x3,
            headlight: (b1 >> 5) & 0x1 != 0,
            doors: (b1 >> 6) & 0x3,
            err_sys: b2 & 0x1 != 0,
            err_mtr: (b2 >> 1) & 0x1 != 0,
            err_eps: (b2 >> 2) & 0x1 != 0,
            err_brk: (b2 >> 3) & 0x1 != 0,
        })
    }
}

// ============================================================================
// helpers
// ============================================================================

fn scale_u8(value: f32, scale: f32, min: f32, max: f32) -> u8 {
    let v = value.clamp(min, max);
    (v / scale).round().clamp(0.0, 255.0) as u8
}

fn scale_u16(value: f32, scale: f32, min: f32, max: f32) -> u16 {
    let v = value.clamp(min, max);
    (v / scale).round().clamp(0.0, 65535.0) as u16
}

fn scale_i16(value: f32, scale: f32, min: f32, max: f32) -> i16 {
    let v = value.clamp(min, max);
    (v / scale).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

fn scale_i8(value: f32, scale: f32, min: f32, max: f32) -> i8 {
    let v = value.clamp(min, max);
    (v / scale).round().clamp(i8::MIN as f32, i8::MAX as f32) as i8
}

/// Placeholder checksum — Turing Drive has not provided the algorithm.
/// Replace once the spec is available.
fn checksum_stub(_id: u32, _payload: &[u8]) -> u8 {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn motor_speed_mode_round_trips() {
        let cmd = AdsVcuMtr {
            motor_en: true,
            gear_en: true,
            mode: MotorMode::Speed,
            gear: Gear::Drive,
            throttle_pct: 0.0,
            acceleration_mps2: 1.234,
            speed_mps: -2.5,
        };
        let buf = cmd.encode();
        // flags: en=1, gear_en=1, mode=1, gear=1 -> 0b0000_1111
        assert_eq!(buf[0], 0b0000_1111);
        // throttle 0
        assert_eq!(buf[1], 0);
        // accel 1234 raw
        assert_eq!(u16::from_le_bytes([buf[2], buf[3]]), 1234);
        // speed -2500
        assert_eq!(i16::from_le_bytes([buf[4], buf[5]]), -2500);
    }

    #[test]
    fn eps_signed_angle() {
        let cmd = AdsVcuEps {
            eps_en: true,
            mode: EpsMode::FrontWheel,
            tire_angle_deg: -10.0,
            tire_ang_speed_dps: 5.0,
        };
        let buf = cmd.encode();
        assert_eq!(buf[0], 0b0000_0011);
        assert_eq!(i16::from_le_bytes([buf[1], buf[2]]), -5000);
        assert_eq!(buf[3] as i8, 25);
    }

    #[test]
    fn motor_status_decode() {
        // state=2 (Auto), throttle=42, gear=1 (D), speed=1.5 m/s -> 1500 raw
        let mut buf = [0u8; 8];
        buf[0] = 2;
        buf[1] = 42;
        buf[2] = 1;
        buf[3..5].copy_from_slice(&1500i16.to_le_bytes());
        let m = VcuAdsMtr::decode(&buf).unwrap();
        assert_eq!(m.state, 2);
        assert_eq!(m.throttle_pct, 42);
        assert_eq!(m.gear, Gear::Drive);
        assert!((m.speed_mps - 1.5).abs() < 1e-6);
    }
}
