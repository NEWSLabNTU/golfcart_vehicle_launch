//! Shared state between ROS callbacks and the CAN I/O threads.

use parking_lot::Mutex;
use std::time::Instant;

use crate::dbc::{
    BlinkerCtrl, BrakeMode, EpsMode, Gear, MotorMode, VcuAdsBrk, VcuAdsEps, VcuAdsMtr,
    VcuAdsVehicle,
};

/// Latest commands written by ROS subscribers/services. The TX thread reads
/// this on every period and packs the four ADS_VCU_* frames.
#[derive(Debug, Clone, Copy)]
pub struct CommandState {
    // Motor / brake / EPS enables follow the autonomous-mode toggle.
    pub auto_enabled: bool,

    pub motor_mode: MotorMode,
    pub gear: Gear,
    pub target_speed_mps: f32,
    pub target_acceleration_mps2: f32,
    pub target_throttle_pct: f32,

    pub brake_mode: BrakeMode,
    pub target_deceleration_mps2: f32,

    pub eps_mode: EpsMode,
    pub target_tire_angle_rad: f32,

    pub blinker: BlinkerCtrl,
    pub headlight: bool,
    /// Driver-side e-stop latched in software (set via future
    /// `~/input/emergency_stop` topic; currently only the ECU-side e-stop is
    /// handled, see `fault_latched`).
    pub estop: bool,
    /// Sticky latch raised when the ECU reports any hazard
    /// (`Vcu_Ads_Estop` or any of the four `Vcu_Ads_Error_Code_*` bits).
    /// While set, the TX loop disengages and commands a safety brake; the
    /// engage service rejects mode=AUTONOMOUS until the user sends a
    /// MANUAL/NO_COMMAND request to clear it.
    pub fault_latched: bool,
    /// Last time a Control message was received. Commands are only sent while
    /// recent — protects the vehicle from a stalled planner.
    pub last_control_at: Option<Instant>,
}

impl Default for CommandState {
    fn default() -> Self {
        Self {
            auto_enabled: false,
            motor_mode: MotorMode::Speed,
            gear: Gear::Parking,
            target_speed_mps: 0.0,
            target_acceleration_mps2: 0.0,
            target_throttle_pct: 0.0,
            brake_mode: BrakeMode::Pressure,
            target_deceleration_mps2: 0.0,
            eps_mode: EpsMode::FrontWheel,
            target_tire_angle_rad: 0.0,
            blinker: BlinkerCtrl::Off,
            headlight: false,
            estop: false,
            fault_latched: false,
            last_control_at: None,
        }
    }
}

/// Latest status decoded from VCU_ADS_* frames. The ROS publish timer reads
/// this and emits Autoware reports.
#[derive(Debug, Clone, Copy, Default)]
pub struct StatusState {
    pub mtr: Option<VcuAdsMtr>,
    pub eps: Option<VcuAdsEps>,
    pub brk: Option<VcuAdsBrk>,
    pub veh: Option<VcuAdsVehicle>,
}

#[derive(Default)]
pub struct SharedState {
    pub command: Mutex<CommandState>,
    pub status: Mutex<StatusState>,
}
