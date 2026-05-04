//! ROS parameter loading.

use anyhow::{Context, Result};
use rclrs::Node;
use std::sync::Arc;

pub struct Params {
    /// SocketCAN interface name (e.g. `can0`). Mandatory, no default.
    pub can_interface: String,
    /// Frequency at which the four ADS_VCU_* command frames are transmitted.
    pub tx_rate_hz: f64,
    /// Frequency at which Autoware status reports are published.
    pub publish_rate_hz: f64,
    /// `frame_id` written into VelocityReport.header.
    pub frame_id: String,
    /// Maximum age of a Control message before TX trips into SafetyBrake.
    pub control_timeout_ms: u64,
    /// Maximum age of a VCU_ADS_VEHICLE frame before status is treated as
    /// stale. Drives diagnostics and engage refusal.
    pub report_timeout_ms: u64,
    /// Maximum forward speed setpoint allowed (m/s, unsigned magnitude).
    /// Reverse is also capped at the same magnitude. Useful for low-speed
    /// commissioning runs.
    pub max_speed_mps: f32,
    /// Maximum forward acceleration setpoint allowed (m/s², unsigned).
    pub max_accel_mps2: f32,
    /// Maximum brake deceleration setpoint allowed (m/s², unsigned).
    pub max_decel_mps2: f32,
    /// Maximum tire-angle setpoint allowed (rad, magnitude). Caps the
    /// front-wheel deflection regardless of what Autoware emits.
    pub max_tire_angle_rad: f32,
    /// Steering rate limit while stopped (rad/s). Prevents lock-jolt at v=0.
    pub steer_rate_stopped_rps: f32,
    /// Steering rate limit at low velocity (rad/s).
    pub steer_rate_low_vel_rps: f32,
    /// Steering rate limit at nominal velocity (rad/s).
    pub steer_rate_nominal_rps: f32,
    /// Speed boundary (m/s) below which `steer_rate_low_vel_rps` applies.
    pub steer_low_vel_thresh_mps: f32,
    /// Minimum dwell time (ms) between gear-cmd changes on CAN. Suppresses
    /// chatter from a planner that flips gear request fast.
    pub gear_change_margin_ms: u64,
    /// Brake pressure (MPa) commanded while a gear shift is queued at low
    /// speed. Forces the vehicle to settle before the gearbox engages.
    pub shift_brake_pressure_mpa: f32,
    /// |Speed| (m/s) below which gear shifts are allowed and brake-during-
    /// shift is asserted.
    pub shift_low_vel_thresh_mps: f32,
}

impl Params {
    pub fn from_node(node: &Node) -> Result<Self> {
        let can_interface = node
            .declare_parameter::<Arc<str>>("can_interface")
            .mandatory()
            .context("`can_interface` parameter is required (e.g. can0)")?
            .get()
            .to_string();

        let tx_rate_hz = node
            .declare_parameter("tx_rate_hz")
            .default(100.0)
            .mandatory()?
            .get();

        let publish_rate_hz = node
            .declare_parameter("publish_rate_hz")
            .default(50.0)
            .mandatory()?
            .get();

        let frame_id = node
            .declare_parameter::<Arc<str>>("frame_id")
            .default("base_link".into())
            .mandatory()?
            .get()
            .to_string();

        let control_timeout_ms = node
            .declare_parameter("control_timeout_ms")
            .default(500)
            .mandatory()?
            .get();

        let report_timeout_ms = node
            .declare_parameter("report_timeout_ms")
            .default(1000)
            .mandatory()?
            .get();

        let max_speed_mps = node
            .declare_parameter("max_speed_mps")
            .default(5.0)
            .mandatory()?
            .get();

        let max_accel_mps2 = node
            .declare_parameter("max_accel_mps2")
            .default(2.0)
            .mandatory()?
            .get();

        let max_decel_mps2 = node
            .declare_parameter("max_decel_mps2")
            .default(4.0)
            .mandatory()?
            .get();

        let max_tire_angle_rad = node
            .declare_parameter("max_tire_angle_rad")
            .default(0.349) // ≈ 20°, matches mechanical EPS limit
            .mandatory()?
            .get();

        let steer_rate_stopped_rps = node
            .declare_parameter("steer_rate_stopped_rps")
            .default(0.4)
            .mandatory()?
            .get();
        let steer_rate_low_vel_rps = node
            .declare_parameter("steer_rate_low_vel_rps")
            .default(0.4)
            .mandatory()?
            .get();
        let steer_rate_nominal_rps = node
            .declare_parameter("steer_rate_nominal_rps")
            .default(0.8)
            .mandatory()?
            .get();
        let steer_low_vel_thresh_mps = node
            .declare_parameter("steer_low_vel_thresh_mps")
            .default(1.0)
            .mandatory()?
            .get();

        let gear_change_margin_ms = node
            .declare_parameter("gear_change_margin_ms")
            .default(2000)
            .mandatory()?
            .get();
        let shift_brake_pressure_mpa = node
            .declare_parameter("shift_brake_pressure_mpa")
            .default(0.7)
            .mandatory()?
            .get();
        let shift_low_vel_thresh_mps = node
            .declare_parameter("shift_low_vel_thresh_mps")
            .default(0.1)
            .mandatory()?
            .get();

        Ok(Self {
            can_interface,
            tx_rate_hz,
            publish_rate_hz,
            frame_id,
            control_timeout_ms: control_timeout_ms as u64,
            report_timeout_ms: report_timeout_ms as u64,
            max_speed_mps: max_speed_mps as f32,
            max_accel_mps2: max_accel_mps2 as f32,
            max_decel_mps2: max_decel_mps2 as f32,
            max_tire_angle_rad: max_tire_angle_rad as f32,
            steer_rate_stopped_rps: steer_rate_stopped_rps as f32,
            steer_rate_low_vel_rps: steer_rate_low_vel_rps as f32,
            steer_rate_nominal_rps: steer_rate_nominal_rps as f32,
            steer_low_vel_thresh_mps: steer_low_vel_thresh_mps as f32,
            gear_change_margin_ms: gear_change_margin_ms as u64,
            shift_brake_pressure_mpa: shift_brake_pressure_mpa as f32,
            shift_low_vel_thresh_mps: shift_low_vel_thresh_mps as f32,
        })
    }
}
