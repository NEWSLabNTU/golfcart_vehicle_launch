//! ROS parameter loading.

use anyhow::{ensure, Context, Result};
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
    /// Minimum Control publish rate (Hz) required to command a speed. Below
    /// it the speed setpoint is held at 0 — a planner that updates a few times
    /// a second cannot steer a moving vehicle safely, and the gap between
    /// updates is long enough for the cart to travel blind. Steering and gear
    /// still pass through, and full silence is handled by
    /// `control_timeout_ms`. Set 0 to disable the check.
    pub control_min_rate_hz: f32,
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
    /// Commanded decelerations smaller than this (m/s², magnitude) are sent as
    /// no deceleration at all. Any `Target_Deceleration > 0` makes the ROOTS
    /// VCU cut motor torque, yet it does not brake below 1.2 m/s², and
    /// Autoware's longitudinal PID dithers a few tenths either side of zero at
    /// cruise - so without a deadband the motor toggles on and off. Inside the
    /// deadband the motor stays on and the speed-mode target speed does the
    /// slowing. 0 disables.
    pub decel_deadband_mps2: f32,
    /// Maximum tire-angle setpoint allowed (rad, magnitude). Caps the
    /// front-wheel deflection regardless of what Autoware emits.
    pub max_tire_angle_rad: f32,
    /// Flip the sign of the tire angle on the way to and from the VCU.
    ///
    /// Autoware follows REP-103: a positive `steering_tire_angle` turns left.
    /// ROOTS counts the other way - the vendor's own bench simulator maps its
    /// right-turn key to a *positive* `Ads_Vcu_Target_Tire_Angle`. Without the
    /// flip the cart steers the wrong way, and its reported angle disagrees
    /// with the command that produced it. Applied to TX setpoints and to the
    /// `SteeringReport` / actuation status decoded from RX, so both sides stay
    /// in Autoware's frame. Set false if a future VCU build changes convention.
    pub invert_steering: bool,
    /// Front-to-rear axle distance (m). Used only to derive
    /// `VelocityReport.heading_rate` from the steering report, since the VCU
    /// reports no yaw rate. The launch passes it from
    /// `golfcart_vehicle_description/config/vehicle_info.param.yaml`, the file
    /// the rest of Autoware reads; the default is that file's 2.061.
    pub wheel_base: f32,
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
    /// Brake deceleration (m/s²) commanded while a gear shift is queued at low
    /// speed. Forces the vehicle to settle before the gearbox engages. ROOTS
    /// brakes on `Ads_Vcu_Target_Deceleration` only (stroke/pressure are
    /// ignored), and braking is segmented — values below 1.2 m/s² do not
    /// actuate at all, so the default sits in segment 3 (≥2.8 ≈ full) to
    /// guarantee the cart is firmly held during the shift.
    pub shift_brake_decel_mps2: f32,
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

        let control_min_rate_hz = node
            .declare_parameter("control_min_rate_hz")
            .default(10.0)
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

        let decel_deadband_mps2: f64 = node
            .declare_parameter("decel_deadband_mps2")
            .default(0.2)
            .mandatory()?
            .get();
        // Negative would be meaningless; at or past 1.2 m/s² the deadband
        // would swallow the first brake stage and every gentle planner stop
        // with it.
        ensure!(
            decel_deadband_mps2.is_finite() && (0.0..1.2).contains(&decel_deadband_mps2),
            "`decel_deadband_mps2` must be in [0, 1.2) m/s², got {decel_deadband_mps2}"
        );

        let invert_steering = node
            .declare_parameter("invert_steering")
            .default(true)
            .mandatory()?
            .get();

        let wheel_base: f64 = node
            .declare_parameter("wheel_base")
            .default(2.061) // golfcart_vehicle_description vehicle_info.param.yaml
            .mandatory()?
            .get();
        // Divides the yaw-rate formula: zero or negative would publish inf or a
        // yaw rate of the wrong sign, which the EKF would then trust.
        ensure!(
            wheel_base.is_finite() && wheel_base > 0.0,
            "`wheel_base` must be a positive length in metres, got {wheel_base}"
        );

        let max_tire_angle_rad = node
            .declare_parameter("max_tire_angle_rad")
            // ~29.8 deg, just inside the VCU's ±30° (Ads_Vcu_Target_Tire_Angle).
            // Was 0.349 (20°), inherited from the PWM cart; the basement aisle
            // corners need ~30° (Phase 8, V3). Keep equal to max_steer_angle in
            // golfcart_vehicle_description's vehicle_info.param.yaml.
            .default(0.52)
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
        let shift_brake_decel_mps2 = node
            .declare_parameter("shift_brake_decel_mps2")
            .default(3.0)
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
            control_min_rate_hz: control_min_rate_hz as f32,
            report_timeout_ms: report_timeout_ms as u64,
            max_speed_mps: max_speed_mps as f32,
            max_accel_mps2: max_accel_mps2 as f32,
            max_decel_mps2: max_decel_mps2 as f32,
            decel_deadband_mps2: decel_deadband_mps2 as f32,
            max_tire_angle_rad: max_tire_angle_rad as f32,
            invert_steering,
            wheel_base: wheel_base as f32,
            steer_rate_stopped_rps: steer_rate_stopped_rps as f32,
            steer_rate_low_vel_rps: steer_rate_low_vel_rps as f32,
            steer_rate_nominal_rps: steer_rate_nominal_rps as f32,
            steer_low_vel_thresh_mps: steer_low_vel_thresh_mps as f32,
            gear_change_margin_ms: gear_change_margin_ms as u64,
            shift_brake_decel_mps2: shift_brake_decel_mps2 as f32,
            shift_low_vel_thresh_mps: shift_low_vel_thresh_mps as f32,
        })
    }
}
