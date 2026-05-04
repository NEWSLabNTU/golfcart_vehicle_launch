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
    /// Driver-side e-stop. Set by either:
    ///   * `~/input/emergency_stop` (`std_msgs::Bool`)
    ///   * `~/input/emergency_cmd`  (`tier4_vehicle_msgs::VehicleEmergencyStamped`)
    /// Trips `SafetyBrake` while held. Note: there is no watchdog on the
    /// e-stop topic — if the publisher dies while `estop=true`, the cart
    /// stays in `SafetyBrake` indefinitely, which is the desired fail-safe
    /// (stuck-on is safer than stuck-off). Recovery requires republishing
    /// `false` from a working publisher, or restarting the interface.
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
    /// Most recent gear actually transmitted on CAN. The TX path latches this
    /// to suppress chatter — incoming gear_cmd changes are deferred until
    /// `last_gear_change_at` is at least `gear_change_margin` old, matching
    /// pacmod's gear anti-chatter behaviour.
    pub last_gear_sent: Gear,
    pub last_gear_change_at: Option<Instant>,
    /// Most recent blinker actually transmitted on CAN. Diagnostics compares
    /// this against `Vcu_Ads_Blinker` to surface a stuck-blinker condition
    /// — common quirk on CAN-bus light controllers that latch on edge.
    pub last_blinker_sent: BlinkerCtrl,
    pub last_blinker_change_at: Option<Instant>,
    /// True while the CAN TX path is failing persistently. Set by the TX
    /// thread after `TX_FAIL_THRESHOLD` consecutive failed ticks; cleared
    /// once a tick succeeds. Surfaced as a diagnostic; combines with
    /// `fault_latched` to keep the cart safe-braked even if RX channel
    /// looks healthy.
    pub tx_failed: bool,
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
            last_gear_sent: Gear::Parking,
            last_gear_change_at: None,
            last_blinker_sent: BlinkerCtrl::Off,
            last_blinker_change_at: None,
            tx_failed: false,
        }
    }
}

/// Latest status decoded from VCU_ADS_* frames. The ROS publish timer reads
/// this and emits Autoware reports. Each frame is paired with the monotonic
/// `Instant` it was received so consumers can detect staleness (RX silently
/// dying or VCU going quiet leaves the cached value stale forever otherwise).
/// Each frame also carries a small ring buffer of arrival timestamps from
/// which a windowed receive rate is computed for diagnostics.
#[derive(Debug, Clone, Copy, Default)]
pub struct StatusState {
    pub mtr: Option<VcuAdsMtr>,
    pub mtr_at: Option<Instant>,
    pub mtr_freq: FreqWindow,
    pub eps: Option<VcuAdsEps>,
    pub eps_at: Option<Instant>,
    pub eps_freq: FreqWindow,
    pub brk: Option<VcuAdsBrk>,
    pub brk_at: Option<Instant>,
    pub brk_freq: FreqWindow,
    pub veh: Option<VcuAdsVehicle>,
    pub veh_at: Option<Instant>,
    pub veh_freq: FreqWindow,
    /// Count of CAN frames the dbc-codegen decoder rejected (unknown ID or
    /// payload that fails range checks). Surfaced in diagnostics so a
    /// transceiver dumping noise is distinguishable from a silent VCU.
    pub bad_frames: u64,
}

/// Rolling-buffer frequency tracker for one CAN frame ID. Holds up to
/// `FREQ_WINDOW_LEN` arrival timestamps. `record(now)` pushes; `rate_hz`
/// returns the windowed receive rate, or `None` until the buffer is filled.
pub const FREQ_WINDOW_LEN: usize = 16;

#[derive(Debug, Clone, Copy)]
pub struct FreqWindow {
    samples: [Option<Instant>; FREQ_WINDOW_LEN],
    head: usize,
    filled: bool,
}

impl Default for FreqWindow {
    fn default() -> Self {
        Self {
            samples: [None; FREQ_WINDOW_LEN],
            head: 0,
            filled: false,
        }
    }
}

impl FreqWindow {
    pub fn record(&mut self, now: Instant) {
        self.samples[self.head] = Some(now);
        self.head = (self.head + 1) % FREQ_WINDOW_LEN;
        if self.head == 0 {
            self.filled = true;
        }
    }

    pub fn rate_hz(&self) -> Option<f32> {
        if !self.filled {
            return None;
        }
        let oldest_idx = self.head; // next slot to overwrite = oldest sample
        let newest_idx = (self.head + FREQ_WINDOW_LEN - 1) % FREQ_WINDOW_LEN;
        match (self.samples[oldest_idx], self.samples[newest_idx]) {
            (Some(o), Some(n)) if n > o => {
                let dt = n.duration_since(o).as_secs_f32();
                if dt > 0.0 {
                    Some((FREQ_WINDOW_LEN as f32 - 1.0) / dt)
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

/// Shared state across the ROS executor, the CAN RX thread, and the CAN TX
/// thread.
///
/// Lock-ordering convention (must hold to avoid deadlock):
///
///   1. `command` first
///   2. `status` second
///
/// All paths that need both must acquire `command` before `status`. Paths that
/// need only one are unconstrained. Current callers:
///   * `can_io::tx_loop`            — `command` only
///   * `can_io::rx_loop`            — `status` only (write per frame)
///   * `can_io::handle_vehicle_status` — `command` only (after `status`
///     guard already released)
///   * `node::publish_status`       — `status` then `command` is *not* used;
///     reads them via separate temporary locks (each released before the
///     next), so no nesting occurs.
///   * `node::publish_diagnostics`  — same as above.
///   * `node` engage service        — `command` first, `status` second
///     (matches the convention).
///
/// New code that touches both must acquire `command` first. Snapshot+release
/// pattern (deref the guard into a `Copy` value, drop it, then take the next
/// lock) keeps critical sections short and avoids accidental nesting.
#[derive(Default)]
pub struct SharedState {
    pub command: Mutex<CommandState>,
    pub status: Mutex<StatusState>,
}
