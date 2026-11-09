//! Shared state between ROS callbacks and the CAN I/O threads.

use parking_lot::Mutex;
use std::time::{Duration, Instant};

use crate::dbc::{
    BlinkerCtrl, BrakeMode, EpsMode, Gear, MotorMode, SubsystemState, VcuAdsBrk, VcuAdsEps,
    VcuAdsMtr, VcuAdsVehicle,
};

/// Aggregate control mode of the vehicle, derived from the four subsystem
/// states the VCU reports (`VCU_ADS_MTR.Motor_State`, `VCU_ADS_BRK.Brake_State`,
/// `VCU_ADS_EPS.EPS_State`, `VCU_ADS_VEHICLE.Driving_State`).
///
/// The driver — not this node — performs every mode transition, on the
/// vehicle's own controls. We only observe the result: all four subsystems
/// agreeing is the only state either side of the handover is well defined in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VehicleMode {
    /// All four subsystems report `Manual`: the driver has the vehicle.
    Manual,
    /// All four subsystems report `Autonomous`: the ADS may command.
    Autonomous,
    /// Anything else — mixed states, any `Invalid` / `Remote Control`, or a
    /// missing/stale report frame. No commands are transmitted.
    #[default]
    Abnormal,
}

/// Who asked for an e-stop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstopSource {
    /// `~/input/emergency_stop` (`std_msgs::Bool`): the driver's button or a
    /// hand-rolled caller.
    Driver,
    /// `~/input/emergency_cmd` (`tier4_vehicle_msgs::VehicleEmergencyStamped`):
    /// Autoware's `vehicle_cmd_gate`.
    Mrm,
}

/// Requested e-stops, kept per source. The effective e-stop is their OR.
///
/// The two sources must not share one flag. `vehicle_cmd_gate` publishes
/// `emergency_cmd` with `emergency=false` on every control cycle, so with a
/// shared flag a driver's press was cleared by the gate's next routine message,
/// within one cycle. Each topic now writes, and can release, only its own flag.
///
/// There is no watchdog on either topic: a publisher that dies while holding
/// `true` leaves the cart in `SafetyBrake`, which is the intended fail-safe
/// (stuck-on is safer than stuck-off). Recovery is a `false` from a working
/// publisher on that same topic, or restarting the interface.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct EstopState {
    driver: bool,
    mrm: bool,
}

impl EstopState {
    /// Record `source`'s latest request. Returns `Some(engaged)` when that
    /// source's own flag changed, so the caller logs transitions, not repeats.
    pub fn set(&mut self, source: EstopSource, engaged: bool) -> Option<bool> {
        let flag = match source {
            EstopSource::Driver => &mut self.driver,
            EstopSource::Mrm => &mut self.mrm,
        };
        let changed = *flag != engaged;
        *flag = engaged;
        changed.then_some(engaged)
    }

    pub fn driver(&self) -> bool {
        self.driver
    }

    pub fn mrm(&self) -> bool {
        self.mrm
    }

    /// True while any source holds the e-stop.
    pub fn active(&self) -> bool {
        self.driver || self.mrm
    }
}

/// Latest commands written by ROS subscribers/services. The TX thread reads
/// this on every period and packs the four ADS_VCU_* frames.
#[derive(Debug, Clone, Copy)]
pub struct CommandState {
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
    /// Requested e-stops, one flag per source (see `EstopState`). Trips
    /// `SafetyBrake` while any source holds it.
    pub estop: EstopState,
    /// Sticky latch raised when the ECU reports any hazard
    /// (`Vcu_Ads_Estop` or any of the four `Vcu_Ads_Error_Code_*` bits).
    /// While set (or while `tx_fault_latched` is), the TX loop commands a
    /// safety brake (as long as the vehicle is in `Autonomous`) and the
    /// control-mode report reads `DISENGAGED`. Cleared by a MANUAL/NO_COMMAND
    /// request, or on its own once the vehicle has been in `Manual` with no
    /// hazard bit for `fault_clear_hold_ms` (see `update_hazard_latch`).
    pub hazard_latched: bool,
    /// Last time a VCU_ADS_VEHICLE frame carried a hazard bit. The auto-clear
    /// hold is measured from here, so a hazard still asserted keeps it latched.
    pub last_hazard_at: Option<Instant>,
    /// Sticky latch raised by persistent CAN TX failure (`TX_FAIL_THRESHOLD`
    /// consecutive failed ticks). Same effect as `hazard_latched`, but never
    /// cleared automatically: only a MANUAL/NO_COMMAND request clears it.
    pub tx_fault_latched: bool,
    /// Last time a Control message was received. Commands are only sent while
    /// recent — protects the vehicle from a stalled planner.
    pub last_control_at: Option<Instant>,
    /// Arrival times of the last `FREQ_WINDOW_LEN` Control messages. The TX
    /// path zeroes the speed setpoint when Autoware publishes too slowly to
    /// steer a moving vehicle safely — see `control_min_rate_hz`. A publisher
    /// that stops entirely is the `control_timeout_ms` watchdog's business.
    pub control_freq: FreqWindow,
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

/// What `CommandState::update_hazard_latch` did, so the caller logs edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HazardLatchEvent {
    /// Nothing changed.
    Unchanged,
    /// A hazard bit raised the latch (rising edge only).
    Latched,
    /// The latch cleared itself: vehicle in Manual, no hazard for the hold.
    AutoCleared,
}

impl CommandState {
    /// Either latch is set: the TX loop safety-brakes in Autonomous, the
    /// control-mode report reads DISENGAGED and AUTONOMOUS is refused.
    pub fn fault_latched(&self) -> bool {
        self.hazard_latched || self.tx_fault_latched
    }

    /// Clear both latches — a MANUAL/NO_COMMAND request.
    pub fn clear_fault_latches(&mut self) {
        self.hazard_latched = false;
        self.tx_fault_latched = false;
    }

    /// Feed one VCU_ADS_VEHICLE frame's hazard summary into the ECU-hazard
    /// latch.
    ///
    /// A hazard bit latches at once. The latch clears itself only when the
    /// vehicle is in `Manual` (all four subsystems, fresh frames) and no
    /// hazard bit has been seen for `clear_hold`. On 2026-11-09 the VCU set
    /// `Vcu_Ads_Error_Code_Mtr` for a frame or a few 1-3 s after every
    /// dashboard AUTO -> Manual, and the latch then refused the next AUTO
    /// until someone sent MANUAL by hand. In Manual we command nothing, so
    /// the latch protects nothing there; entering AUTO is the driver's own
    /// act, and a hazard still present then re-latches on the next frame.
    /// `Autonomous` and `Abnormal` never clear. A zero `clear_hold`
    /// disables the auto-clear. `tx_fault_latched` is not touched.
    pub fn update_hazard_latch(
        &mut self,
        hazard: bool,
        vehicle_mode: VehicleMode,
        now: Instant,
        clear_hold: Duration,
    ) -> HazardLatchEvent {
        if hazard {
            self.last_hazard_at = Some(now);
            if self.hazard_latched {
                return HazardLatchEvent::Unchanged;
            }
            self.hazard_latched = true;
            return HazardLatchEvent::Latched;
        }
        if !self.hazard_latched
            || clear_hold.is_zero()
            || vehicle_mode != VehicleMode::Manual
        {
            return HazardLatchEvent::Unchanged;
        }
        let quiet = self
            .last_hazard_at
            .map_or(true, |t| now.saturating_duration_since(t) >= clear_hold);
        if !quiet {
            return HazardLatchEvent::Unchanged;
        }
        self.hazard_latched = false;
        HazardLatchEvent::AutoCleared
    }
}

impl Default for CommandState {
    fn default() -> Self {
        Self {
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
            estop: EstopState::default(),
            hazard_latched: false,
            last_hazard_at: None,
            tx_fault_latched: false,
            last_control_at: None,
            control_freq: FreqWindow::default(),
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

impl StatusState {
    /// The four reported subsystem states, in `[MTR, BRK, EPS, Drv]` order.
    /// A frame that is missing or older than `report_timeout` yields `None` —
    /// a cached state from a VCU that has gone quiet says nothing about who
    /// holds the vehicle right now.
    pub fn subsystem_states(&self, report_timeout: Duration) -> [Option<SubsystemState>; 4] {
        let fresh = |at: Option<Instant>| at.map_or(false, |t| t.elapsed() <= report_timeout);
        [
            self.mtr
                .filter(|_| fresh(self.mtr_at))
                .map(|m| SubsystemState::from_raw(m.vcu_ads_motor_state_raw())),
            self.brk
                .filter(|_| fresh(self.brk_at))
                .map(|b| SubsystemState::from_raw(b.vcu_ads_brake_state_raw())),
            self.eps
                .filter(|_| fresh(self.eps_at))
                .map(|e| SubsystemState::from_raw(e.vcu_ads_eps_state_raw())),
            self.veh
                .filter(|_| fresh(self.veh_at))
                .map(|v| SubsystemState::from_raw(v.vcu_ads_driving_state_raw())),
        ]
    }

    /// Aggregate the four subsystem states into the vehicle's control mode.
    /// Unanimity is required in both directions; everything else — including
    /// a single stale frame — is `Abnormal`, which blocks all commanding.
    pub fn vehicle_mode(&self, report_timeout: Duration) -> VehicleMode {
        let states = self.subsystem_states(report_timeout);
        let all = |want: SubsystemState| states.iter().all(|s| *s == Some(want));
        if all(SubsystemState::Autonomous) {
            VehicleMode::Autonomous
        } else if all(SubsystemState::Manual) {
            VehicleMode::Manual
        } else {
            VehicleMode::Abnormal
        }
    }
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
///   * `can_io::tx_loop`            — `status` (mode + speed) and `command`
///     via separate temporary locks, never nested
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

#[cfg(test)]
mod vehicle_mode_tests {
    use super::*;
    use std::time::Duration;

    const TIMEOUT: Duration = Duration::from_millis(1000);
    const MANUAL: u8 = 1;
    const AUTO: u8 = 2;
    const INVALID: u8 = 0;

    /// Build a status where every frame arrived just now with the given
    /// subsystem states `[mtr, brk, eps, drv]`.
    fn status(states: [u8; 4]) -> StatusState {
        let now = Instant::now();
        StatusState {
            mtr: Some(VcuAdsMtr::new(states[0], 0, Gear::Parking.to_raw(), 0.0).unwrap()),
            mtr_at: Some(now),
            brk: Some(VcuAdsBrk::new(states[1], 0, 0, 0.0).unwrap()),
            brk_at: Some(now),
            eps: Some(VcuAdsEps::new(states[2], 0.0, 0.0).unwrap()),
            eps_at: Some(now),
            veh: Some(
                VcuAdsVehicle::new(0, states[3], false, 0, false, 0, false, false, false, false)
                    .unwrap(),
            ),
            veh_at: Some(now),
            ..Default::default()
        }
    }

    #[test]
    fn all_manual_is_manual() {
        assert_eq!(
            status([MANUAL; 4]).vehicle_mode(TIMEOUT),
            VehicleMode::Manual
        );
    }

    #[test]
    fn all_autonomous_is_autonomous() {
        assert_eq!(
            status([AUTO; 4]).vehicle_mode(TIMEOUT),
            VehicleMode::Autonomous
        );
    }

    #[test]
    fn mixed_states_are_abnormal() {
        // Three across, one lagging behind — the handover is incomplete.
        assert_eq!(
            status([AUTO, AUTO, AUTO, MANUAL]).vehicle_mode(TIMEOUT),
            VehicleMode::Abnormal
        );
        assert_eq!(
            status([MANUAL, MANUAL, MANUAL, AUTO]).vehicle_mode(TIMEOUT),
            VehicleMode::Abnormal
        );
    }

    #[test]
    fn invalid_state_is_abnormal() {
        // Post-restart BRK/Drv come up Invalid; never treat that as either mode.
        assert_eq!(
            status([AUTO, INVALID, AUTO, AUTO]).vehicle_mode(TIMEOUT),
            VehicleMode::Abnormal
        );
    }

    #[test]
    fn no_frames_is_abnormal() {
        assert_eq!(
            StatusState::default().vehicle_mode(TIMEOUT),
            VehicleMode::Abnormal
        );
    }

    #[test]
    fn stale_frame_is_abnormal() {
        // All four say Autonomous, but the brake report has gone quiet: a
        // cached state must never keep us commanding.
        let mut s = status([AUTO; 4]);
        s.brk_at = Some(Instant::now() - Duration::from_secs(5));
        assert_eq!(s.vehicle_mode(TIMEOUT), VehicleMode::Abnormal);
        assert_eq!(s.subsystem_states(TIMEOUT)[1], None);
    }
}

#[cfg(test)]
mod estop_tests {
    //! `vehicle_cmd_gate` publishes `emergency_cmd` with `emergency=false` on
    //! every control cycle, so a release on one topic must never clear an
    //! e-stop the other topic is holding.

    use super::{EstopSource, EstopState};

    #[test]
    fn driver_estop_survives_mrm_false() {
        let mut e = EstopState::default();
        e.set(EstopSource::Driver, true);
        e.set(EstopSource::Mrm, false); // the gate's routine per-cycle message
        assert!(e.active(), "driver e-stop must hold through emergency_cmd=false");
        assert!(e.driver());
        assert!(!e.mrm());
    }

    #[test]
    fn mrm_estop_survives_driver_false() {
        let mut e = EstopState::default();
        e.set(EstopSource::Mrm, true);
        e.set(EstopSource::Driver, false);
        assert!(e.active(), "MRM e-stop must hold through emergency_stop=false");
        assert!(e.mrm());
        assert!(!e.driver());
    }

    #[test]
    fn released_only_when_both_release() {
        let mut e = EstopState::default();
        e.set(EstopSource::Driver, true);
        e.set(EstopSource::Mrm, true);
        e.set(EstopSource::Driver, false);
        assert!(e.active());
        e.set(EstopSource::Mrm, false);
        assert!(!e.active());
    }

    #[test]
    fn set_reports_only_that_sources_transitions() {
        let mut e = EstopState::default();
        assert_eq!(e.set(EstopSource::Driver, true), Some(true));
        assert_eq!(e.set(EstopSource::Driver, true), None, "repeat is not a transition");
        // The MRM flag was never set, so a routine false from the gate is
        // not a release and must not be logged as one.
        assert_eq!(e.set(EstopSource::Mrm, false), None);
        assert_eq!(e.set(EstopSource::Driver, false), Some(false));
    }
}

#[cfg(test)]
mod hazard_latch_tests {
    //! The 2026-11-09 failure: an err_mtr blip 1-3 s after leaving AUTO left
    //! the latch set, and the next dashboard AUTO was refused.

    use super::{CommandState, HazardLatchEvent, VehicleMode};
    use std::time::{Duration, Instant};

    const HOLD: Duration = Duration::from_millis(2000);
    #[allow(non_snake_case)]
    fn MS(ms: u64) -> Duration {
        Duration::from_millis(ms)
    }

    #[test]
    fn blip_after_leaving_auto_clears_after_the_hold_in_manual() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        // Driver left AUTO; 2.7 s later one frame carries err_mtr.
        assert_eq!(
            c.update_hazard_latch(true, VehicleMode::Manual, t0, HOLD),
            HazardLatchEvent::Latched
        );
        assert!(c.fault_latched());
        // Quiet frames in Manual, short of the hold: still latched.
        assert_eq!(
            c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(1990), HOLD),
            HazardLatchEvent::Unchanged
        );
        assert!(c.fault_latched());
        // Hold elapsed.
        assert_eq!(
            c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(2000), HOLD),
            HazardLatchEvent::AutoCleared
        );
        assert!(!c.fault_latched());
        // Logged once: the next frame is not another clear.
        assert_eq!(
            c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(2020), HOLD),
            HazardLatchEvent::Unchanged
        );
    }

    #[test]
    fn hazard_still_asserted_in_manual_stays_latched() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        for i in 0..300 {
            c.update_hazard_latch(true, VehicleMode::Manual, t0 + MS(20 * i), HOLD);
        }
        assert!(c.hazard_latched);
        // One quiet frame right after a long assertion does not clear it.
        assert_eq!(
            c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(6000), HOLD),
            HazardLatchEvent::Unchanged
        );
        assert!(c.hazard_latched);
    }

    #[test]
    fn hazard_resets_the_hold() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        c.update_hazard_latch(true, VehicleMode::Manual, t0, HOLD);
        c.update_hazard_latch(true, VehicleMode::Manual, t0 + MS(1500), HOLD);
        c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(2500), HOLD);
        assert!(c.hazard_latched, "hold runs from the last hazard frame");
        c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(3500), HOLD);
        assert!(!c.hazard_latched);
    }

    #[test]
    fn autonomous_never_auto_clears() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        c.update_hazard_latch(true, VehicleMode::Autonomous, t0, HOLD);
        for mode in [VehicleMode::Autonomous, VehicleMode::Abnormal] {
            assert_eq!(
                c.update_hazard_latch(false, mode, t0 + Duration::from_secs(60), HOLD),
                HazardLatchEvent::Unchanged
            );
            assert!(c.fault_latched(), "{mode:?} must not clear the latch");
        }
    }

    #[test]
    fn tx_failure_latch_is_not_auto_cleared() {
        let mut c = CommandState::default();
        c.tx_fault_latched = true;
        let t0 = Instant::now();
        c.update_hazard_latch(false, VehicleMode::Manual, t0, HOLD);
        c.update_hazard_latch(false, VehicleMode::Manual, t0 + Duration::from_secs(60), HOLD);
        assert!(c.fault_latched());
        // A hazard latch cleared beside it leaves the TX latch standing.
        c.update_hazard_latch(true, VehicleMode::Manual, t0, HOLD);
        assert_eq!(
            c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(2000), HOLD),
            HazardLatchEvent::AutoCleared
        );
        assert!(c.tx_fault_latched && c.fault_latched());
        // Only the explicit MANUAL/NO_COMMAND clear removes it.
        c.clear_fault_latches();
        assert!(!c.fault_latched());
    }

    #[test]
    fn zero_hold_disables_the_auto_clear() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        c.update_hazard_latch(true, VehicleMode::Manual, t0, Duration::ZERO);
        c.update_hazard_latch(false, VehicleMode::Manual, t0 + Duration::from_secs(60), Duration::ZERO);
        assert!(c.hazard_latched);
    }

    #[test]
    fn hazard_on_entering_auto_relatches() {
        let mut c = CommandState::default();
        let t0 = Instant::now();
        c.update_hazard_latch(true, VehicleMode::Manual, t0, HOLD);
        c.update_hazard_latch(false, VehicleMode::Manual, t0 + MS(2000), HOLD);
        assert!(!c.fault_latched());
        assert_eq!(
            c.update_hazard_latch(true, VehicleMode::Autonomous, t0 + MS(5000), HOLD),
            HazardLatchEvent::Latched
        );
        assert!(c.fault_latched());
    }
}
