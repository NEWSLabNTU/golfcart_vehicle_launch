//! CAN socket I/O — RX and TX threads.
//!
//! `spawn_rx` reads frames in a blocking loop and updates `SharedState::status`.
//! `spawn_tx` polls `SharedState::command` at a fixed rate, encodes the four
//! ADS_VCU_* frames, and writes them to the socket. Both threads exit when
//! `running` flips to `false`.
//!
//! Frame encode/decode comes from the dbc-codegen output; see `dbc.rs`.

use anyhow::{Context, Result};
use rclrs::{log_error, log_info, log_warn};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Socket, StandardId};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::dbc::{
    checksum_stub, AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, BlinkerCtrl, BrakeMode, Gear,
    Messages, SubsystemState, VcuAdsVehicle,
};
use crate::state::{CommandState, SharedState};

const NODE_NAME: &str = "golfcart_vehicle_interface";

/// Brake pressure (MPa) commanded during safety-brake (Autoware silent past the
/// control-watchdog window, ECU fault, or driver e-stop). Picked moderate so
/// the cart stops without an axle-jolting lockup; DBC max is 12.75.
const SAFETY_BRAKE_PRESSURE_MPA: f32 = 4.0;

pub struct CanThreads {
    pub rx: JoinHandle<()>,
    pub tx: JoinHandle<()>,
}

/// Steering rate-limit configuration. Slew rate depends on speed bucket so
/// the EPS isn't yanked at standstill (mechanical stress) and tracks tightly
/// at speed (path-following accuracy).
#[derive(Debug, Clone, Copy)]
pub struct SteerLimits {
    pub stopped_rps: f32,
    pub low_vel_rps: f32,
    pub nominal_rps: f32,
    pub low_vel_thresh_mps: f32,
}

/// Gear-shift policy. `change_margin` debounces gear_cmd; while a shift is
/// pending and the cart is below `low_vel_thresh_mps`, the brake is asserted
/// at `brake_pressure_mpa` so the gearbox engages cleanly.
#[derive(Debug, Clone, Copy)]
pub struct GearShiftConfig {
    pub change_margin: Duration,
    pub brake_pressure_mpa: f32,
    pub low_vel_thresh_mps: f32,
}

pub fn spawn(
    interface: &str,
    tx_enabled: bool,
    tx_rate_hz: f64,
    control_timeout: Duration,
    steer_limits: SteerLimits,
    gear_config: GearShiftConfig,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
) -> Result<CanThreads> {
    // RX uses its own socket so a long blocking read can't queue behind a
    // TX write. SocketCAN sockets are cheap (file descriptor + ring buffer)
    // and independent timeouts simplify shutdown — they're worth keeping
    // separate even though one socket would technically work.
    let rx_socket = open_socket(interface).context("opening RX socket")?;
    let tx_socket = open_socket(interface).context("opening TX socket")?;

    let interface_owned: String = interface.to_string();

    let rx_state = Arc::clone(&state);
    let rx_running = Arc::clone(&running);
    let rx_iface = interface_owned.clone();
    let rx = thread::Builder::new()
        .name("can_rx".into())
        .spawn(move || rx_loop(rx_socket, rx_iface, rx_state, rx_running))?;

    let tx_state = Arc::clone(&state);
    let tx_running = Arc::clone(&running);
    let tx_iface = interface_owned;
    let period = Duration::from_secs_f64(1.0 / tx_rate_hz);
    let tx = thread::Builder::new()
        .name("can_tx".into())
        .spawn(move || {
            tx_loop(
                tx_socket,
                tx_iface,
                tx_enabled,
                tx_state,
                tx_running,
                period,
                control_timeout,
                steer_limits,
                gear_config,
            )
        })?;

    Ok(CanThreads { rx, tx })
}

fn open_socket(interface: &str) -> Result<CanSocket> {
    let sock = CanSocket::open(interface)
        .with_context(|| format!("opening CAN interface '{interface}'"))?;
    // Non-blocking with a short read timeout so the thread can observe shutdown.
    sock.set_read_timeout(Duration::from_millis(100))?;
    sock.set_write_timeout(Duration::from_millis(100))?;
    Ok(sock)
}

fn rx_loop(
    mut socket: CanSocket,
    interface: String,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
) {
    // Reopen budget: count consecutive non-timeout errors, attempt reopen
    // after threshold. Survives interface bounces (cable yank, USB-CAN
    // reset) without requiring a process restart.
    const REOPEN_AFTER_ERRORS: u32 = 10;
    const REOPEN_BACKOFF: Duration = Duration::from_millis(500);
    let mut consecutive_errors: u32 = 0;
    while running.load(Ordering::Relaxed) {
        let frame = match socket.read_frame() {
            Ok(f) => {
                consecutive_errors = 0;
                f
            }
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                consecutive_errors = consecutive_errors.saturating_add(1);
                log_error!(
                    NODE_NAME,
                    "CAN read error (consecutive={consecutive_errors}): {e}"
                );
                if consecutive_errors >= REOPEN_AFTER_ERRORS {
                    thread::sleep(REOPEN_BACKOFF);
                    match open_socket(&interface) {
                        Ok(new_sock) => {
                            log_warn!(NODE_NAME, "CAN RX socket reopened on '{interface}'");
                            socket = new_sock;
                            consecutive_errors = 0;
                        }
                        Err(open_err) => {
                            log_error!(
                                NODE_NAME,
                                "CAN RX reopen failed on '{interface}': {open_err:#}"
                            );
                        }
                    }
                } else {
                    thread::sleep(Duration::from_millis(50));
                }
                continue;
            }
        };

        let CanFrame::Data(data) = frame else {
            continue;
        };
        let id = match data.id() {
            socketcan::Id::Standard(s) => s.as_raw() as u32,
            socketcan::Id::Extended(e) => e.as_raw(),
        };
        let parsed = match Messages::from_can_message(id, data.data()) {
            Ok(m) => m,
            Err(_) => {
                // Unknown ID or malformed payload. Bump counter so diag can
                // surface flaky-transceiver vs silent-VCU.
                let mut s = state.status.lock();
                s.bad_frames = s.bad_frames.saturating_add(1);
                continue;
            }
        };
        let now = Instant::now();
        match parsed {
            Messages::VcuAdsMtr(m) => {
                let mut s = state.status.lock();
                s.mtr = Some(m);
                s.mtr_at = Some(now);
                s.mtr_freq.record(now);
            }
            Messages::VcuAdsEps(m) => {
                let mut s = state.status.lock();
                s.eps = Some(m);
                s.eps_at = Some(now);
                s.eps_freq.record(now);
            }
            Messages::VcuAdsBrk(m) => {
                let mut s = state.status.lock();
                s.brk = Some(m);
                s.brk_at = Some(now);
                s.brk_freq.record(now);
            }
            Messages::VcuAdsVehicle(m) => {
                {
                    let mut s = state.status.lock();
                    s.veh = Some(m);
                    s.veh_at = Some(now);
                    s.veh_freq.record(now);
                }
                handle_vehicle_status(&state, &m);
            }
            // ADS_VCU_* frames are TX-only from our side; ignore loopback.
            _ => {}
        }
    }
}

/// On every VCU_ADS_VEHICLE frame, look for hazard signals and latch a fault
/// if any are set, and detect driver overrides while we claim auto. Both
/// conditions disengage auto. A latched fault additionally forces the TX loop
/// into safety-brake until the user issues a MANUAL/NO_COMMAND request.
fn handle_vehicle_status(state: &Arc<SharedState>, m: &VcuAdsVehicle) {
    let estop = m.vcu_ads_estop_raw();
    let err_sys = m.vcu_ads_error_code_sys_raw();
    let err_mtr = m.vcu_ads_error_code_mtr_raw();
    let err_eps = m.vcu_ads_error_code_eps_raw();
    let err_brk = m.vcu_ads_error_code_brk_raw();
    let fault = estop || err_sys || err_mtr || err_eps || err_brk;
    let vcu_state = SubsystemState::from_raw(m.vcu_ads_driving_state_raw());
    let driver_override =
        matches!(vcu_state, SubsystemState::Manual | SubsystemState::RemoteControl);

    if !fault && !driver_override {
        return;
    }

    let mut cmd = state.command.lock();
    if fault && !cmd.fault_latched {
        // Log only on the rising edge — frames arrive at ~50 Hz and we don't
        // want to spam.
        log_error!(
            NODE_NAME,
            "ECU hazard latched: estop={} err_sys={} err_mtr={} err_eps={} err_brk={}",
            estop, err_sys, err_mtr, err_eps, err_brk
        );
    }
    if driver_override && cmd.auto_enabled {
        log_warn!(
            NODE_NAME,
            "driver override detected (VCU state {:?}); disengaging auto",
            vcu_state
        );
    }
    if fault {
        cmd.fault_latched = true;
    }
    cmd.auto_enabled = false;
}

fn tx_loop(
    mut socket: CanSocket,
    interface: String,
    tx_enabled: bool,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
    period: Duration,
    control_timeout: Duration,
    steer_limits: SteerLimits,
    gear_config: GearShiftConfig,
) {
    if !tx_enabled {
        log_warn!(
            NODE_NAME,
            "CAN TX DISABLED via tx_enabled=false: encoded frames will NOT \
             be sent to '{interface}'. Vehicle will not move from this node."
        );
    } else {
        log_info!(NODE_NAME, "CAN TX enabled on '{interface}'");
    }
    // MTR-frame staleness threshold for TX-side decisions. Independent of the
    // ROS-side `report_timeout_ms` because TX runs much faster (100 Hz) and a
    // shorter window keeps gating responsive to RX dropouts.
    const MTR_FRESH_FOR_TX: Duration = Duration::from_millis(250);

    let mut rolling: u8 = 0;
    let mut next_tick = Instant::now();
    let mut prev_tire_rad: f32 = 0.0;
    let mut prev_tire_at: Option<Instant> = None;
    let mut consecutive_tx_errors: u32 = 0;
    let mut last_tx_error_log: Option<Instant> = None;
    while running.load(Ordering::Relaxed) {
        let cmd_snapshot = *state.command.lock();
        let mode = TxMode::evaluate(&cmd_snapshot, control_timeout);
        rolling = rolling.wrapping_add(1);

        // Speed only trusted if MTR frame is fresh. Stale (or never received)
        // → mark unknown so callers default to the most conservative
        // branch: stopped steer-rate (slowest slew) and gear-shift gate
        // refuses the change.
        let (speed_mps, speed_known) = {
            let s = state.status.lock();
            match (s.mtr, s.mtr_at) {
                (Some(m), Some(at)) if at.elapsed() <= MTR_FRESH_FOR_TX => {
                    (m.vcu_ads_vehicle_speed().abs() as f32, true)
                }
                _ => (0.0, false),
            }
        };
        let now = Instant::now();
        let driving = mode == TxMode::Driving;

        // Slew-limit steering setpoint based on actual vehicle speed.
        // Unknown speed forces stopped-rate (slowest) — better to track
        // tightly than yank the wheel on a phantom high-speed assumption.
        let limited_tire_rad = if driving {
            let max_rate = if !speed_known || speed_mps < 0.05 {
                steer_limits.stopped_rps
            } else if speed_mps < steer_limits.low_vel_thresh_mps {
                steer_limits.low_vel_rps
            } else {
                steer_limits.nominal_rps
            };
            let dt = prev_tire_at.map(|t| now.duration_since(t).as_secs_f32()).unwrap_or(0.0);
            let max_step = max_rate * dt.max(0.0);
            let target = cmd_snapshot.target_tire_angle_rad;
            let delta = (target - prev_tire_rad).clamp(-max_step, max_step);
            prev_tire_rad + delta
        } else {
            // When idle/safety-brake, keep state aligned with the commanded
            // hold (zero) so re-engage starts from neutral.
            0.0
        };
        prev_tire_rad = limited_tire_rad;
        prev_tire_at = Some(now);

        // Gear anti-chatter: only forward a new gear once `change_margin` has
        // elapsed since the last accepted change AND the cart is below
        // `low_vel_thresh_mps`. Pending shifts trigger a brake assertion.
        let requested_gear = cmd_snapshot.gear;
        let last_change_ok = cmd_snapshot
            .last_gear_change_at
            .map_or(true, |t| now.duration_since(t) >= gear_config.change_margin);
        // Unknown speed forbids gear changes — refuses the dangerous case
        // where MTR is stale at a near-zero cached value while the cart is
        // actually moving (gearbox damage on engagement).
        let low_speed = speed_known && speed_mps < gear_config.low_vel_thresh_mps;
        let gear_pending = requested_gear != cmd_snapshot.last_gear_sent;
        let allow_change = gear_pending && last_change_ok && low_speed;
        let actual_gear = if allow_change {
            requested_gear
        } else {
            cmd_snapshot.last_gear_sent
        };
        let shift_brake = gear_pending && low_speed;
        // The blinker actually placed on the wire matches what build_frames
        // emits below: hazard while in safety-brake, otherwise the user cmd.
        let blinker_on_wire = if mode == TxMode::SafetyBrake {
            BlinkerCtrl::Hazard
        } else {
            cmd_snapshot.blinker
        };
        let blinker_changed = blinker_on_wire != cmd_snapshot.last_blinker_sent;
        if allow_change || blinker_changed {
            let mut c = state.command.lock();
            if allow_change {
                c.last_gear_sent = requested_gear;
                c.last_gear_change_at = Some(now);
            }
            if blinker_changed {
                c.last_blinker_sent = blinker_on_wire;
                c.last_blinker_change_at = Some(now);
            }
        }

        // CAN TX failure escalation: count consecutive ticks with at least
        // one frame failing to send. Threshold trips a fault latch and the
        // diagnostic flag, surfacing to Autoware / system_error_monitor.
        // Log rate-limited to one line per second to avoid drowning the
        // console at 100 Hz.
        const TX_FAIL_THRESHOLD: u32 = 25; // ~250 ms at 100 Hz
        const TX_LOG_INTERVAL: Duration = Duration::from_secs(1);
        let mut tick_failed = false;
        let frames = build_frames(
            &cmd_snapshot,
            mode,
            rolling,
            limited_tire_rad,
            actual_gear,
            if shift_brake {
                Some(gear_config.brake_pressure_mpa)
            } else {
                None
            },
        );
        // Skip the actual socket write when TX is administratively disabled.
        // We still run the rest of the loop so state (gear/blinker echo,
        // slew limiter) advances exactly as it would in production — useful
        // for bench testing the encoder pipeline without driving the cart.
        if tx_enabled {
            for (id, payload) in frames {
                if let Err(e) = send_frame(&socket, id, &payload) {
                    tick_failed = true;
                    let log_now = last_tx_error_log
                        .map_or(true, |t| t.elapsed() >= TX_LOG_INTERVAL);
                    if log_now {
                        log_error!(
                            NODE_NAME,
                            "CAN write error (id 0x{id:x}, consecutive={consecutive_tx_errors}): {e}"
                        );
                        last_tx_error_log = Some(Instant::now());
                    }
                }
            }
        }
        if tick_failed {
            consecutive_tx_errors = consecutive_tx_errors.saturating_add(1);
            if consecutive_tx_errors == TX_FAIL_THRESHOLD {
                log_error!(
                    NODE_NAME,
                    "CAN TX failing for {TX_FAIL_THRESHOLD} ticks; latching fault"
                );
                let mut c = state.command.lock();
                c.fault_latched = true;
                c.auto_enabled = false;
                c.tx_failed = true;
            }
            // Try reopening the socket every TX_REOPEN_INTERVAL ticks while
            // failing — handles interface bounce. Skip rate is gentle
            // because we still want to keep emitting heartbeats once link
            // returns.
            const TX_REOPEN_INTERVAL: u32 = 50;
            if consecutive_tx_errors % TX_REOPEN_INTERVAL == 0 {
                match open_socket(&interface) {
                    Ok(new_sock) => {
                        log_warn!(NODE_NAME, "CAN TX socket reopened on '{interface}'");
                        socket = new_sock;
                    }
                    Err(open_err) => {
                        log_error!(
                            NODE_NAME,
                            "CAN TX reopen failed on '{interface}': {open_err:#}"
                        );
                    }
                }
            }
        } else if consecutive_tx_errors > 0 {
            // Recovery — clear the diag flag but leave fault_latched alone:
            // operator must explicitly disengage to clear the latch (matches
            // ECU-fault recovery semantics).
            consecutive_tx_errors = 0;
            let mut c = state.command.lock();
            c.tx_failed = false;
        }

        next_tick += period;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            // Lagged — reset the schedule.
            next_tick = now;
        }
    }
}

/// Per-tick TX state machine. Decides what kind of frames to send based on
/// engagement, freshness of Autoware control, and any latched faults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxMode {
    /// User has not engaged (or just disengaged). Send heartbeats with all
    /// enables off; do not claim Veh_Auto_En.
    Idle,
    /// User engaged, but no Control message has arrived yet — wait without
    /// braking. We claim Veh_Auto_En so the VCU is ready to drive.
    EngagedWaiting,
    /// Driving: engaged, Control fresh, no faults. Apply Autoware setpoints.
    Driving,
    /// Safety brake: latched ECU fault, driver e-stop, OR engaged+stale (the
    /// planner stopped emitting Control mid-drive). Command active braking
    /// while keeping Veh_Auto_En so the VCU honours the brake.
    SafetyBrake,
}

impl TxMode {
    fn evaluate(cmd: &CommandState, control_timeout: Duration) -> Self {
        let was_driving = cmd.last_control_at.is_some();
        let stale = cmd
            .last_control_at
            .map(|t| t.elapsed() > control_timeout)
            .unwrap_or(true);

        if cmd.fault_latched || cmd.estop {
            return Self::SafetyBrake;
        }
        if cmd.auto_enabled && was_driving && stale {
            return Self::SafetyBrake;
        }
        if cmd.auto_enabled && !stale {
            return Self::Driving;
        }
        if cmd.auto_enabled {
            return Self::EngagedWaiting;
        }
        Self::Idle
    }

    /// True while we want the VCU to honour our actuator commands. False when
    /// we're idle or completely surrendering authority.
    fn claims_authority(self) -> bool {
        matches!(
            self,
            Self::EngagedWaiting | Self::Driving | Self::SafetyBrake
        )
    }
}

fn send_frame(socket: &CanSocket, id: u32, payload: &[u8; 8]) -> std::io::Result<()> {
    let std_id = StandardId::new(id as u16).expect("frame id fits in 11 bits");
    let frame = CanFrame::new(std_id, payload).expect("8-byte payload");
    socket.write_frame(&frame)
}

fn build_frames(
    cmd: &CommandState,
    mode: TxMode,
    rolling: u8,
    slew_limited_tire_rad: f32,
    actual_gear: Gear,
    shift_brake_pressure_mpa: Option<f32>,
) -> [(u32, [u8; 8]); 4] {
    let chksum = checksum_stub();
    let driving = mode == TxMode::Driving;
    let safety_brake = mode == TxMode::SafetyBrake;
    let authority = mode.claims_authority();
    let shift_braking = shift_brake_pressure_mpa.is_some();

    // Motor: only enable when actually driving on planner setpoints. Zero
    // throttle while shift-braking so the gear can engage cleanly.
    let motor = AdsVcuMtr::new(
        driving,
        driving,
        cmd.motor_mode.as_bool(),
        actual_gear.to_raw(),
        if driving && !shift_braking { cmd.target_throttle_pct } else { 0.0 },
        if driving && !shift_braking {
            cmd.target_acceleration_mps2.max(0.0)
        } else {
            0.0
        },
        if driving && !shift_braking { cmd.target_speed_mps } else { 0.0 },
        chksum,
    )
    .expect("AdsVcuMtr fields are clamped at the call site");

    // Brake: enable while driving (so Autoware can brake), in safety-brake
    // (planner failure / fault), and while a gear shift is pending at low
    // speed (settle the gearbox before engaging).
    let (brake_pressure, brake_decel) = match (mode, shift_brake_pressure_mpa) {
        (TxMode::SafetyBrake, _) => (SAFETY_BRAKE_PRESSURE_MPA, 0.0),
        (TxMode::Driving, Some(p)) => (p, 0.0),
        (TxMode::Driving, None) => (0.0, cmd.target_deceleration_mps2),
        (_, Some(p)) => (p, 0.0),
        _ => (0.0, 0.0),
    };
    let brake_mode = if safety_brake || shift_braking {
        BrakeMode::Pressure
    } else {
        cmd.brake_mode
    };
    let brake = AdsVcuBrk::new(
        driving || safety_brake || shift_braking,
        brake_mode.to_raw(),
        0.0,
        brake_pressure,
        brake_decel,
        0,
        chksum,
    )
    .expect("AdsVcuBrk fields are clamped at the call site");

    // Steering: tracking only meaningful while driving. Hold straight (zero
    // setpoint, eps_en=false) in safety-brake — we don't want to swerve.
    // Use the slew-limited tire angle so EPS sees a continuous trajectory.
    let eps = AdsVcuEps::new(
        driving,
        cmd.eps_mode.to_raw(),
        if driving {
            slew_limited_tire_rad.to_degrees()
        } else {
            0.0
        },
        0.0,
        chksum,
    )
    .expect("AdsVcuEps fields are clamped at the call site");

    // Vehicle frame: hand authority to VCU only when we're trying to do
    // something. Ads_Status reports "Running" only while genuinely driving;
    // anything else is "Error" so the VCU sees the abnormal state. Hazard
    // blinker overrides the user blinker on safety-brake.
    let blinker = if safety_brake {
        BlinkerCtrl::Hazard
    } else {
        cmd.blinker
    };
    let veh = AdsVcuVehicle::new(
        authority,
        if driving { 1 } else { 0 },
        false,
        cmd.estop || safety_brake,
        blinker.to_raw(),
        cmd.headlight,
        matches!(cmd.blinker, BlinkerCtrl::Right),
        matches!(cmd.blinker, BlinkerCtrl::Left),
        matches!(actual_gear, Gear::Reverse),
        authority,
        0,
        rolling,
        chksum,
    )
    .expect("AdsVcuVehicle fields are clamped at the call site");

    [
        (AdsVcuMtr::MESSAGE_ID, *motor.raw()),
        (AdsVcuBrk::MESSAGE_ID, *brake.raw()),
        (AdsVcuEps::MESSAGE_ID, *eps.raw()),
        (AdsVcuVehicle::MESSAGE_ID, *veh.raw()),
    ]
}

#[cfg(test)]
mod tx_mode_tests {
    use super::TxMode;
    use crate::state::CommandState;
    use std::time::{Duration, Instant};

    const TIMEOUT: Duration = Duration::from_millis(500);

    fn cmd() -> CommandState {
        CommandState::default()
    }

    #[test]
    fn idle_when_disengaged() {
        let c = cmd();
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::Idle);
    }

    #[test]
    fn engaged_waiting_when_no_control_yet() {
        let mut c = cmd();
        c.auto_enabled = true;
        // last_control_at = None => never received Control
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::EngagedWaiting);
    }

    #[test]
    fn driving_when_fresh_control() {
        let mut c = cmd();
        c.auto_enabled = true;
        c.last_control_at = Some(Instant::now());
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::Driving);
    }

    #[test]
    fn safety_brake_when_engaged_and_stale() {
        let mut c = cmd();
        c.auto_enabled = true;
        c.last_control_at = Some(Instant::now() - Duration::from_secs(2));
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn safety_brake_when_fault_latched_overrides_engagement() {
        let mut c = cmd();
        c.auto_enabled = true;
        c.last_control_at = Some(Instant::now());
        c.fault_latched = true;
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn safety_brake_when_estop_even_disengaged() {
        // estop should drive a brake regardless of auto state — guards against
        // the cart coasting after a driver e-stop while disengaged.
        let mut c = cmd();
        c.estop = true;
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn cold_start_does_not_safety_brake_on_engage() {
        // Re-engage path: was never driving, no Control yet — must NOT trip
        // SafetyBrake, must wait. Regression for the stale-watchdog bug.
        let mut c = cmd();
        c.auto_enabled = true;
        c.last_control_at = None;
        assert_eq!(TxMode::evaluate(&c, TIMEOUT), TxMode::EngagedWaiting);
    }

    #[test]
    fn claims_authority_excludes_idle() {
        assert!(!TxMode::Idle.claims_authority());
        assert!(TxMode::EngagedWaiting.claims_authority());
        assert!(TxMode::Driving.claims_authority());
        assert!(TxMode::SafetyBrake.claims_authority());
    }
}
