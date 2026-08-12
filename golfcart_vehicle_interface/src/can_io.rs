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
    checksum_stub, AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, BlinkerCtrl, BrakeMode, EpsMode,
    Gear, Messages, VcuAdsVehicle,
};
use crate::state::{CommandState, SharedState, VehicleMode};

const NODE_NAME: &str = "golfcart_vehicle_interface";

/// Brake deceleration (m/s²) commanded during safety-brake (Autoware silent
/// past the control-watchdog window, ECU fault, or driver e-stop). ROOTS brakes
/// on `Ads_Vcu_Target_Deceleration` only — stroke/pressure are ignored — and
/// the deceleration is segmented: <1.2 no brake, 1.2–1.8 ~33%, 1.8–2.8 ~70%,
/// ≥2.8 ~100%. We sit in segment 3 for a firm, definite stop. The vehicle-frame
/// `Ads_Vcu_Veh_Estop` bit is also asserted as a redundant max-decel path.
const SAFETY_BRAKE_DECEL_MPS2: f32 = 3.0;

/// `Ads_Vcu_Ads_Status`: 1 = Running, 0 = Error. Held at Running for as long as
/// this node transmits — see the note in `build_frames`.
const ADS_STATUS_RUNNING: u8 = 1;

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
/// at `brake_decel_mps2` (ROOTS honours deceleration, not pressure) so the
/// gearbox engages cleanly.
#[derive(Debug, Clone, Copy)]
pub struct GearShiftConfig {
    pub change_margin: Duration,
    pub brake_decel_mps2: f32,
    pub low_vel_thresh_mps: f32,
}

pub fn spawn(
    interface: &str,
    tx_enabled: bool,
    tx_rate_hz: f64,
    control_timeout: Duration,
    report_timeout: Duration,
    steer_limits: SteerLimits,
    gear_config: GearShiftConfig,
    steering_sign: f32,
    control_min_rate_hz: f32,
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
                report_timeout,
                steer_limits,
                gear_config,
                steering_sign,
                control_min_rate_hz,
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
/// if any are set. A latched fault forces the TX loop into safety-brake (while
/// the vehicle is in `Autonomous`) until the user issues a MANUAL/NO_COMMAND
/// request.
///
/// Driver override needs no handling here: the driver switching any subsystem
/// back to `Manual` breaks the all-four-`Autonomous` unanimity, so the TX loop
/// stops commanding on its own (see `TxMode::evaluate`).
fn handle_vehicle_status(state: &Arc<SharedState>, m: &VcuAdsVehicle) {
    let estop = m.vcu_ads_estop_raw();
    let err_sys = m.vcu_ads_error_code_sys_raw();
    let err_mtr = m.vcu_ads_error_code_mtr_raw();
    let err_eps = m.vcu_ads_error_code_eps_raw();
    let err_brk = m.vcu_ads_error_code_brk_raw();
    let fault = estop || err_sys || err_mtr || err_eps || err_brk;

    if !fault {
        return;
    }

    let mut cmd = state.command.lock();
    if !cmd.fault_latched {
        // Log only on the rising edge — frames arrive at ~50 Hz and we don't
        // want to spam.
        log_error!(
            NODE_NAME,
            "ECU hazard latched: estop={} err_sys={} err_mtr={} err_eps={} err_brk={}",
            estop, err_sys, err_mtr, err_eps, err_brk
        );
    }
    cmd.fault_latched = true;
}

fn tx_loop(
    mut socket: CanSocket,
    interface: String,
    tx_enabled: bool,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
    period: Duration,
    control_timeout: Duration,
    report_timeout: Duration,
    steer_limits: SteerLimits,
    gear_config: GearShiftConfig,
    steering_sign: f32,
    control_min_rate_hz: f32,
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
    // Rate-guard logging: one line when the input rate goes bad, one when it
    // recovers, and a reminder no more often than this while it stays bad. At
    // 100 Hz TX an unthrottled warning would bury every other log line.
    const RATE_WARN_INTERVAL: Duration = Duration::from_secs(5);
    // Recovery needs the rate to hold up for this long. Without hysteresis a
    // slow publisher looks healthy for the instant after each message arrives,
    // which would flap the state, defeat the log throttle, and hand speed
    // authority back and forth many times a second.
    const RATE_RECOVER_HOLD: Duration = Duration::from_secs(1);
    let mut rate_low = false;
    let mut last_rate_warn: Option<Instant> = None;
    let mut rate_ok_since: Option<Instant> = None;
    let mut prev_vehicle_mode = VehicleMode::default();
    while running.load(Ordering::Relaxed) {
        // Control mode is the vehicle's to declare: the driver switches the
        // subsystems over, we read the four reported states back. Commands go
        // on the wire only while all four say `Autonomous`.
        let vehicle_mode = state.status.lock().vehicle_mode(report_timeout);
        if vehicle_mode != prev_vehicle_mode {
            log_info!(
                NODE_NAME,
                "vehicle control mode {:?} -> {:?}",
                prev_vehicle_mode,
                vehicle_mode
            );
            if vehicle_mode == VehicleMode::Autonomous {
                // Entering auto: drop any Control timestamp from the previous
                // run, otherwise the staleness watchdog trips SafetyBrake
                // before the planner has published its first command.
                state.command.lock().last_control_at = None;
            }
            prev_vehicle_mode = vehicle_mode;
        }

        let mut cmd_snapshot = *state.command.lock();
        let mode = TxMode::evaluate(&cmd_snapshot, vehicle_mode, control_timeout);
        rolling = rolling.wrapping_add(1);

        // Autoware publishing too slowly to command a speed: hold the setpoint
        // at 0 and say so, throttled.
        let now = Instant::now();
        let guard = rate_guard(&cmd_snapshot, control_min_rate_hz, now);
        if guard.too_slow {
            rate_ok_since = None;
            let due = last_rate_warn.map_or(true, |t| now.duration_since(t) >= RATE_WARN_INTERVAL);
            if !rate_low || due {
                match guard.rate_hz {
                    Some(hz) => log_warn!(
                        NODE_NAME,
                        "Control input rate {hz:.1} Hz below control_min_rate_hz \
                         {control_min_rate_hz:.1} Hz: speed setpoint held at 0"
                    ),
                    None => log_warn!(
                        NODE_NAME,
                        "Control input too sparse to measure (min \
                         {control_min_rate_hz:.1} Hz): speed setpoint held at 0"
                    ),
                }
                last_rate_warn = Some(now);
            }
            rate_low = true;
        } else if rate_low {
            // Hold the latch until the rate has been good for a while — see
            // RATE_RECOVER_HOLD.
            let ok_since = *rate_ok_since.get_or_insert(now);
            if now.duration_since(ok_since) >= RATE_RECOVER_HOLD {
                match guard.rate_hz {
                    Some(hz) => log_info!(NODE_NAME, "Control input rate recovered: {hz:.1} Hz"),
                    None => log_info!(NODE_NAME, "Control input rate recovered"),
                }
                rate_low = false;
                rate_ok_since = None;
                last_rate_warn = None;
            }
        }
        // Latched, not instantaneous: speed authority is not handed back for
        // the few milliseconds after each late message.
        if rate_low {
            cmd_snapshot.target_speed_mps = 0.0;
            cmd_snapshot.target_acceleration_mps2 = 0.0;
            cmd_snapshot.target_throttle_pct = 0.0;
        }

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
        // Brake-during-shift is an actuation, so it is only asserted while we
        // are actually driving the vehicle — never in manual/abnormal.
        let shift_brake = driving && gear_pending && low_speed;
        // Parking pins both speed and steering to zero (see `build_frames`);
        // the slew limiter has to agree, or re-selecting Drive would replay a
        // stale angle through the rate limit.
        let parked = actual_gear == Gear::Parking;

        // Slew-limit steering setpoint based on actual vehicle speed.
        // Unknown speed forces stopped-rate (slowest) — better to track
        // tightly than yank the wheel on a phantom high-speed assumption.
        let limited_tire_rad = if driving && !parked {
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
            // When idle/parked/safety-brake, keep state aligned with the
            // commanded hold (zero) so re-engage starts from neutral.
            0.0
        };
        prev_tire_rad = limited_tire_rad;
        prev_tire_at = Some(now);

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
                Some(gear_config.brake_decel_mps2)
            } else {
                None
            },
            steering_sign,
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
    /// The vehicle is not in `Autonomous` — the driver holds it, or the
    /// reported states disagree. Send heartbeats with all enables off, all
    /// setpoints zero, and no Veh_Auto_En claim.
    Idle,
    /// Vehicle in `Autonomous`, but no Control message has arrived yet — wait
    /// without braking. We claim Veh_Auto_En so the VCU is ready to drive.
    EngagedWaiting,
    /// Driving: `Autonomous`, Control fresh, no faults. Apply Autoware setpoints.
    Driving,
    /// Safety brake: latched ECU fault, driver e-stop, OR auto+stale (the
    /// planner stopped emitting Control mid-drive). Command active braking
    /// while keeping Veh_Auto_En so the VCU honours the brake. Only reachable
    /// while the vehicle is in `Autonomous`; in manual the driver's own brake
    /// is the authority and we stay off the wire.
    SafetyBrake,
}

impl TxMode {
    fn evaluate(cmd: &CommandState, vehicle_mode: VehicleMode, control_timeout: Duration) -> Self {
        // Mode gate first: nothing is commanded unless all four VCU subsystem
        // states report Autonomous. Manual and Abnormal both mean hands off.
        if vehicle_mode != VehicleMode::Autonomous {
            return Self::Idle;
        }

        let was_driving = cmd.last_control_at.is_some();
        let stale = cmd
            .last_control_at
            .map(|t| t.elapsed() > control_timeout)
            .unwrap_or(true);

        if cmd.fault_latched || cmd.estop {
            return Self::SafetyBrake;
        }
        if was_driving && stale {
            return Self::SafetyBrake;
        }
        if !stale {
            return Self::Driving;
        }
        Self::EngagedWaiting
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

/// Verdict of the Control input-rate guard.
#[derive(Debug, Clone, Copy, PartialEq)]
struct RateGuard {
    /// Windowed publish rate, `None` until enough Control messages have arrived.
    rate_hz: Option<f32>,
    /// True while the rate is too low to command a speed.
    too_slow: bool,
}

/// Decide whether Autoware is publishing Control fast enough to be allowed to
/// command a speed, and zero the speed setpoint if it is not.
///
/// This is a *rate* check, distinct from the `control_timeout_ms` watchdog:
/// silence trips the watchdog and brakes, but a planner that keeps publishing at
/// a few hertz never trips it while still leaving the cart to travel blind
/// between updates. Steering, gear and the brake setpoint still pass through —
/// only the speed is withheld, so the vehicle coasts to a stop under whatever
/// the driver or the brake command asks for rather than being yanked.
///
/// Both the windowed rate and the age of the newest message are checked: the
/// window alone would keep reporting the old rate through a sudden stall until
/// enough new samples arrived to pull the average down.
fn rate_guard(cmd: &CommandState, min_rate_hz: f32, now: Instant) -> RateGuard {
    let rate_hz = cmd.control_freq.rate_hz();
    if min_rate_hz <= 0.0 || cmd.last_control_at.is_none() {
        // Guard disabled, or nothing has been received yet — the watchdog and
        // the TX state machine own that case.
        return RateGuard { rate_hz, too_slow: false };
    }
    let max_gap = Duration::from_secs_f32(1.0 / min_rate_hz);
    let gap_too_long = cmd
        .last_control_at
        .map(|t| now.saturating_duration_since(t) > max_gap)
        .unwrap_or(true);
    let window_too_slow = rate_hz.map_or(false, |hz| hz < min_rate_hz);
    RateGuard {
        rate_hz,
        too_slow: gap_too_long || window_too_slow,
    }
}

fn build_frames(
    cmd: &CommandState,
    mode: TxMode,
    rolling: u8,
    slew_limited_tire_rad: f32,
    actual_gear: Gear,
    shift_brake_decel_mps2: Option<f32>,
    steering_sign: f32,
) -> [(u32, [u8; 8]); 4] {
    let chksum = checksum_stub();
    let driving = mode == TxMode::Driving;
    let safety_brake = mode == TxMode::SafetyBrake;
    let authority = mode.claims_authority();
    let shift_braking = shift_brake_decel_mps2.is_some();

    // Two protections carried over from the ROOTS bench simulator
    // (`Roots_ipc_can_test.py`), re-applied on every frame we build so nothing
    // can leak onto the wire:
    //
    //   * Target speed is never negative. `Ads_Vcu_Target_Speed` is a
    //     magnitude — reverse is selected with gear R — and the VCU misbehaves
    //     on a negative target. `f32::max` also folds NaN to 0.0.
    //   * Parking pins speed and steering to zero. While the transmitted gear
    //     is P the cart must neither drive nor steer, whatever the planner
    //     asks for.
    let parked = actual_gear == Gear::Parking;
    let target_speed_mps = if parked {
        0.0
    } else {
        cmd.target_speed_mps.max(0.0)
    };
    let tire_rad = if parked { 0.0 } else { slew_limited_tire_rad };

    // Manual: once Ads_Vcu_Target_Deceleration > 0 the VCU cuts motor torque
    // (deceleration takes priority over speed) to protect the motor. Mirror
    // that here so motor and brake never fight on the bus — drop the motor
    // whenever we are actively decelerating on the planner's command.
    let decel_active = driving && cmd.target_deceleration_mps2 > 0.0;
    // Motor: only drive when on fresh planner setpoints, not shift-braking,
    // not decelerating, and not parked. Gear-enable stays on while driving so
    // the VCU always knows the requested gear even when torque is cut.
    let motor_active = driving && !shift_braking && !decel_active && !parked;
    let motor = AdsVcuMtr::new(
        motor_active,
        driving,
        // Mode bits are a claim about how we intend to drive, so they only go
        // out with authority. Idle frames are then byte-identical to the ROOTS
        // simulator's, which is what the VCU accepts before handing over.
        authority && cmd.motor_mode.as_bool(),
        actual_gear.to_raw(),
        if motor_active { cmd.target_throttle_pct } else { 0.0 },
        if motor_active { cmd.target_acceleration_mps2.max(0.0) } else { 0.0 },
        if motor_active { target_speed_mps } else { 0.0 },
        chksum,
    )
    .expect("AdsVcuMtr fields are clamped at the call site");

    // Brake: enable while driving (so Autoware can brake), in safety-brake
    // (planner failure / fault), and while a gear shift is pending at low
    // speed (settle the gearbox before engaging). ROOTS acts only on
    // `Ads_Vcu_Target_Deceleration`; stroke/pressure are ignored, so we send 0
    // for both and pick the deceleration setpoint per mode.
    let brake_decel = match (mode, shift_brake_decel_mps2) {
        (TxMode::SafetyBrake, _) => SAFETY_BRAKE_DECEL_MPS2,
        (_, Some(d)) => d,
        (TxMode::Driving, None) => cmd.target_deceleration_mps2,
        _ => 0.0,
    };
    let brake_mode = if !authority {
        BrakeMode::Invalid
    } else if safety_brake || shift_braking {
        BrakeMode::Pressure
    } else {
        cmd.brake_mode
    };
    let brake = AdsVcuBrk::new(
        driving || safety_brake || shift_braking,
        brake_mode.to_raw(),
        0.0, // stroke — ignored by ROOTS
        0.0, // pressure — ignored by ROOTS
        brake_decel,
        0,
        chksum,
    )
    .expect("AdsVcuBrk fields are clamped at the call site");

    // Steering: tracking only meaningful while driving. Hold straight (zero
    // setpoint, eps_en=false) in safety-brake — we don't want to swerve.
    // Use the slew-limited tire angle so EPS sees a continuous trajectory.
    let eps = AdsVcuEps::new(
        driving && !parked,
        if authority { cmd.eps_mode.to_raw() } else { EpsMode::Invalid.to_raw() },
        // Autoware's REP-103 sign (positive = left) converted to the vendor's
        // (positive = right) on the way out - see `invert_steering`.
        if driving { steering_sign * tire_rad.to_degrees() } else { 0.0 },
        0.0,
        chksum,
    )
    .expect("AdsVcuEps fields are clamped at the call site");

    // Vehicle frame: hand authority to VCU only when we're trying to do
    // something. Hazard blinker overrides the user blinker on safety-brake.
    //
    // `Ads_Status` is this node's own health, NOT whether we are driving. The
    // VCU reads it as "is the ADS alive and sane" and refuses to leave Manual
    // while it says Error - so reporting Error until we drive made the AUTO
    // button impossible to use, since the driver presses it while we are idle.
    // The ROOTS simulator holds it Running unconditionally; so do we, for as
    // long as this node is on the bus at all.
    //
    // Without authority (the vehicle is in manual or an abnormal state) every
    // actuating field goes out zeroed — lights, prompts and the e-stop bit
    // included. The driver owns the vehicle then; the frames are pure
    // heartbeat, which is what clears MTR/EPS out of `Invalid` on the VCU
    // side without commanding anything.
    let blinker = if !authority {
        BlinkerCtrl::Off
    } else if safety_brake {
        BlinkerCtrl::Hazard
    } else {
        cmd.blinker
    };
    let veh = AdsVcuVehicle::new(
        authority,
        ADS_STATUS_RUNNING,
        false,
        authority && (cmd.estop || safety_brake),
        blinker.to_raw(),
        authority && cmd.headlight,
        authority && matches!(cmd.blinker, BlinkerCtrl::Right),
        authority && matches!(cmd.blinker, BlinkerCtrl::Left),
        authority && matches!(actual_gear, Gear::Reverse),
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
    use crate::state::{CommandState, VehicleMode};
    use std::time::{Duration, Instant};

    const TIMEOUT: Duration = Duration::from_millis(500);
    const AUTO: VehicleMode = VehicleMode::Autonomous;

    fn cmd() -> CommandState {
        CommandState::default()
    }

    #[test]
    fn idle_when_vehicle_manual() {
        let mut c = cmd();
        c.last_control_at = Some(Instant::now());
        assert_eq!(
            TxMode::evaluate(&c, VehicleMode::Manual, TIMEOUT),
            TxMode::Idle
        );
    }

    #[test]
    fn idle_when_vehicle_abnormal() {
        // Mixed / Invalid / stale subsystem states: fresh planner commands
        // must not reach the wire.
        let mut c = cmd();
        c.last_control_at = Some(Instant::now());
        assert_eq!(
            TxMode::evaluate(&c, VehicleMode::Abnormal, TIMEOUT),
            TxMode::Idle
        );
    }

    #[test]
    fn idle_when_manual_even_with_estop_or_fault() {
        // In manual the driver owns the brake pedal; we stay off the wire
        // instead of commanding a safety brake behind their back.
        let mut c = cmd();
        c.estop = true;
        c.fault_latched = true;
        assert_eq!(
            TxMode::evaluate(&c, VehicleMode::Manual, TIMEOUT),
            TxMode::Idle
        );
    }

    #[test]
    fn engaged_waiting_when_no_control_yet() {
        let c = cmd();
        // last_control_at = None => never received Control
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::EngagedWaiting);
    }

    #[test]
    fn driving_when_fresh_control() {
        let mut c = cmd();
        c.last_control_at = Some(Instant::now());
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::Driving);
    }

    #[test]
    fn safety_brake_when_auto_and_stale() {
        let mut c = cmd();
        c.last_control_at = Some(Instant::now() - Duration::from_secs(2));
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn safety_brake_when_fault_latched_overrides_driving() {
        let mut c = cmd();
        c.last_control_at = Some(Instant::now());
        c.fault_latched = true;
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn safety_brake_when_estop_in_auto() {
        let mut c = cmd();
        c.estop = true;
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::SafetyBrake);
    }

    #[test]
    fn cold_start_does_not_safety_brake_on_handover() {
        // Handover path: was never driving, no Control yet — must NOT trip
        // SafetyBrake, must wait. Regression for the stale-watchdog bug.
        let mut c = cmd();
        c.last_control_at = None;
        assert_eq!(TxMode::evaluate(&c, AUTO, TIMEOUT), TxMode::EngagedWaiting);
    }

    #[test]
    fn claims_authority_excludes_idle() {
        assert!(!TxMode::Idle.claims_authority());
        assert!(TxMode::EngagedWaiting.claims_authority());
        assert!(TxMode::Driving.claims_authority());
        assert!(TxMode::SafetyBrake.claims_authority());
    }
}

#[cfg(test)]
mod protection_tests {
    //! The two ROOTS bench-simulator safety rules, checked on the bytes that
    //! `build_frames` actually produces: speed is never negative, and gear P
    //! pins speed and steering to zero.

    use super::{build_frames, TxMode};
    use crate::dbc::{AdsVcuEps, AdsVcuMtr, Gear};
    use crate::state::CommandState;
    use std::time::Instant;

    /// Steering sign for tests that are not about the sign itself.
    const NO_FLIP: f32 = 1.0;

    fn driving_cmd() -> CommandState {
        let mut c = CommandState::default();
        c.last_control_at = Some(Instant::now());
        c
    }

    /// Decode the MTR and EPS frames out of a `build_frames` result.
    fn mtr_eps(frames: [(u32, [u8; 8]); 4]) -> (AdsVcuMtr, AdsVcuEps) {
        let mtr = AdsVcuMtr::try_from(frames[0].1.as_slice()).expect("MTR decodes");
        let eps = AdsVcuEps::try_from(frames[2].1.as_slice()).expect("EPS decodes");
        (mtr, eps)
    }

    #[test]
    fn negative_speed_never_reaches_the_wire() {
        let mut c = driving_cmd();
        c.target_speed_mps = -3.0; // reverse is gear R, never a negative target
        let (mtr, _) = mtr_eps(build_frames(&c, TxMode::Driving, 0, 0.0, Gear::Drive, None, NO_FLIP));
        assert_eq!(mtr.ads_vcu_target_speed(), 0.0);
    }

    #[test]
    fn nan_speed_folds_to_zero() {
        let mut c = driving_cmd();
        c.target_speed_mps = f32::NAN;
        let (mtr, _) = mtr_eps(build_frames(&c, TxMode::Driving, 0, 0.0, Gear::Drive, None, NO_FLIP));
        assert_eq!(mtr.ads_vcu_target_speed(), 0.0);
    }

    #[test]
    fn parking_pins_speed_and_steering() {
        let mut c = driving_cmd();
        c.target_speed_mps = 2.0;
        c.target_tire_angle_rad = 0.3;
        let (mtr, eps) = mtr_eps(build_frames(
            &c,
            TxMode::Driving,
            0,
            0.3, // slew limiter already tracking a non-zero angle
            Gear::Parking,
            None,
            NO_FLIP,
        ));
        assert_eq!(mtr.ads_vcu_target_speed(), 0.0);
        assert_eq!(eps.ads_vcu_target_tire_angle(), 0.0);
        assert!(!bool::from(mtr.ads_vcu_motor_en()));
        assert!(!bool::from(eps.ads_vcu_eps_en()));
    }

    #[test]
    fn drive_gear_still_passes_setpoints() {
        // Guard against the parking pin being over-eager.
        let mut c = driving_cmd();
        c.target_speed_mps = 2.0;
        let (mtr, eps) = mtr_eps(build_frames(&c, TxMode::Driving, 0, 0.2, Gear::Drive, None, NO_FLIP));
        assert!((mtr.ads_vcu_target_speed() - 2.0).abs() < 0.05);
        assert!((eps.ads_vcu_target_tire_angle() - 0.2_f32.to_degrees()).abs() < 0.5);
    }

    #[test]
    fn idle_mode_sends_no_setpoints() {
        // Vehicle in manual/abnormal: heartbeat only.
        let mut c = driving_cmd();
        c.target_speed_mps = 2.0;
        c.target_tire_angle_rad = 0.3;
        let (mtr, eps) = mtr_eps(build_frames(&c, TxMode::Idle, 0, 0.3, Gear::Drive, None, NO_FLIP));
        assert_eq!(mtr.ads_vcu_target_speed(), 0.0);
        assert_eq!(eps.ads_vcu_target_tire_angle(), 0.0);
        assert!(!bool::from(mtr.ads_vcu_motor_en()));
        assert!(!bool::from(mtr.ads_vcu_gear_en()));
        assert!(!bool::from(eps.ads_vcu_eps_en()));
    }
}

#[cfg(test)]
mod heartbeat_tests {
    //! The idle heartbeat is what the driver's AUTO button is judged against:
    //! the VCU only leaves `Manual` if the ADS frames on the bus look right
    //! while it is still idle.
    //!
    //! The expected bytes are the ones captured from the ROOTS bench simulator
    //! (`Roots_ipc_can_test.py`) on the run where the AUTO button worked —
    //! `candump` on `can0`, 2026-08-12. Our own capture from the same session
    //! differed in exactly the fields fixed here, and the VCU stayed `Manual`.

    use super::{build_frames, TxMode};
    use crate::dbc::{AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, Gear};
    use crate::state::CommandState;

    /// Steering sign for tests that are not about the sign itself.
    const NO_FLIP: f32 = 1.0;

    /// MTR, BRK, EPS, VEHICLE payloads for an idle tick with rolling counter 0x5F.
    fn idle_frames() -> [(u32, [u8; 8]); 4] {
        build_frames(
            &CommandState::default(),
            TxMode::Idle,
            0x5F,
            0.0,
            Gear::Parking,
            None,
            NO_FLIP,
        )
    }

    #[test]
    fn idle_heartbeat_matches_the_reference_capture() {
        let frames = idle_frames();
        assert_eq!(frames[0].1, [0x00; 8], "MTR heartbeat");
        assert_eq!(frames[1].1, [0x00; 8], "BRK heartbeat");
        assert_eq!(frames[2].1, [0x00; 8], "EPS heartbeat");
        assert_eq!(
            frames[3].1,
            [0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5F, 0x00],
            "VEHICLE heartbeat: Ads_Status=Running, rolling counter, nothing else"
        );
    }

    #[test]
    fn ads_status_is_running_in_every_mode() {
        // It reports this node's health, not whether we are driving. Held at
        // Running so the VCU will hand over when the driver asks it to.
        for mode in [
            TxMode::Idle,
            TxMode::EngagedWaiting,
            TxMode::Driving,
            TxMode::SafetyBrake,
        ] {
            let frames = build_frames(
                &CommandState::default(),
                mode,
                0,
                0.0,
                Gear::Parking,
                None,
                NO_FLIP,
            );
            let veh = AdsVcuVehicle::try_from(frames[3].1.as_slice()).expect("VEHICLE decodes");
            assert_eq!(veh.ads_vcu_ads_status_raw(), 1, "Ads_Status in {mode:?}");
        }
    }

    #[test]
    fn idle_claims_no_mode() {
        // Mode bits are a claim about how we intend to drive. Sending them
        // while the driver holds the vehicle is what made our idle frames
        // differ from the reference.
        let frames = idle_frames();
        let mtr = AdsVcuMtr::try_from(frames[0].1.as_slice()).expect("MTR decodes");
        let brk = AdsVcuBrk::try_from(frames[1].1.as_slice()).expect("BRK decodes");
        let eps = AdsVcuEps::try_from(frames[2].1.as_slice()).expect("EPS decodes");
        assert!(!bool::from(mtr.ads_vcu_mtr_mode()));
        assert_eq!(brk.ads_vcu_brk_mode_raw(), 0);
        assert_eq!(eps.ads_vcu_eps_mode_raw(), 0);
    }

    #[test]
    fn driving_still_claims_its_modes() {
        // The zeroing must not leak into the modes we need while driving.
        let mut c = CommandState::default();
        c.last_control_at = Some(std::time::Instant::now());
        let frames = build_frames(&c, TxMode::Driving, 0, 0.0, Gear::Drive, None, NO_FLIP);
        let mtr = AdsVcuMtr::try_from(frames[0].1.as_slice()).expect("MTR decodes");
        let brk = AdsVcuBrk::try_from(frames[1].1.as_slice()).expect("BRK decodes");
        let eps = AdsVcuEps::try_from(frames[2].1.as_slice()).expect("EPS decodes");
        assert!(bool::from(mtr.ads_vcu_mtr_mode()), "speed control");
        assert_eq!(brk.ads_vcu_brk_mode_raw(), 2, "pressure control");
        assert_eq!(eps.ads_vcu_eps_mode_raw(), 1, "front-wheel steering");
    }
}

#[cfg(test)]
mod steering_sign_tests {
    //! Autoware counts a positive tire angle to the left (REP-103); ROOTS counts
    //! it to the right — the vendor's own bench simulator maps its right-turn key
    //! to a positive `Ads_Vcu_Target_Tire_Angle`. The sign therefore flips at the
    //! CAN boundary, which is what `invert_steering` controls.

    use super::{build_frames, TxMode};
    use crate::dbc::{AdsVcuEps, Gear};
    use crate::state::CommandState;
    use std::time::Instant;

    const FLIP: f32 = -1.0;
    const NO_FLIP: f32 = 1.0;

    fn eps_degrees(tire_rad: f32, sign: f32) -> f32 {
        let mut c = CommandState::default();
        c.last_control_at = Some(Instant::now());
        c.target_tire_angle_rad = tire_rad;
        let frames = build_frames(&c, TxMode::Driving, 0, tire_rad, Gear::Drive, None, sign);
        AdsVcuEps::try_from(frames[2].1.as_slice())
            .expect("EPS decodes")
            .ads_vcu_target_tire_angle()
    }

    #[test]
    fn autoware_left_becomes_vendor_negative() {
        // +0.2 rad is a left turn for Autoware, so it must leave as negative deg.
        let deg = eps_degrees(0.2, FLIP);
        assert!(deg < 0.0, "left turn should be negative on the wire, got {deg}");
        assert!((deg + 0.2_f32.to_degrees()).abs() < 0.5, "magnitude preserved");
    }

    #[test]
    fn autoware_right_becomes_vendor_positive() {
        let deg = eps_degrees(-0.2, FLIP);
        assert!(deg > 0.0, "right turn should be positive on the wire, got {deg}");
    }

    #[test]
    fn flip_disabled_passes_the_sign_through() {
        assert!(eps_degrees(0.2, NO_FLIP) > 0.0);
        assert!(eps_degrees(-0.2, NO_FLIP) < 0.0);
    }

    #[test]
    fn zero_is_unaffected_by_the_flip() {
        // -0.0 would still decode as 0, but be explicit: straight stays straight.
        assert_eq!(eps_degrees(0.0, FLIP), 0.0);
    }
}

#[cfg(test)]
mod rate_guard_tests {
    //! The rate guard withholds the speed setpoint while Autoware publishes too
    //! slowly to steer a moving vehicle. It is deliberately separate from the
    //! `control_timeout_ms` watchdog, which handles silence by braking.

    use super::rate_guard;
    use crate::state::{CommandState, FREQ_WINDOW_LEN};
    use std::time::{Duration, Instant};

    const MIN_RATE: f32 = 10.0;

    /// A command state whose Control window was filled at `period` intervals,
    /// the newest sample `age` ago.
    fn cmd_at_rate(period: Duration, age: Duration) -> (CommandState, Instant) {
        let now = Instant::now();
        let newest = now - age;
        let mut c = CommandState::default();
        for i in (0..FREQ_WINDOW_LEN).rev() {
            c.control_freq.record(newest - period * i as u32);
        }
        c.last_control_at = Some(newest);
        (c, now)
    }

    #[test]
    fn fast_publisher_passes() {
        let (c, now) = cmd_at_rate(Duration::from_millis(20), Duration::from_millis(5));
        let g = rate_guard(&c, MIN_RATE, now);
        assert!(!g.too_slow);
        assert!(g.rate_hz.unwrap() > MIN_RATE);
    }

    #[test]
    fn slow_publisher_is_caught() {
        // 4 Hz: never trips a 500 ms watchdog, still far too slow to drive on.
        let (c, now) = cmd_at_rate(Duration::from_millis(250), Duration::from_millis(10));
        let g = rate_guard(&c, MIN_RATE, now);
        assert!(g.too_slow);
        assert!(g.rate_hz.unwrap() < MIN_RATE);
    }

    #[test]
    fn a_sudden_gap_is_caught_before_the_window_catches_up() {
        // Window still averages a healthy 50 Hz, but nothing has arrived for
        // 300 ms — three missed periods at the minimum rate.
        let (c, now) = cmd_at_rate(Duration::from_millis(20), Duration::from_millis(300));
        let g = rate_guard(&c, MIN_RATE, now);
        assert!(g.too_slow, "gap since the last message must count");
        assert!(g.rate_hz.unwrap() > MIN_RATE, "window is still optimistic");
    }

    #[test]
    fn nothing_received_yet_is_not_the_guards_problem() {
        // Before the first Control the TX state machine is in Idle or
        // EngagedWaiting; there is no speed to withhold.
        let c = CommandState::default();
        assert!(!rate_guard(&c, MIN_RATE, Instant::now()).too_slow);
    }

    #[test]
    fn zero_disables_the_guard() {
        let (c, now) = cmd_at_rate(Duration::from_millis(500), Duration::from_millis(400));
        assert!(!rate_guard(&c, 0.0, now).too_slow);
    }
}
