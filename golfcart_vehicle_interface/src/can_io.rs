//! CAN socket I/O — RX and TX threads.
//!
//! `spawn_rx` reads frames in a blocking loop and updates `SharedState::status`.
//! `spawn_tx` polls `SharedState::command` at a fixed rate, encodes the four
//! ADS_VCU_* frames, and writes them to the socket. Both threads exit when
//! `running` flips to `false`.
//!
//! Frame encode/decode comes from the dbc-codegen output; see `dbc.rs`.

use anyhow::{Context, Result};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Socket, SocketOptions, StandardId};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::dbc::{
    checksum_stub, AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, BlinkerCtrl, BrakeMode, Gear,
    Messages, VcuAdsVehicle,
};
use crate::state::{CommandState, SharedState};

const NODE_NAME: &str = "golfcart_vehicle_interface";
/// Watchdog for stale commands. If no Control message arrives within this
/// window after we've started receiving them, we transition from "drive" to
/// "safety brake" — the planner is presumed crashed.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);

/// Brake pressure (MPa) commanded during safety-brake (Autoware silent for
/// CONTROL_TIMEOUT, ECU fault, or driver e-stop). Picked moderate so the cart
/// stops without an axle-jolting lockup; DBC max is 12.75.
const SAFETY_BRAKE_PRESSURE_MPA: f32 = 4.0;

pub struct CanThreads {
    pub rx: JoinHandle<()>,
    pub tx: JoinHandle<()>,
}

pub fn spawn(
    interface: &str,
    tx_rate_hz: f64,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
) -> Result<CanThreads> {
    let rx_socket = open_socket(interface).context("opening RX socket")?;
    let tx_socket = open_socket(interface).context("opening TX socket")?;

    let rx_state = Arc::clone(&state);
    let rx_running = Arc::clone(&running);
    let rx = thread::Builder::new()
        .name("can_rx".into())
        .spawn(move || rx_loop(rx_socket, rx_state, rx_running))?;

    let tx_state = Arc::clone(&state);
    let tx_running = Arc::clone(&running);
    let period = Duration::from_secs_f64(1.0 / tx_rate_hz);
    let tx = thread::Builder::new()
        .name("can_tx".into())
        .spawn(move || tx_loop(tx_socket, tx_state, tx_running, period))?;

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

fn rx_loop(socket: CanSocket, state: Arc<SharedState>, running: Arc<AtomicBool>) {
    while running.load(Ordering::Relaxed) {
        let frame = match socket.read_frame() {
            Ok(f) => f,
            Err(e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                continue;
            }
            Err(e) => {
                eprintln!("[{NODE_NAME}] CAN read error: {e}");
                thread::sleep(Duration::from_millis(50));
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
            Err(_) => continue, // unknown ID or malformed — skip
        };
        match parsed {
            Messages::VcuAdsMtr(m) => state.status.lock().mtr = Some(m),
            Messages::VcuAdsEps(m) => state.status.lock().eps = Some(m),
            Messages::VcuAdsBrk(m) => state.status.lock().brk = Some(m),
            Messages::VcuAdsVehicle(m) => {
                state.status.lock().veh = Some(m);
                handle_vehicle_status(&state, &m);
            }
            // ADS_VCU_* frames are TX-only from our side; ignore loopback.
            _ => {}
        }
    }
}

/// On every VCU_ADS_VEHICLE frame, look for hazard signals and latch a fault
/// if any are set. Latching disengages auto and forces the TX loop into
/// safety-brake until the user issues a MANUAL/NO_COMMAND request.
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
        eprintln!(
            "[{NODE_NAME}] ECU hazard latched: estop={} err_sys={} err_mtr={} err_eps={} err_brk={}",
            estop, err_sys, err_mtr, err_eps, err_brk
        );
    }
    cmd.fault_latched = true;
    cmd.auto_enabled = false;
}

fn tx_loop(
    socket: CanSocket,
    state: Arc<SharedState>,
    running: Arc<AtomicBool>,
    period: Duration,
) {
    let mut rolling: u8 = 0;
    let mut next_tick = Instant::now();
    while running.load(Ordering::Relaxed) {
        let cmd = *state.command.lock();
        let mode = TxMode::evaluate(&cmd);
        rolling = rolling.wrapping_add(1);

        for (id, payload) in build_frames(&cmd, mode, rolling) {
            if let Err(e) = send_frame(&socket, id, &payload) {
                eprintln!("[{NODE_NAME}] CAN write error (id 0x{id:x}): {e}");
            }
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
    fn evaluate(cmd: &CommandState) -> Self {
        let was_driving = cmd.last_control_at.is_some();
        let stale = cmd
            .last_control_at
            .map(|t| t.elapsed() > CONTROL_TIMEOUT)
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

fn build_frames(cmd: &CommandState, mode: TxMode, rolling: u8) -> [(u32, [u8; 8]); 4] {
    let chksum = checksum_stub();
    let driving = mode == TxMode::Driving;
    let safety_brake = mode == TxMode::SafetyBrake;
    let authority = mode.claims_authority();

    // Motor: only enable when actually driving on planner setpoints.
    let motor = AdsVcuMtr::new(
        driving,
        driving,
        cmd.motor_mode.as_bool(),
        cmd.gear.to_raw(),
        if driving { cmd.target_throttle_pct } else { 0.0 },
        if driving {
            cmd.target_acceleration_mps2.max(0.0)
        } else {
            0.0
        },
        if driving { cmd.target_speed_mps } else { 0.0 },
        chksum,
    )
    .expect("AdsVcuMtr fields are clamped at the call site");

    // Brake: enable while driving (so Autoware can brake) AND while in
    // safety-brake (so we can stop the cart on planner failure / fault).
    let (brake_pressure, brake_decel) = match mode {
        TxMode::Driving => (0.0, cmd.target_deceleration_mps2),
        TxMode::SafetyBrake => (SAFETY_BRAKE_PRESSURE_MPA, 0.0),
        _ => (0.0, 0.0),
    };
    let brake_mode = if safety_brake {
        BrakeMode::Pressure
    } else {
        cmd.brake_mode
    };
    let brake = AdsVcuBrk::new(
        driving || safety_brake,
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
    let eps = AdsVcuEps::new(
        driving,
        cmd.eps_mode.to_raw(),
        if driving {
            cmd.target_tire_angle_rad.to_degrees()
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
        matches!(cmd.gear, Gear::Reverse),
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
