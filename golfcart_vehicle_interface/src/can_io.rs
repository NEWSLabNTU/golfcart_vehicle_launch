//! CAN socket I/O — RX and TX threads.
//!
//! `spawn_rx` reads frames in a blocking loop and updates `SharedState::status`.
//! `spawn_tx` polls `SharedState::command` at a fixed rate, encodes the four
//! ADS_VCU_* frames, and writes them to the socket. Both threads exit when
//! `running` flips to `false`.

use anyhow::{Context, Result};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Socket, SocketOptions, StandardId};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::dbc::{
    AdsVcuBrk, AdsVcuEps, AdsVcuMtr, AdsVcuVehicle, BlinkerCtrl, BrakeMode, ID_ADS_VCU_BRK,
    ID_ADS_VCU_EPS, ID_ADS_VCU_MTR, ID_ADS_VCU_VEHICLE, ID_VCU_ADS_BRK, ID_VCU_ADS_EPS,
    ID_VCU_ADS_MTR, ID_VCU_ADS_VEHICLE, VcuAdsBrk, VcuAdsEps, VcuAdsMtr, VcuAdsVehicle,
};
use crate::state::{CommandState, SharedState};

const NODE_NAME: &str = "golfcart_vehicle_interface";
/// Watchdog for stale commands. If no Control message arrives within this
/// window we stop driving the throttle/brake/steer signals.
const CONTROL_TIMEOUT: Duration = Duration::from_millis(500);

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
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock
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

        let CanFrame::Data(data) = frame else { continue };
        let id = match data.id() {
            socketcan::Id::Standard(s) => s.as_raw() as u32,
            socketcan::Id::Extended(e) => e.as_raw(),
        };
        let payload = data.data();
        let mut status = state.status.lock();
        match id {
            ID_VCU_ADS_MTR => status.mtr = VcuAdsMtr::decode(payload),
            ID_VCU_ADS_EPS => status.eps = VcuAdsEps::decode(payload),
            ID_VCU_ADS_BRK => status.brk = VcuAdsBrk::decode(payload),
            ID_VCU_ADS_VEHICLE => status.veh = VcuAdsVehicle::decode(payload),
            _ => {}
        }
    }
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
        let stale = cmd
            .last_control_at
            .map(|t| t.elapsed() > CONTROL_TIMEOUT)
            .unwrap_or(true);
        // When stale or disengaged we still send heartbeats so the VCU sees a
        // live link, but with all enables off and zero setpoints.
        let live = cmd.auto_enabled && !stale && !cmd.estop;
        let frames = build_frames(&cmd, live, rolling);
        rolling = rolling.wrapping_add(1);

        for (id, payload) in frames {
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

fn send_frame(socket: &CanSocket, id: u32, payload: &[u8; 8]) -> std::io::Result<()> {
    let std_id = StandardId::new(id as u16).expect("frame id fits in 11 bits");
    let frame = CanFrame::new(std_id, payload).expect("8-byte payload");
    socket.write_frame(&frame)
}

fn build_frames(cmd: &CommandState, live: bool, rolling: u8) -> [(u32, [u8; 8]); 4] {
    let motor = AdsVcuMtr {
        motor_en: live,
        gear_en: live,
        mode: cmd.motor_mode,
        gear: cmd.gear,
        throttle_pct: if live { cmd.target_throttle_pct } else { 0.0 },
        acceleration_mps2: if live { cmd.target_acceleration_mps2.max(0.0) } else { 0.0 },
        speed_mps: if live { cmd.target_speed_mps } else { 0.0 },
    };

    let brake = AdsVcuBrk {
        brk_en: live || cmd.estop,
        mode: if cmd.estop { BrakeMode::Pressure } else { cmd.brake_mode },
        stroke_mm: 0.0,
        // Apply max pressure on e-stop; otherwise use the planner's deceleration request.
        pressure_mpa: if cmd.estop { 8.0 } else { 0.0 },
        deceleration_mps2: if live { cmd.target_deceleration_mps2 } else { 0.0 },
        pedal_pct: 0,
    };

    let eps = AdsVcuEps {
        eps_en: live,
        mode: cmd.eps_mode,
        tire_angle_deg: if live { cmd.target_tire_angle_rad.to_degrees() } else { 0.0 },
        tire_ang_speed_dps: 0.0,
    };

    let veh = AdsVcuVehicle {
        auto_en: cmd.auto_enabled,
        ads_status: if live { 1 } else { 0 },
        vin_req: false,
        estop: cmd.estop,
        blinker: if cmd.estop { BlinkerCtrl::Hazard } else { cmd.blinker },
        headlight: cmd.headlight,
        turn_right_prompt: matches!(cmd.blinker, BlinkerCtrl::Right),
        turn_left_prompt: matches!(cmd.blinker, BlinkerCtrl::Left),
        backup_prompt: matches!(cmd.gear, crate::dbc::Gear::Reverse),
        auto_prompt: cmd.auto_enabled,
        door_ctrl: 0,
        rolling_counter: rolling,
    };

    [
        (ID_ADS_VCU_MTR, motor.encode()),
        (ID_ADS_VCU_BRK, brake.encode()),
        (ID_ADS_VCU_EPS, eps.encode()),
        (ID_ADS_VCU_VEHICLE, veh.encode()),
    ]
}
