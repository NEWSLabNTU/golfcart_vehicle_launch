//! Mock VCU — bench-rig companion for `golfcart_vehicle_interface`.
//!
//! Reads the four `ADS_VCU_*` command frames the interface sends, runs a
//! tiny first-order physics model, and emits plausible `VCU_ADS_*` echoes.
//! Intended for use against `vcan0` (see `scripts/can/up-vcan0.sh`) so the
//! whole TX FSM, fault-handling, and diagnostic surface can be exercised
//! without touching real hardware.
//!
//! CLI flags allow injecting the fault states the real VCU would produce:
//!
//!   --interface <iface>     CAN interface (default: vcan0)
//!   --rate <hz>             VCU_ADS_* TX rate (default: 50)
//!   --estop                 Assert Vcu_Ads_Estop
//!   --err-sys               Assert Vcu_Ads_Error_Code_Sys
//!   --err-mtr               Assert Vcu_Ads_Error_Code_Mtr
//!   --err-eps               Assert Vcu_Ads_Error_Code_Eps
//!   --err-brk               Assert Vcu_Ads_Error_Code_Brk
//!   --drop-mtr              Skip transmitting VCU_ADS_MTR
//!   --drop-eps              Skip transmitting VCU_ADS_EPS
//!   --drop-brk              Skip transmitting VCU_ADS_BRK
//!   --drop-veh              Skip transmitting VCU_ADS_VEHICLE
//!   --manual                Report Driving_State = Manual (instead of Autonomous)
//!   --help                  Print usage and exit
//!
//! Sharing the dbc-codegen output with the production binary is done by
//! including `../dbc.rs` directly — that file has the `include!()` macro
//! that pulls in `dbc_messages.rs` from `OUT_DIR` (same OUT_DIR for both
//! bins in this crate).

#[allow(dead_code)]
#[path = "../dbc.rs"]
mod dbc;

use anyhow::{Context, Result};
use socketcan::{CanFrame, CanSocket, EmbeddedFrame, Socket, StandardId};
use std::env;
use std::process;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::thread;
use std::time::{Duration, Instant};

use parking_lot::Mutex;

use dbc::{Messages, VcuAdsBrk, VcuAdsEps, VcuAdsMtr, VcuAdsVehicle};

#[derive(Debug, Clone, Copy)]
struct Config {
    interface: &'static str,
    rate_hz: f32,
    estop: bool,
    err_sys: bool,
    err_mtr: bool,
    err_eps: bool,
    err_brk: bool,
    drop_mtr: bool,
    drop_eps: bool,
    drop_brk: bool,
    drop_veh: bool,
    manual: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            interface: "vcan0",
            rate_hz: 50.0,
            estop: false,
            err_sys: false,
            err_mtr: false,
            err_eps: false,
            err_brk: false,
            drop_mtr: false,
            drop_eps: false,
            drop_brk: false,
            drop_veh: false,
            manual: false,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct World {
    // Latest ADS_VCU_* commands seen on the wire.
    auto_en: bool,
    motor_en: bool,
    eps_en: bool,
    target_speed_mps: f32,
    target_gear: u8,
    target_tire_deg: f32,
    target_brake_pressure: f32,
    // Integrated state.
    speed_mps: f32,
    tire_deg: f32,
    rolling_counter: u8,
}

impl Default for World {
    fn default() -> Self {
        Self {
            auto_en: false,
            motor_en: false,
            eps_en: false,
            target_speed_mps: 0.0,
            target_gear: 0,
            target_tire_deg: 0.0,
            target_brake_pressure: 0.0,
            speed_mps: 0.0,
            tire_deg: 0.0,
            rolling_counter: 0,
        }
    }
}

fn parse_args() -> Result<Config> {
    let mut cfg = Config::default();
    let args: Vec<String> = env::args().skip(1).collect();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--help" | "-h" => {
                print_help();
                process::exit(0);
            }
            "--interface" => {
                i += 1;
                let s = args
                    .get(i)
                    .with_context(|| "--interface needs a value")?
                    .clone();
                cfg.interface = Box::leak(s.into_boxed_str());
            }
            "--rate" => {
                i += 1;
                cfg.rate_hz = args
                    .get(i)
                    .with_context(|| "--rate needs a value")?
                    .parse()
                    .with_context(|| "--rate must be a float")?;
            }
            "--estop" => cfg.estop = true,
            "--err-sys" => cfg.err_sys = true,
            "--err-mtr" => cfg.err_mtr = true,
            "--err-eps" => cfg.err_eps = true,
            "--err-brk" => cfg.err_brk = true,
            "--drop-mtr" => cfg.drop_mtr = true,
            "--drop-eps" => cfg.drop_eps = true,
            "--drop-brk" => cfg.drop_brk = true,
            "--drop-veh" => cfg.drop_veh = true,
            "--manual" => cfg.manual = true,
            other => {
                anyhow::bail!("unknown arg '{other}' (try --help)");
            }
        }
        i += 1;
    }
    Ok(cfg)
}

fn print_help() {
    println!(
        "mock_vcu — bench companion for golfcart_vehicle_interface

Usage: mock_vcu [OPTIONS]

  --interface <iface>     CAN interface (default: vcan0)
  --rate <hz>             VCU_ADS_* TX rate (default: 50)
  --estop                 Assert Vcu_Ads_Estop
  --err-sys               Assert Vcu_Ads_Error_Code_Sys
  --err-mtr               Assert Vcu_Ads_Error_Code_Mtr
  --err-eps               Assert Vcu_Ads_Error_Code_Eps
  --err-brk               Assert Vcu_Ads_Error_Code_Brk
  --drop-mtr              Skip transmitting VCU_ADS_MTR
  --drop-eps              Skip transmitting VCU_ADS_EPS
  --drop-brk              Skip transmitting VCU_ADS_BRK
  --drop-veh              Skip transmitting VCU_ADS_VEHICLE
  --manual                Report Driving_State = Manual
  --help                  Print this help"
    );
}

fn open_socket(interface: &str) -> Result<CanSocket> {
    let sock = CanSocket::open(interface)
        .with_context(|| format!("opening CAN interface '{interface}'"))?;
    sock.set_read_timeout(Duration::from_millis(100))?;
    sock.set_write_timeout(Duration::from_millis(100))?;
    Ok(sock)
}

fn rx_loop(socket: CanSocket, world: Arc<Mutex<World>>, running: Arc<AtomicBool>) {
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
                eprintln!("[mock_vcu] CAN read error: {e}");
                thread::sleep(Duration::from_millis(50));
                continue;
            }
        };
        let CanFrame::Data(data) = frame else { continue };
        let id = match data.id() {
            socketcan::Id::Standard(s) => s.as_raw() as u32,
            socketcan::Id::Extended(e) => e.as_raw(),
        };
        let parsed = match Messages::from_can_message(id, data.data()) {
            Ok(m) => m,
            Err(_) => continue,
        };
        match parsed {
            Messages::AdsVcuMtr(m) => {
                let mut w = world.lock();
                w.motor_en = m.ads_vcu_motor_en().into();
                w.target_speed_mps = m.ads_vcu_target_speed();
                w.target_gear = m.ads_vcu_target_gear_raw();
            }
            Messages::AdsVcuEps(m) => {
                let mut w = world.lock();
                w.eps_en = m.ads_vcu_eps_en().into();
                w.target_tire_deg = m.ads_vcu_target_tire_angle();
            }
            Messages::AdsVcuBrk(m) => {
                let mut w = world.lock();
                w.target_brake_pressure = m.ads_vcu_target_pressure();
            }
            Messages::AdsVcuVehicle(m) => {
                let mut w = world.lock();
                w.auto_en = m.ads_vcu_veh_auto_en().into();
            }
            _ => {}
        }
    }
}

fn send_frame(socket: &CanSocket, id: u32, payload: &[u8; 8]) -> std::io::Result<()> {
    let std_id = StandardId::new(id as u16).expect("frame id fits in 11 bits");
    let frame = CanFrame::new(std_id, payload).expect("8-byte payload");
    socket.write_frame(&frame)
}

fn main() -> Result<()> {
    let cfg = parse_args()?;
    println!(
        "[mock_vcu] iface={} rate={}Hz estop={} drop=(mtr={},eps={},brk={},veh={})",
        cfg.interface, cfg.rate_hz, cfg.estop,
        cfg.drop_mtr, cfg.drop_eps, cfg.drop_brk, cfg.drop_veh
    );

    let rx_socket = open_socket(cfg.interface).context("opening RX socket")?;
    let tx_socket = open_socket(cfg.interface).context("opening TX socket")?;
    let world = Arc::new(Mutex::new(World::default()));
    let running = Arc::new(AtomicBool::new(true));

    // No explicit signal handler — SIGINT terminates the process and the
    // RX thread is killed with it. The `running` flag exists for symmetry
    // with the production binary's shutdown protocol; since we don't
    // intercept SIGINT, only the main TX loop reads it for graceful exit
    // on internal errors.

    let rx_world = Arc::clone(&world);
    let rx_running = Arc::clone(&running);
    let rx_thread = thread::spawn(move || rx_loop(rx_socket, rx_world, rx_running));

    // Physics constants — first-order lag toward target.
    const TAU_SPEED_S: f32 = 0.5;
    const TAU_TIRE_S: f32 = 0.2;
    let period = Duration::from_secs_f32(1.0 / cfg.rate_hz);
    let dt = period.as_secs_f32();
    let alpha_speed = (dt / TAU_SPEED_S).min(1.0);
    let alpha_tire = (dt / TAU_TIRE_S).min(1.0);

    let mut next_tick = Instant::now();
    while running.load(Ordering::Relaxed) {
        // Snapshot + integrate.
        let snapshot = {
            let mut w = world.lock();
            // Speed integration only when motor enabled and auto active —
            // otherwise the cart coasts toward zero.
            let target_speed = if w.motor_en && w.auto_en {
                w.target_speed_mps
            } else {
                0.0
            };
            w.speed_mps += alpha_speed * (target_speed - w.speed_mps);
            // Tire angle tracks regardless of eps_en (real EPS holds last
            // commanded position when disabled — we model the disabled
            // case by holding instead of zeroing).
            if w.eps_en {
                w.tire_deg += alpha_tire * (w.target_tire_deg - w.tire_deg);
            }
            w.rolling_counter = w.rolling_counter.wrapping_add(1);
            *w
        };

        if !cfg.drop_mtr {
            let driving_state: u8 = if cfg.manual {
                1
            } else if snapshot.auto_en {
                2
            } else {
                1
            };
            let gear_raw = snapshot.target_gear;
            let throttle_pct: u8 = ((snapshot.speed_mps.abs() / 5.0) * 100.0)
                .clamp(0.0, 255.0) as u8;
            let mtr = VcuAdsMtr::new(driving_state, throttle_pct, gear_raw, snapshot.speed_mps)
                .expect("MTR fields in range");
            let _ = send_frame(&tx_socket, VcuAdsMtr::MESSAGE_ID, mtr.raw());
        }

        if !cfg.drop_eps {
            let eps_state: u8 = if snapshot.eps_en { 1 } else { 0 };
            let eps = VcuAdsEps::new(eps_state, snapshot.tire_deg, 0.0)
                .expect("EPS fields in range");
            let _ = send_frame(&tx_socket, VcuAdsEps::MESSAGE_ID, eps.raw());
        }

        if !cfg.drop_brk {
            // Echo the commanded pressure so the diag pipeline sees a
            // reasonable feedback signal; brake position/stroke aren't
            // modelled.
            let brk = VcuAdsBrk::new(0, 0, 0, snapshot.target_brake_pressure)
                .expect("BRK fields in range");
            let _ = send_frame(&tx_socket, VcuAdsBrk::MESSAGE_ID, brk.raw());
        }

        if !cfg.drop_veh {
            let driving_state: u8 = if cfg.manual {
                1
            } else if snapshot.auto_en {
                2
            } else {
                1
            };
            let veh = VcuAdsVehicle::new(
                snapshot.rolling_counter,
                driving_state,
                cfg.estop,
                0,            // blinker echo
                false,        // headlight
                0,            // doors
                cfg.err_sys,
                cfg.err_mtr,
                cfg.err_eps,
                cfg.err_brk,
            )
            .expect("VEH fields in range");
            let _ = send_frame(&tx_socket, VcuAdsVehicle::MESSAGE_ID, veh.raw());
        }

        next_tick += period;
        let now = Instant::now();
        if next_tick > now {
            thread::sleep(next_tick - now);
        } else {
            next_tick = now;
        }
    }

    let _ = rx_thread.join();
    println!("[mock_vcu] shutdown");
    Ok(())
}

