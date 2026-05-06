mod can_io;
mod dbc;
mod node;
mod params;
mod state;

use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_error, log_info};
use std::{
    sync::{atomic::{AtomicBool, Ordering}, Arc},
    time::Duration,
};

use crate::node::VehicleInterfaceNode;
use crate::params::Params;
use crate::state::SharedState;

const NODE_NAME: &str = "golfcart_vehicle_interface";

fn main() -> Result<()> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node(NODE_NAME)?;

    let params = Params::from_node(&node)?;
    let state = Arc::new(SharedState::default());

    let running = Arc::new(AtomicBool::new(true));
    let steer_limits = can_io::SteerLimits {
        stopped_rps: params.steer_rate_stopped_rps,
        low_vel_rps: params.steer_rate_low_vel_rps,
        nominal_rps: params.steer_rate_nominal_rps,
        low_vel_thresh_mps: params.steer_low_vel_thresh_mps,
    };
    let gear_config = can_io::GearShiftConfig {
        change_margin: Duration::from_millis(params.gear_change_margin_ms),
        brake_pressure_mpa: params.shift_brake_pressure_mpa,
        low_vel_thresh_mps: params.shift_low_vel_thresh_mps,
    };
    let can_threads = match can_io::spawn(
        &params.can_interface,
        params.tx_enabled,
        params.tx_rate_hz,
        Duration::from_millis(params.control_timeout_ms),
        steer_limits,
        gear_config,
        Arc::clone(&state),
        Arc::clone(&running),
    ) {
        Ok(t) => t,
        Err(e) => {
            log_error!(NODE_NAME, "failed to start CAN I/O: {e:#}");
            return Err(e);
        }
    };

    let _vi_node = VehicleInterfaceNode::new(&node, &params, Arc::clone(&state))?;

    log_info!(NODE_NAME, "spinning");
    let spin_result = executor.spin(SpinOptions::default()).first_error();

    log_info!(NODE_NAME, "shutting down CAN threads");
    running.store(false, Ordering::Relaxed);
    let _ = can_threads.rx.join();
    let _ = can_threads.tx.join();

    spin_result?;
    Ok(())
}
