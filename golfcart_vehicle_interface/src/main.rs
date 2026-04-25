mod can_io;
mod dbc;
mod node;
mod params;
mod state;

use anyhow::Result;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions, log_error, log_info};
use std::sync::{atomic::{AtomicBool, Ordering}, Arc};

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
    let can_threads = match can_io::spawn(
        &params.can_interface,
        params.tx_rate_hz,
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
