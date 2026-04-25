//! ROS parameter loading.

use anyhow::{Context, Result};
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

        Ok(Self {
            can_interface,
            tx_rate_hz,
            publish_rate_hz,
            frame_id,
        })
    }
}
