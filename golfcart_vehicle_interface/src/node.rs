//! ROS node wiring: subscriptions/publishers/service backed by `SharedState`.

use anyhow::Result;
use autoware_control_msgs::msg::Control;
use autoware_vehicle_msgs::msg::{
    ControlModeReport, GearCommand, GearReport, HazardLightsCommand, HazardLightsReport,
    SteeringReport, TurnIndicatorsCommand, TurnIndicatorsReport, VelocityReport,
};
use autoware_vehicle_msgs::srv::{ControlModeCommand, ControlModeCommand_Request, ControlModeCommand_Response};
use builtin_interfaces::msg::Time;
use rclrs::{Node, Publisher, Service, ServiceInfo, Subscription, Timer, log_info};
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crate::dbc::{BlinkerCtrl, BrakeMode, EpsMode, Gear, MotorMode, SubsystemState};
use crate::params::Params;
use crate::state::SharedState;

const NODE_NAME: &str = "golfcart_vehicle_interface";

#[allow(dead_code)] // Held to keep ROS entities alive for the node's lifetime.
pub struct VehicleInterfaceNode {
    sub_control: Subscription<Control>,
    sub_gear: Subscription<GearCommand>,
    sub_turn: Subscription<TurnIndicatorsCommand>,
    sub_hazard: Subscription<HazardLightsCommand>,
    pub_velocity: Publisher<VelocityReport>,
    pub_steering: Publisher<SteeringReport>,
    pub_gear: Publisher<GearReport>,
    pub_mode: Publisher<ControlModeReport>,
    pub_turn: Publisher<TurnIndicatorsReport>,
    pub_hazard: Publisher<HazardLightsReport>,
    srv_mode: Service<ControlModeCommand>,
    publish_timer: Timer,
}

impl VehicleInterfaceNode {
    pub fn new(node: &Node, params: &Params, state: Arc<SharedState>) -> Result<Self> {
        // ----- Subscriptions: Autoware → CAN command state ------------------
        let sub_control = {
            let state = Arc::clone(&state);
            node.create_subscription("~/input/control_cmd", move |msg: Control| {
                let mut cmd = state.command.lock();
                cmd.target_speed_mps = msg.longitudinal.velocity;
                cmd.target_acceleration_mps2 = msg.longitudinal.acceleration;
                cmd.target_tire_angle_rad = msg.lateral.steering_tire_angle;
                cmd.motor_mode = MotorMode::Speed;
                cmd.eps_mode = EpsMode::FrontWheel;
                cmd.brake_mode = BrakeMode::Pressure;
                cmd.target_deceleration_mps2 = (-msg.longitudinal.acceleration).max(0.0);
                cmd.last_control_at = Some(Instant::now());
            })?
        };

        let sub_gear = {
            let state = Arc::clone(&state);
            node.create_subscription("~/input/gear_cmd", move |msg: GearCommand| {
                let mut cmd = state.command.lock();
                cmd.gear = autoware_gear_to_dbc(msg.command);
            })?
        };

        let sub_turn = {
            let state = Arc::clone(&state);
            node.create_subscription(
                "~/input/turn_indicators_cmd",
                move |msg: TurnIndicatorsCommand| {
                    let mut cmd = state.command.lock();
                    if !matches!(cmd.blinker, BlinkerCtrl::Hazard) {
                        cmd.blinker = match msg.command {
                            TurnIndicatorsCommand::ENABLE_LEFT => BlinkerCtrl::Left,
                            TurnIndicatorsCommand::ENABLE_RIGHT => BlinkerCtrl::Right,
                            _ => BlinkerCtrl::Off,
                        };
                    }
                },
            )?
        };

        let sub_hazard = {
            let state = Arc::clone(&state);
            node.create_subscription(
                "~/input/hazard_lights_cmd",
                move |msg: HazardLightsCommand| {
                    let mut cmd = state.command.lock();
                    if msg.command == HazardLightsCommand::ENABLE {
                        cmd.blinker = BlinkerCtrl::Hazard;
                    } else if matches!(cmd.blinker, BlinkerCtrl::Hazard) {
                        cmd.blinker = BlinkerCtrl::Off;
                    }
                },
            )?
        };

        // ----- Publishers: CAN status → Autoware reports --------------------
        let pub_velocity = node.create_publisher::<VelocityReport>("~/output/velocity_status")?;
        let pub_steering = node.create_publisher::<SteeringReport>("~/output/steering_status")?;
        let pub_gear = node.create_publisher::<GearReport>("~/output/gear_status")?;
        let pub_mode = node.create_publisher::<ControlModeReport>("~/output/control_mode")?;
        let pub_turn =
            node.create_publisher::<TurnIndicatorsReport>("~/output/turn_indicators_status")?;
        let pub_hazard =
            node.create_publisher::<HazardLightsReport>("~/output/hazard_lights_status")?;

        // ----- Service: ControlModeCommand → CAN auto_enable ----------------
        let srv_mode = {
            let state = Arc::clone(&state);
            node.create_service::<ControlModeCommand, _>(
                "~/input/control_mode_request",
                move |req: ControlModeCommand_Request, _info: ServiceInfo| {
                    let enable = matches!(
                        req.mode,
                        ControlModeCommand_Request::AUTONOMOUS
                            | ControlModeCommand_Request::AUTONOMOUS_STEER_ONLY
                            | ControlModeCommand_Request::AUTONOMOUS_VELOCITY_ONLY
                    );
                    let mut cmd = state.command.lock();
                    cmd.auto_enabled = enable;
                    if !enable {
                        cmd.target_speed_mps = 0.0;
                        cmd.target_acceleration_mps2 = 0.0;
                        cmd.target_tire_angle_rad = 0.0;
                    }
                    log_info!(
                        NODE_NAME,
                        "ControlMode request: mode={} -> auto_enabled={}",
                        req.mode,
                        enable
                    );
                    ControlModeCommand_Response { success: true }
                },
            )?
        };

        // ----- Publish timer: read status, emit Autoware reports ------------
        let publish_period = Duration::from_secs_f64(1.0 / params.publish_rate_hz);
        let frame_id = params.frame_id.clone();
        let timer_state = Arc::clone(&state);
        let timer_pubs = Publishers {
            velocity: pub_velocity.clone(),
            steering: pub_steering.clone(),
            gear: pub_gear.clone(),
            mode: pub_mode.clone(),
            turn: pub_turn.clone(),
            hazard: pub_hazard.clone(),
        };
        let publish_timer = node.create_timer_repeating(publish_period, move || {
            publish_status(&timer_state, &timer_pubs, &frame_id);
        })?;

        log_info!(
            NODE_NAME,
            "vehicle interface ready (can={}, tx={}Hz, pub={}Hz)",
            params.can_interface,
            params.tx_rate_hz,
            params.publish_rate_hz
        );

        Ok(Self {
            sub_control,
            sub_gear,
            sub_turn,
            sub_hazard,
            pub_velocity,
            pub_steering,
            pub_gear,
            pub_mode,
            pub_turn,
            pub_hazard,
            srv_mode,
            publish_timer,
        })
    }
}

struct Publishers {
    velocity: Publisher<VelocityReport>,
    steering: Publisher<SteeringReport>,
    gear: Publisher<GearReport>,
    mode: Publisher<ControlModeReport>,
    turn: Publisher<TurnIndicatorsReport>,
    hazard: Publisher<HazardLightsReport>,
}

fn publish_status(state: &Arc<SharedState>, pubs: &Publishers, frame_id: &str) {
    let status = *state.status.lock();
    let stamp = now_stamp();

    if let Some(mtr) = status.mtr {
        let header = std_msgs::msg::Header {
            stamp: stamp.clone(),
            frame_id: frame_id.to_string(),
        };
        let _ = pubs.velocity.publish(VelocityReport {
            header,
            longitudinal_velocity: mtr.speed_mps,
            lateral_velocity: 0.0,
            heading_rate: 0.0,
        });
        let _ = pubs.gear.publish(GearReport {
            stamp: stamp.clone(),
            report: dbc_gear_to_autoware(mtr.gear),
        });
    }

    if let Some(eps) = status.eps {
        let _ = pubs.steering.publish(SteeringReport {
            stamp: stamp.clone(),
            steering_tire_angle: eps.tire_angle_deg.to_radians(),
        });
    }

    if let Some(veh) = status.veh {
        let _ = pubs.mode.publish(ControlModeReport {
            stamp: stamp.clone(),
            mode: dbc_state_to_control_mode(veh.driving_state),
        });
        let (turn, hazard) = dbc_blinker_to_autoware(veh.blinker);
        let _ = pubs.turn.publish(TurnIndicatorsReport {
            stamp: stamp.clone(),
            report: turn,
        });
        let _ = pubs.hazard.publish(HazardLightsReport {
            stamp,
            report: hazard,
        });
    }
}

fn now_stamp() -> Time {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    Time {
        sec: dur.as_secs() as i32,
        nanosec: dur.subsec_nanos(),
    }
}

fn autoware_gear_to_dbc(cmd: u8) -> Gear {
    match cmd {
        GearCommand::DRIVE
        | GearCommand::DRIVE_2
        | GearCommand::DRIVE_3
        | GearCommand::DRIVE_4
        | GearCommand::DRIVE_5
        | GearCommand::DRIVE_6
        | GearCommand::DRIVE_7
        | GearCommand::DRIVE_8
        | GearCommand::DRIVE_9
        | GearCommand::LOW
        | GearCommand::LOW_2 => Gear::Drive,
        GearCommand::REVERSE | GearCommand::REVERSE_2 => Gear::Reverse,
        GearCommand::NEUTRAL => Gear::Neutral,
        _ => Gear::Parking,
    }
}

fn dbc_gear_to_autoware(g: Gear) -> u8 {
    match g {
        Gear::Drive => GearReport::DRIVE,
        Gear::Reverse => GearReport::REVERSE,
        Gear::Neutral => GearReport::NEUTRAL,
        Gear::Parking => GearReport::PARK,
    }
}

fn dbc_state_to_control_mode(s: SubsystemState) -> u8 {
    match s {
        SubsystemState::Autonomous => ControlModeReport::AUTONOMOUS,
        SubsystemState::Manual | SubsystemState::RemoteControl => ControlModeReport::MANUAL,
        SubsystemState::Invalid => ControlModeReport::NOT_READY,
    }
}

fn dbc_blinker_to_autoware(b: u8) -> (u8, u8) {
    match b {
        1 => (TurnIndicatorsReport::ENABLE_LEFT, HazardLightsReport::DISABLE),
        2 => (TurnIndicatorsReport::ENABLE_RIGHT, HazardLightsReport::DISABLE),
        3 => (TurnIndicatorsReport::DISABLE, HazardLightsReport::ENABLE),
        _ => (TurnIndicatorsReport::DISABLE, HazardLightsReport::DISABLE),
    }
}
