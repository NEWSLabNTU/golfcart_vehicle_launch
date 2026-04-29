//! ROS node wiring: subscriptions/publishers/service backed by `SharedState`.

use anyhow::Result;
use autoware_control_msgs::msg::Control;
use autoware_vehicle_msgs::msg::{
    ControlModeReport, GearCommand, GearReport, HazardLightsCommand, HazardLightsReport,
    SteeringReport, TurnIndicatorsCommand, TurnIndicatorsReport, VelocityReport,
};
use autoware_vehicle_msgs::srv::{ControlModeCommand, ControlModeCommand_Request, ControlModeCommand_Response};
use builtin_interfaces::msg::Time;
use diagnostic_msgs::msg::{DiagnosticArray, DiagnosticStatus, KeyValue};
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
    sub_estop: Subscription<std_msgs::msg::Bool>,
    pub_velocity: Publisher<VelocityReport>,
    pub_steering: Publisher<SteeringReport>,
    pub_gear: Publisher<GearReport>,
    pub_mode: Publisher<ControlModeReport>,
    pub_turn: Publisher<TurnIndicatorsReport>,
    pub_hazard: Publisher<HazardLightsReport>,
    pub_diag: Publisher<DiagnosticArray>,
    srv_mode: Service<ControlModeCommand>,
    publish_timer: Timer,
    diag_timer: Timer,
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

        // Driver/operator e-stop (separate from the ECU-side estop signal).
        // Setting `cmd.estop = true` trips SafetyBrake the same way an ECU
        // hazard does. Releasing (false) clears the driver e-stop, but a
        // latched ECU fault stays latched until a MANUAL/NO_COMMAND request.
        let sub_estop = {
            let state = Arc::clone(&state);
            node.create_subscription(
                "~/input/emergency_stop",
                move |msg: std_msgs::msg::Bool| {
                    let mut cmd = state.command.lock();
                    if msg.data && !cmd.estop {
                        log_info!(NODE_NAME, "driver e-stop ENGAGED");
                    } else if !msg.data && cmd.estop {
                        log_info!(NODE_NAME, "driver e-stop released");
                    }
                    cmd.estop = msg.data;
                    if msg.data {
                        cmd.auto_enabled = false;
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
        // /diagnostics is the standard ROS health channel; Autoware's
        // system_error_monitor consumes these and rolls up into HazardStatus.
        let pub_diag = node.create_publisher::<DiagnosticArray>("/diagnostics")?;

        // ----- Service: ControlModeCommand → CAN auto_enable ----------------
        let srv_mode = {
            let state = Arc::clone(&state);
            node.create_service::<ControlModeCommand, _>(
                "~/input/control_mode_request",
                move |req: ControlModeCommand_Request, _info: ServiceInfo| {
                    let want_engage = matches!(
                        req.mode,
                        ControlModeCommand_Request::AUTONOMOUS
                            | ControlModeCommand_Request::AUTONOMOUS_STEER_ONLY
                            | ControlModeCommand_Request::AUTONOMOUS_VELOCITY_ONLY
                    );
                    let mut cmd = state.command.lock();

                    if want_engage {
                        // Refuse if a previous fault was latched OR the ECU is
                        // currently reporting any hazard. Caller must first
                        // request MANUAL/NO_COMMAND to clear, after addressing
                        // the underlying issue.
                        let live_fault = state
                            .status
                            .lock()
                            .veh
                            .map(|v| {
                                v.vcu_ads_estop_raw()
                                    || v.vcu_ads_error_code_sys_raw()
                                    || v.vcu_ads_error_code_mtr_raw()
                                    || v.vcu_ads_error_code_eps_raw()
                                    || v.vcu_ads_error_code_brk_raw()
                            })
                            .unwrap_or(false);
                        if cmd.fault_latched || live_fault {
                            log_info!(
                                NODE_NAME,
                                "ControlMode AUTONOMOUS rejected: fault_latched={} live_fault={}",
                                cmd.fault_latched,
                                live_fault
                            );
                            return ControlModeCommand_Response { success: false };
                        }
                        cmd.auto_enabled = true;
                    } else {
                        // Disengage path also clears the latch so the user can
                        // re-engage after fixing the issue.
                        cmd.auto_enabled = false;
                        cmd.fault_latched = false;
                        cmd.target_speed_mps = 0.0;
                        cmd.target_acceleration_mps2 = 0.0;
                        cmd.target_tire_angle_rad = 0.0;
                    }
                    log_info!(
                        NODE_NAME,
                        "ControlMode request: mode={} -> auto_enabled={} fault_latched={}",
                        req.mode,
                        cmd.auto_enabled,
                        cmd.fault_latched
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

        // Diagnostics: 1 Hz roll-up of subsystem health for system_error_monitor.
        let diag_state = Arc::clone(&state);
        let diag_pub = pub_diag.clone();
        let diag_timer = node.create_timer_repeating(Duration::from_secs(1), move || {
            publish_diagnostics(&diag_state, &diag_pub);
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
            sub_estop,
            pub_velocity,
            pub_steering,
            pub_gear,
            pub_mode,
            pub_turn,
            pub_hazard,
            pub_diag,
            srv_mode,
            publish_timer,
            diag_timer,
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
    let cmd = *state.command.lock();
    let stamp = now_stamp();

    if let Some(mtr) = status.mtr {
        let header = std_msgs::msg::Header {
            stamp: stamp.clone(),
            frame_id: frame_id.to_string(),
        };
        let _ = pubs.velocity.publish(VelocityReport {
            header,
            longitudinal_velocity: mtr.vcu_ads_vehicle_speed(),
            lateral_velocity: 0.0,
            heading_rate: 0.0,
        });
        let _ = pubs.gear.publish(GearReport {
            stamp: stamp.clone(),
            report: dbc_gear_to_autoware(Gear::from_raw(mtr.vcu_ads_gear_position_raw())),
        });
    }

    if let Some(eps) = status.eps {
        let _ = pubs.steering.publish(SteeringReport {
            stamp: stamp.clone(),
            steering_tire_angle: eps.vcu_ads_tire_angle().to_radians(),
        });
    }

    // ControlModeReport: respect local intent, not just what the VCU echoes.
    // A fault latch or user disengage MUST surface as DISENGAGED so Autoware
    // can react regardless of the VCU's still-cached Autonomous state.
    let mode = if cmd.fault_latched {
        ControlModeReport::DISENGAGED
    } else if !cmd.auto_enabled {
        ControlModeReport::MANUAL
    } else if let Some(veh) = status.veh {
        dbc_state_to_control_mode(SubsystemState::from_raw(veh.vcu_ads_driving_state_raw()))
    } else {
        ControlModeReport::NOT_READY
    };
    let _ = pubs.mode.publish(ControlModeReport {
        stamp: stamp.clone(),
        mode,
    });

    if let Some(veh) = status.veh {
        let (turn, hazard) = dbc_blinker_to_autoware(veh.vcu_ads_blinker_raw());
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

/// Publish a DiagnosticArray summarising vehicle-interface health. Autoware's
/// `system_error_monitor` aggregates `/diagnostics` across the system into
/// `HazardStatus`, which drives MRM (Minimal Risk Maneuver). We emit one
/// status per logical subsystem so a single ECU fault doesn't poison the
/// whole interface's health.
fn publish_diagnostics(state: &Arc<SharedState>, publisher: &Publisher<DiagnosticArray>) {
    let cmd = *state.command.lock();
    let status = *state.status.lock();
    let stamp = now_stamp();

    let header = std_msgs::msg::Header {
        stamp: stamp.clone(),
        frame_id: String::new(),
    };

    let kv = |k: &str, v: String| KeyValue {
        key: k.to_string(),
        value: v,
    };

    let mut entries: Vec<DiagnosticStatus> = Vec::new();

    // Per-subsystem error bits straight from VCU.
    if let Some(veh) = status.veh {
        for (name, faulted) in [
            ("vehicle_interface/system", veh.vcu_ads_error_code_sys_raw()),
            ("vehicle_interface/motor", veh.vcu_ads_error_code_mtr_raw()),
            ("vehicle_interface/eps", veh.vcu_ads_error_code_eps_raw()),
            ("vehicle_interface/brake", veh.vcu_ads_error_code_brk_raw()),
        ] {
            entries.push(DiagnosticStatus {
                level: if faulted {
                    DiagnosticStatus::ERROR
                } else {
                    DiagnosticStatus::OK
                },
                name: name.to_string(),
                message: if faulted {
                    "ECU reports subsystem fault".to_string()
                } else {
                    "ok".to_string()
                },
                hardware_id: "cax_ads_can".to_string(),
                values: vec![],
            });
        }

        let estop_active = veh.vcu_ads_estop_raw() || cmd.estop;
        entries.push(DiagnosticStatus {
            level: if estop_active {
                DiagnosticStatus::ERROR
            } else {
                DiagnosticStatus::OK
            },
            name: "vehicle_interface/estop".to_string(),
            message: match (veh.vcu_ads_estop_raw(), cmd.estop) {
                (true, true) => "ECU + driver e-stop active".to_string(),
                (true, false) => "ECU e-stop active".to_string(),
                (false, true) => "driver e-stop active".to_string(),
                _ => "ok".to_string(),
            },
            hardware_id: "cax_ads_can".to_string(),
            values: vec![],
        });
    } else {
        entries.push(DiagnosticStatus {
            level: DiagnosticStatus::STALE,
            name: "vehicle_interface/system".to_string(),
            message: "no VCU_ADS_VEHICLE frame received yet".to_string(),
            hardware_id: "cax_ads_can".to_string(),
            values: vec![],
        });
    }

    // Top-level rollup with operational context the monitor can show.
    let overall_faulted = cmd.fault_latched
        || cmd.estop
        || status
            .veh
            .map(|v| {
                v.vcu_ads_error_code_sys_raw()
                    || v.vcu_ads_error_code_mtr_raw()
                    || v.vcu_ads_error_code_eps_raw()
                    || v.vcu_ads_error_code_brk_raw()
                    || v.vcu_ads_estop_raw()
            })
            .unwrap_or(false);
    entries.push(DiagnosticStatus {
        level: if overall_faulted {
            DiagnosticStatus::ERROR
        } else {
            DiagnosticStatus::OK
        },
        name: "vehicle_interface".to_string(),
        message: if cmd.fault_latched {
            "fault latched; engage will be refused until MANUAL/NO_COMMAND request"
                .to_string()
        } else if overall_faulted {
            "active fault".to_string()
        } else {
            "ok".to_string()
        },
        hardware_id: "cax_ads_can".to_string(),
        values: vec![
            kv("auto_enabled", cmd.auto_enabled.to_string()),
            kv("fault_latched", cmd.fault_latched.to_string()),
            kv("driver_estop", cmd.estop.to_string()),
            kv(
                "speed_mps",
                status
                    .mtr
                    .map(|m| format!("{:.3}", m.vcu_ads_vehicle_speed()))
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
            kv(
                "vcu_driving_state",
                status
                    .veh
                    .map(|v| v.vcu_ads_driving_state_raw().to_string())
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
        ],
    });

    let _ = publisher.publish(DiagnosticArray {
        header,
        status: entries,
    });
}

fn dbc_blinker_to_autoware(b: u8) -> (u8, u8) {
    match b {
        1 => (TurnIndicatorsReport::ENABLE_LEFT, HazardLightsReport::DISABLE),
        2 => (TurnIndicatorsReport::ENABLE_RIGHT, HazardLightsReport::DISABLE),
        3 => (TurnIndicatorsReport::DISABLE, HazardLightsReport::ENABLE),
        _ => (TurnIndicatorsReport::DISABLE, HazardLightsReport::DISABLE),
    }
}
