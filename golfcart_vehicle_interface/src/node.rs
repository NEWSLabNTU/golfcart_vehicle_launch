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
use tier4_vehicle_msgs::msg::{ActuationStatus, ActuationStatusStamped, VehicleEmergencyStamped};
use rclrs::{
    Clock, Node, Publisher, QoSProfile, Service, ServiceInfo, SubscriptionOptions, Subscription,
    Timer, log_info,
};
use std::sync::Arc;
use std::time::{Duration, Instant};

use crate::dbc::{
    BlinkerCtrl, BrakeMode, EpsMode, Gear, MotorMode, ACCEL_MPS2_MAX,
    ACCEL_MPS2_MIN, DECEL_MPS2_MAX, DECEL_MPS2_MIN, SPEED_MPS_MAX,
    TIRE_ANGLE_DEG_MAX, TIRE_ANGLE_DEG_MIN,
};
use crate::params::Params;
use crate::state::{EstopSource, EstopState, SharedState, VehicleMode};

const NODE_NAME: &str = "golfcart_vehicle_interface";

/// ROOTS `Ads_Vcu_Target_Speed` is specified for [0, 25] km/h. Direction is
/// selected by the gear, not the sign of this signal, so the speed setpoint is
/// always a non-negative magnitude capped at this ceiling (25 km/h ≈ 6.944 m/s).
const ROOTS_MAX_SPEED_MPS: f32 = 6.944;

#[allow(dead_code)] // Held to keep ROS entities alive for the node's lifetime.
pub struct VehicleInterfaceNode {
    sub_control: Subscription<Control>,
    sub_gear: Subscription<GearCommand>,
    sub_turn: Subscription<TurnIndicatorsCommand>,
    sub_hazard: Subscription<HazardLightsCommand>,
    sub_estop: Subscription<std_msgs::msg::Bool>,
    sub_emergency: Subscription<VehicleEmergencyStamped>,
    pub_velocity: Publisher<VelocityReport>,
    pub_steering: Publisher<SteeringReport>,
    pub_gear: Publisher<GearReport>,
    pub_mode: Publisher<ControlModeReport>,
    pub_turn: Publisher<TurnIndicatorsReport>,
    pub_hazard: Publisher<HazardLightsReport>,
    pub_actuation: Publisher<ActuationStatusStamped>,
    pub_diag: Publisher<DiagnosticArray>,
    srv_mode: Service<ControlModeCommand>,
    publish_timer: Timer,
    diag_timer: Timer,
}

impl VehicleInterfaceNode {
    pub fn new(node: &Node, params: &Params, state: Arc<SharedState>) -> Result<Self> {
        // ----- Subscriptions: Autoware → CAN command state ------------------
        // Command topics use KeepLast(1) Reliable: drop superseded commands,
        // guarantee delivery while connected. Matches pacmod_interface and
        // Autoware vehicle_cmd_gate publisher QoS — silent QoS mismatch
        // (e.g. publisher BEST_EFFORT vs sub Reliable) results in zero
        // messages delivered, which is hard to diagnose in the field.
        let cmd_qos = QoSProfile::topics_default().keep_last(1).reliable();
        let sub_opts = |topic: &'static str| {
            let mut opts = SubscriptionOptions::new(topic);
            opts.qos = cmd_qos;
            opts
        };
        let max_speed = params.max_speed_mps;
        let max_accel = params.max_accel_mps2;
        let max_decel = params.max_decel_mps2;
        let decel_deadband = params.decel_deadband_mps2;
        let max_tire = params.max_tire_angle_rad;
        let sub_control = {
            let state = Arc::clone(&state);
            node.create_subscription(sub_opts("~/input/control_cmd"), move |msg: Control| {
                // Two-stage saturation: first apply user-policy caps (low-speed
                // commissioning, mechanical EPS limits), then clamp to DBC
                // ranges so the encode call never errors. Out-of-range setpoints
                // would otherwise tear down the vehicle interface mid-drive.
                // ROOTS target speed is a magnitude in [0, 25] km/h; direction
                // comes from the gear, not the sign. Take |velocity|, then clamp
                // to the user-policy cap, the ROOTS 25 km/h ceiling, and finally
                // the DBC signal max so the encode call never errors.
                let speed_cap = max_speed.min(ROOTS_MAX_SPEED_MPS).min(SPEED_MPS_MAX);
                let speed = clamp_f32((msg.longitudinal.velocity as f32).abs(), 0.0, speed_cap);

                let Longitudinal { accel, decel } = split_longitudinal(
                    msg.longitudinal.acceleration as f32,
                    max_accel,
                    max_decel,
                    decel_deadband,
                );

                let tire_user = clamp_f32(
                    msg.lateral.steering_tire_angle as f32,
                    -max_tire,
                    max_tire,
                );
                let tire_rad = clamp_f32(
                    tire_user.to_degrees(),
                    TIRE_ANGLE_DEG_MIN,
                    TIRE_ANGLE_DEG_MAX,
                )
                .to_radians();

                let mut cmd = state.command.lock();
                cmd.target_speed_mps = speed;
                cmd.target_acceleration_mps2 = accel;
                cmd.target_tire_angle_rad = tire_rad;
                cmd.motor_mode = MotorMode::Speed;
                cmd.eps_mode = EpsMode::FrontWheel;
                cmd.brake_mode = BrakeMode::Pressure;
                cmd.target_deceleration_mps2 = decel;
                let now = Instant::now();
                cmd.control_freq.record(now);
                cmd.last_control_at = Some(now);
            })?
        };

        let sub_gear = {
            let state = Arc::clone(&state);
            node.create_subscription(sub_opts("~/input/gear_cmd"), move |msg: GearCommand| {
                let mut cmd = state.command.lock();
                cmd.gear = autoware_gear_to_dbc(msg.command);
            })?
        };

        let sub_turn = {
            let state = Arc::clone(&state);
            node.create_subscription(
                sub_opts("~/input/turn_indicators_cmd"),
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
                sub_opts("~/input/hazard_lights_cmd"),
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

        // E-stop requests, one flag per source (see `EstopState`). Either
        // source engaging trips SafetyBrake the same way an ECU hazard does;
        // a release clears only that source's own flag, so the effective
        // e-stop drops only once both have released. A latched ECU fault
        // stays latched regardless, until a MANUAL/NO_COMMAND request.
        //   * `~/input/emergency_stop` (`std_msgs::Bool`): the driver's
        //     button or a hand-rolled caller.
        //   * `~/input/emergency_cmd` (`VehicleEmergencyStamped`): Autoware's
        //     vehicle_cmd_gate. It publishes `emergency=false` on every
        //     control cycle, which is why the two cannot share one flag: a
        //     shared flag let the gate's next routine message clear a driver
        //     press within one cycle.
        let sub_estop = {
            let state = Arc::clone(&state);
            node.create_subscription(
                sub_opts("~/input/emergency_stop"),
                move |msg: std_msgs::msg::Bool| {
                    let mut cmd = state.command.lock();
                    log_estop_transition(
                        EstopSource::Driver,
                        cmd.estop.set(EstopSource::Driver, msg.data),
                        &cmd.estop,
                    );
                },
            )?
        };

        let sub_emergency = {
            let state = Arc::clone(&state);
            node.create_subscription(
                sub_opts("~/input/emergency_cmd"),
                move |msg: VehicleEmergencyStamped| {
                    let mut cmd = state.command.lock();
                    log_estop_transition(
                        EstopSource::Mrm,
                        cmd.estop.set(EstopSource::Mrm, msg.emergency),
                        &cmd.estop,
                    );
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
        let pub_actuation =
            node.create_publisher::<ActuationStatusStamped>("~/output/actuation_status")?;
        // /diagnostics is the standard ROS health channel; Autoware's
        // system_error_monitor consumes these and rolls up into HazardStatus.
        let pub_diag = node.create_publisher::<DiagnosticArray>("/diagnostics")?;

        // ----- Service: ControlModeCommand ---------------------------------
        // Mode transitions are the driver's, made on the vehicle's own
        // controls; this service cannot engage anything. An AUTONOMOUS request
        // is therefore only an acknowledgement — it succeeds when the VCU
        // already reports all four subsystems autonomous and no fault is
        // latched, and fails otherwise, so Autoware's engage flow blocks until
        // the driver has actually handed the vehicle over. MANUAL /
        // NO_COMMAND stays useful: it clears the fault latch and zeroes the
        // held setpoints.
        let report_timeout_srv = Duration::from_millis(params.report_timeout_ms);
        let srv_mode = {
            let state = Arc::clone(&state);
            node.create_service::<ControlModeCommand, _>(
                "~/input/control_mode_request",
                move |req: ControlModeCommand_Request, _info: ServiceInfo| {
                    // Only full AUTONOMOUS is implemented. STEER_ONLY and
                    // VELOCITY_ONLY would require gating individual *_en bits;
                    // accepting them while sending all enables would be a lie
                    // to Autoware's MRM about the actual handover scope.
                    let want_engage =
                        matches!(req.mode, ControlModeCommand_Request::AUTONOMOUS);
                    let want_disengage = matches!(
                        req.mode,
                        ControlModeCommand_Request::MANUAL
                            | ControlModeCommand_Request::NO_COMMAND
                    );
                    if matches!(
                        req.mode,
                        ControlModeCommand_Request::AUTONOMOUS_STEER_ONLY
                            | ControlModeCommand_Request::AUTONOMOUS_VELOCITY_ONLY
                    ) {
                        log_info!(
                            NODE_NAME,
                            "ControlMode partial-autonomy rejected: mode={} not implemented",
                            req.mode
                        );
                        return ControlModeCommand_Response { success: false };
                    }
                    if !want_engage && !want_disengage {
                        log_info!(
                            NODE_NAME,
                            "ControlMode request rejected: unknown mode={}",
                            req.mode
                        );
                        return ControlModeCommand_Response { success: false };
                    }
                    let mut cmd = state.command.lock();

                    if want_engage {
                        // Nothing to set: the driver switches the vehicle over.
                        // Report whether it is already there and unfaulted.
                        let vehicle_mode =
                            state.status.lock().vehicle_mode(report_timeout_srv);
                        let engaged = vehicle_mode == VehicleMode::Autonomous
                            && !cmd.fault_latched;
                        if !engaged {
                            log_info!(
                                NODE_NAME,
                                "ControlMode AUTONOMOUS not acknowledged: vehicle_mode={:?} \
                                 fault_latched={} — mode changes are made by the driver on \
                                 the vehicle, not over this service",
                                vehicle_mode,
                                cmd.fault_latched
                            );
                        }
                        return ControlModeCommand_Response { success: engaged };
                    }

                    // MANUAL / NO_COMMAND: clear the latch so the vehicle can
                    // be driven autonomously again once the driver hands it
                    // back, and drop any held setpoints.
                    cmd.fault_latched = false;
                    cmd.target_speed_mps = 0.0;
                    cmd.target_acceleration_mps2 = 0.0;
                    cmd.target_tire_angle_rad = 0.0;
                    log_info!(
                        NODE_NAME,
                        "ControlMode request: mode={} -> fault latch cleared, setpoints zeroed",
                        req.mode
                    );
                    ControlModeCommand_Response { success: true }
                },
            )?
        };

        // ----- Publish timer: read status, emit Autoware reports ------------
        let publish_period = Duration::from_secs_f64(1.0 / params.publish_rate_hz);
        let steering_sign = steering_sign(params);
        let wheel_base = params.wheel_base;
        let control_min_rate_hz = params.control_min_rate_hz;
        let report_timeout = Duration::from_millis(params.report_timeout_ms);
        let frame_id = params.frame_id.clone();
        let timer_state = Arc::clone(&state);
        let timer_pubs = Publishers {
            velocity: pub_velocity.clone(),
            steering: pub_steering.clone(),
            gear: pub_gear.clone(),
            mode: pub_mode.clone(),
            turn: pub_turn.clone(),
            hazard: pub_hazard.clone(),
            actuation: pub_actuation.clone(),
        };
        let timer_clock = node.get_clock();
        let publish_timer = node.create_timer_repeating(publish_period, move || {
            publish_status(
                &timer_state,
                &timer_pubs,
                &frame_id,
                &timer_clock,
                report_timeout,
                steering_sign,
                wheel_base,
            );
        })?;

        // Diagnostics: 1 Hz roll-up of subsystem health for system_error_monitor.
        let diag_state = Arc::clone(&state);
        let diag_pub = pub_diag.clone();
        let diag_clock = node.get_clock();
        let diag_timer = node.create_timer_repeating(Duration::from_secs(1), move || {
            publish_diagnostics(
                &diag_state,
                &diag_pub,
                &diag_clock,
                report_timeout,
                control_min_rate_hz,
            );
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
            sub_emergency,
            pub_velocity,
            pub_steering,
            pub_gear,
            pub_mode,
            pub_turn,
            pub_hazard,
            pub_actuation,
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
    actuation: Publisher<ActuationStatusStamped>,
}

/// +1.0 or -1.0, from the `invert_steering` parameter. Autoware counts a
/// positive tire angle to the left (REP-103), ROOTS to the right, so the value
/// changes sign at the CAN boundary in both directions.
pub fn steering_sign(params: &Params) -> f32 {
    if params.invert_steering {
        -1.0
    } else {
        1.0
    }
}

/// Acceleration and deceleration setpoints, both non-negative magnitudes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Longitudinal {
    pub accel: f32,
    pub decel: f32,
}

/// Split Autoware's signed longitudinal acceleration into the VCU's separate
/// `Target_Acceleration` and `Target_Deceleration` setpoints.
///
/// Two-stage saturation: user-policy caps first, then the DBC signal ranges so
/// the encode call never errors.
pub fn split_longitudinal(
    accel_signed: f32,
    max_accel: f32,
    max_decel: f32,
    decel_deadband: f32,
) -> Longitudinal {
    let accel_user = clamp_f32(accel_signed.max(0.0), 0.0, max_accel);
    let accel = clamp_f32(accel_user, ACCEL_MPS2_MIN, ACCEL_MPS2_MAX);
    // Any deceleration > 0 cuts motor torque (the VCU's decel priority, which
    // `build_frames` mirrors), but ROOTS does not brake below 1.2 m/s². A
    // small negative from PID dither at cruise would therefore only switch
    // the motor off, not slow the cart; inside the deadband send none, keep
    // the motor on, and let the speed-mode target do the slowing. Values
    // beyond it pass through unchanged: lifting them into the 1.2 m/s²
    // brake stage is a tuning decision that needs the vehicle.
    let decel_requested = (-accel_signed).max(0.0);
    let decel_requested = if decel_requested < decel_deadband {
        0.0
    } else {
        decel_requested
    };
    let decel_user = clamp_f32(decel_requested, 0.0, max_decel);
    let decel = clamp_f32(decel_user, DECEL_MPS2_MIN, DECEL_MPS2_MAX);
    Longitudinal { accel, decel }
}

/// Yaw rate (rad/s) for `VelocityReport.heading_rate`, from the kinematic
/// bicycle model: `v * tan(delta) / wheel_base`.
///
/// `tire_angle_rad` must already be in Autoware's frame (positive = left, after
/// `invert_steering`), so a left turn going forward gives a positive yaw rate,
/// matching REP-103 and the `SteeringReport` published beside it. The signed
/// velocity carries reverse: backing up with the wheels left turns the nose
/// right. `None` means the EPS report is stale; the rate is then 0.0 rather than
/// a guess from an angle nobody has confirmed recently.
pub fn heading_rate(velocity_mps: f32, tire_angle_rad: Option<f32>, wheel_base_m: f32) -> f32 {
    match tire_angle_rad {
        Some(delta) => velocity_mps * delta.tan() / wheel_base_m,
        None => 0.0,
    }
}

fn publish_status(
    state: &Arc<SharedState>,
    pubs: &Publishers,
    frame_id: &str,
    clock: &Clock,
    report_timeout: Duration,
    steering_sign: f32,
    wheel_base: f32,
) {
    let status = *state.status.lock();
    let cmd = *state.command.lock();
    let stamp = now_stamp(clock);
    let fresh = |at: Option<Instant>| at.map_or(false, |t| t.elapsed() <= report_timeout);
    let mtr_fresh = status.mtr.filter(|_| fresh(status.mtr_at));
    let eps_fresh = status.eps.filter(|_| fresh(status.eps_at));
    let brk_fresh = status.brk.filter(|_| fresh(status.brk_at));
    let veh_fresh = status.veh.filter(|_| fresh(status.veh_at));

    // Tire angle in Autoware's REP-103 frame (positive = left); the VCU
    // counts the other way - see `invert_steering`. Shared by SteeringReport
    // and the heading rate so the two can never disagree on sign.
    let tire_angle_rad = eps_fresh.map(|eps| steering_sign * eps.vcu_ads_tire_angle().to_radians());

    if let Some(mtr) = mtr_fresh {
        let header = std_msgs::msg::Header {
            stamp: stamp.clone(),
            frame_id: frame_id.to_string(),
        };
        let velocity = mtr.vcu_ads_vehicle_speed();
        let _ = pubs.velocity.publish(VelocityReport {
            header,
            longitudinal_velocity: velocity,
            lateral_velocity: 0.0,
            // The VCU reports no yaw rate, so derive one from the steering.
            // Consumers such as vehicle_velocity_converter pass it on as the
            // twist's angular z; a hardcoded 0.0 told them the cart never turns.
            heading_rate: heading_rate(velocity, tire_angle_rad, wheel_base),
        });
        let _ = pubs.gear.publish(GearReport {
            stamp: stamp.clone(),
            report: dbc_gear_to_autoware(Gear::from_raw(mtr.vcu_ads_gear_position_raw())),
        });
    }

    if let Some(steering_tire_angle) = tire_angle_rad {
        let _ = pubs.steering.publish(SteeringReport {
            stamp: stamp.clone(),
            steering_tire_angle,
        });
    }

    // ControlModeReport mirrors the vehicle's own aggregate state: all four
    // VCU subsystems autonomous → AUTONOMOUS, all four manual → MANUAL,
    // anything else (mixed, Invalid, or a stale frame) → NOT_READY. A latched
    // fault still overrides to DISENGAGED: we refuse to command in that state,
    // so reporting AUTONOMOUS would lie to Autoware even if the VCU is willing.
    let mode = if cmd.fault_latched {
        ControlModeReport::DISENGAGED
    } else {
        match status.vehicle_mode(report_timeout) {
            VehicleMode::Autonomous => ControlModeReport::AUTONOMOUS,
            VehicleMode::Manual => ControlModeReport::MANUAL,
            VehicleMode::Abnormal => ControlModeReport::NOT_READY,
        }
    };
    let _ = pubs.mode.publish(ControlModeReport {
        stamp: stamp.clone(),
        mode,
    });

    if let Some(veh) = veh_fresh {
        let (turn, hazard) = dbc_blinker_to_autoware(veh.vcu_ads_blinker_raw());
        let _ = pubs.turn.publish(TurnIndicatorsReport {
            stamp: stamp.clone(),
            report: turn,
        });
        let _ = pubs.hazard.publish(HazardLightsReport {
            stamp: stamp.clone(),
            report: hazard,
        });
    }

    // ActuationStatusStamped: useful for closed-loop tuning of accel/brake/
    // steering. Only published when at least one underlying VCU report is
    // fresh — partial-feedback messages would mislead tuning tooling.
    if mtr_fresh.is_some() || brk_fresh.is_some() || eps_fresh.is_some() {
        let header = std_msgs::msg::Header {
            stamp,
            frame_id: frame_id.to_string(),
        };
        let accel_status = mtr_fresh
            .map(|m| m.vcu_ads_throttle_position_raw() as f64)
            .unwrap_or(0.0);
        // Brake feedback uses pressure (MPa) — Autoware-side calibration
        // converts to a normalised pedal effort.
        let brake_status = brk_fresh
            .map(|b| b.vcu_ads_brake_pressure() as f64)
            .unwrap_or(0.0);
        let steer_status = eps_fresh
            .map(|e| steering_sign as f64 * (e.vcu_ads_tire_angle() as f64).to_radians())
            .unwrap_or(0.0);
        let _ = pubs.actuation.publish(ActuationStatusStamped {
            header,
            status: ActuationStatus {
                accel_status,
                brake_status,
                steer_status,
            },
        });
    }
}

/// Log one source's e-stop transition. `changed` is what `EstopState::set`
/// returned; repeats are silent, since both topics are republished constantly.
fn log_estop_transition(source: EstopSource, changed: Option<bool>, estop: &EstopState) {
    let name = match source {
        EstopSource::Driver => "driver e-stop",
        EstopSource::Mrm => "MRM emergency_cmd",
    };
    match changed {
        Some(true) => log_info!(NODE_NAME, "{name} ENGAGED"),
        Some(false) if estop.active() => log_info!(
            NODE_NAME,
            "{name} released; e-stop still held (driver={} mrm={})",
            estop.driver(),
            estop.mrm()
        ),
        Some(false) => log_info!(NODE_NAME, "{name} released; no e-stop active"),
        None => {}
    }
}

fn now_stamp(clock: &Clock) -> Time {
    // Clock::now() returns rclrs::Time; convert to (sec, nanosec) and rebuild
    // as the project-vendored builtin_interfaces::msg::Time. Negative epoch
    // (only occurs if sim-time runs uninitialised) folds to zero.
    let (sec, nanosec) = clock.now().to_sec_nanosec().unwrap_or((0, 0));
    Time { sec, nanosec }
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

/// Publish a DiagnosticArray summarising vehicle-interface health. Autoware's
/// `system_error_monitor` aggregates `/diagnostics` across the system into
/// `HazardStatus`, which drives MRM (Minimal Risk Maneuver). We emit one
/// status per logical subsystem so a single ECU fault doesn't poison the
/// whole interface's health.
fn publish_diagnostics(
    state: &Arc<SharedState>,
    publisher: &Publisher<DiagnosticArray>,
    clock: &Clock,
    report_timeout: Duration,
    control_min_rate_hz: f32,
) {
    let cmd = *state.command.lock();
    let status = *state.status.lock();
    let stamp = now_stamp(clock);
    let veh_stale = status
        .veh_at
        .map_or(true, |t| t.elapsed() > report_timeout);
    let veh_fresh = if veh_stale { None } else { status.veh };

    let header = std_msgs::msg::Header {
        stamp: stamp.clone(),
        frame_id: String::new(),
    };

    let kv = |k: &str, v: String| KeyValue {
        key: k.to_string(),
        value: v,
    };

    let mut entries: Vec<DiagnosticStatus> = Vec::new();

    // CAN TX health: TX thread sets `tx_failed` once consecutive write
    // errors exceed its threshold. ERROR-level diag so MRM treats it as a
    // hardware fault.
    entries.push(DiagnosticStatus {
        level: if cmd.tx_failed {
            DiagnosticStatus::ERROR
        } else {
            DiagnosticStatus::OK
        },
        name: "vehicle_interface/can_tx".to_string(),
        message: if cmd.tx_failed {
            "persistent CAN TX failures — bus may be down".to_string()
        } else {
            "ok".to_string()
        },
        hardware_id: "cax_ads_can".to_string(),
        values: vec![],
    });

    // Control input rate: the TX path withholds the speed setpoint below
    // `control_min_rate_hz`, so surface why the cart is not moving. WARN, not
    // ERROR: a slow planner is a degraded input, not broken hardware.
    if control_min_rate_hz > 0.0 && cmd.last_control_at.is_some() {
        let rate = cmd.control_freq.rate_hz();
        let too_slow = rate.map_or(true, |hz| hz < control_min_rate_hz);
        entries.push(DiagnosticStatus {
            level: if too_slow {
                DiagnosticStatus::WARN
            } else {
                DiagnosticStatus::OK
            },
            name: "vehicle_interface/control_rate".to_string(),
            message: if too_slow {
                "Control input below minimum rate — speed setpoint held at 0".to_string()
            } else {
                "ok".to_string()
            },
            hardware_id: "cax_ads_can".to_string(),
            values: vec![
                kv(
                    "rate_hz",
                    rate.map_or_else(|| "unknown".to_string(), |hz| format!("{hz:.1}")),
                ),
                kv("min_rate_hz", format!("{control_min_rate_hz:.1}")),
            ],
        });
    }

    // Blinker echo check: stuck-blinker on CAN light controllers is a known
    // quirk. WARN if our last sent value disagrees with VCU echo for >500ms
    // after the change settled.
    if let Some(veh) = veh_fresh {
        let echo = veh.vcu_ads_blinker_raw();
        let sent = cmd.last_blinker_sent.to_raw();
        let mismatched = echo != sent;
        let dwell_ok = cmd
            .last_blinker_change_at
            .map_or(false, |t| t.elapsed() > Duration::from_millis(500));
        if mismatched && dwell_ok {
            entries.push(DiagnosticStatus {
                level: DiagnosticStatus::WARN,
                name: "vehicle_interface/blinker".to_string(),
                message: format!("blinker echo mismatch: sent={sent} echo={echo}"),
                hardware_id: "cax_ads_can".to_string(),
                values: vec![],
            });
        } else {
            entries.push(DiagnosticStatus {
                level: DiagnosticStatus::OK,
                name: "vehicle_interface/blinker".to_string(),
                message: "ok".to_string(),
                hardware_id: "cax_ads_can".to_string(),
                values: vec![],
            });
        }
    }

    // Per-subsystem error bits straight from VCU.
    if let Some(veh) = veh_fresh {
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

        let ecu_estop = veh.vcu_ads_estop_raw();
        let estop_active = ecu_estop || cmd.estop.active();
        entries.push(DiagnosticStatus {
            level: if estop_active {
                DiagnosticStatus::ERROR
            } else {
                DiagnosticStatus::OK
            },
            name: "vehicle_interface/estop".to_string(),
            message: {
                let active: Vec<&str> = [
                    (ecu_estop, "ECU"),
                    (cmd.estop.driver(), "driver"),
                    (cmd.estop.mrm(), "MRM"),
                ]
                .iter()
                .filter(|(on, _)| *on)
                .map(|(_, name)| *name)
                .collect();
                if active.is_empty() {
                    "ok".to_string()
                } else {
                    format!("{} e-stop active", active.join(" + "))
                }
            },
            hardware_id: "cax_ads_can".to_string(),
            values: vec![],
        });
    } else {
        let msg = if status.veh.is_some() {
            "VCU_ADS_VEHICLE frame stale (no recent receive)"
        } else {
            "no VCU_ADS_VEHICLE frame received yet"
        };
        entries.push(DiagnosticStatus {
            level: DiagnosticStatus::STALE,
            name: "vehicle_interface/system".to_string(),
            message: msg.to_string(),
            hardware_id: "cax_ads_can".to_string(),
            values: vec![],
        });
    }

    // Per-frame freshness for the non-veh status frames. Helps localise
    // an RX failure to a specific CAN ID rather than a blanket "VCU silent".
    for (name, at, present) in [
        ("vehicle_interface/frame_mtr", status.mtr_at, status.mtr.is_some()),
        ("vehicle_interface/frame_eps", status.eps_at, status.eps.is_some()),
        ("vehicle_interface/frame_brk", status.brk_at, status.brk.is_some()),
    ] {
        let stale = at.map_or(true, |t| t.elapsed() > report_timeout);
        let (level, message) = match (present, stale) {
            (false, _) => (DiagnosticStatus::STALE, "frame never received"),
            (true, true) => (DiagnosticStatus::STALE, "frame stale"),
            (true, false) => (DiagnosticStatus::OK, "ok"),
        };
        entries.push(DiagnosticStatus {
            level,
            name: name.to_string(),
            message: message.to_string(),
            hardware_id: "cax_ads_can".to_string(),
            values: vec![],
        });
    }

    // Top-level rollup with operational context the monitor can show. Stale
    // VCU report counts as a fault — Autoware should not assume operational
    // ECU state from a frozen snapshot.
    let overall_faulted = cmd.fault_latched
        || cmd.estop.active()
        || veh_stale
        || veh_fresh
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
            kv(
                "vehicle_mode",
                format!("{:?}", status.vehicle_mode(report_timeout)),
            ),
            kv(
                "subsystem_states",
                {
                    let s = status.subsystem_states(report_timeout);
                    let name = |x: Option<crate::dbc::SubsystemState>| {
                        x.map_or_else(|| "stale".to_string(), |v| format!("{v:?}"))
                    };
                    format!(
                        "mtr={} brk={} eps={} drv={}",
                        name(s[0]),
                        name(s[1]),
                        name(s[2]),
                        name(s[3])
                    )
                },
            ),
            kv("fault_latched", cmd.fault_latched.to_string()),
            kv("driver_estop", cmd.estop.driver().to_string()),
            kv("mrm_estop", cmd.estop.mrm().to_string()),
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
            kv(
                "rate_mtr_hz",
                status
                    .mtr_freq
                    .rate_hz()
                    .map(|r| format!("{r:.1}"))
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
            kv(
                "rate_eps_hz",
                status
                    .eps_freq
                    .rate_hz()
                    .map(|r| format!("{r:.1}"))
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
            kv(
                "rate_brk_hz",
                status
                    .brk_freq
                    .rate_hz()
                    .map(|r| format!("{r:.1}"))
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
            kv(
                "rate_veh_hz",
                status
                    .veh_freq
                    .rate_hz()
                    .map(|r| format!("{r:.1}"))
                    .unwrap_or_else(|| "n/a".to_string()),
            ),
            kv("bad_frames", status.bad_frames.to_string()),
            kv("tx_failed", cmd.tx_failed.to_string()),
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

fn clamp_f32(v: f32, min: f32, max: f32) -> f32 {
    if v.is_nan() {
        // NaN from upstream is treated as "no command" — pick the safe middle.
        // For symmetric ranges this is 0.0; for unsigned ranges this is the
        // lower bound (=0). Both prevent garbage propagating to the VCU.
        if min <= 0.0 && 0.0 <= max {
            0.0
        } else {
            min
        }
    } else {
        v.max(min).min(max)
    }
}

#[cfg(test)]
mod heading_rate_tests {
    use super::heading_rate;

    const WHEEL_BASE: f32 = 2.061;
    const LEFT: f32 = 0.2; // rad, Autoware frame: positive is left

    fn expected(v: f32, delta: f32) -> f32 {
        v * delta.tan() / WHEEL_BASE
    }

    #[test]
    fn straight_is_zero() {
        assert_eq!(heading_rate(3.0, Some(0.0), WHEEL_BASE), 0.0);
    }

    #[test]
    fn left_forward_is_positive() {
        let r = heading_rate(3.0, Some(LEFT), WHEEL_BASE);
        assert!(r > 0.0, "left turn forward must be a positive yaw rate, got {r}");
        assert!((r - expected(3.0, LEFT)).abs() < 1e-5);
    }

    #[test]
    fn right_forward_is_negative() {
        let r = heading_rate(3.0, Some(-LEFT), WHEEL_BASE);
        assert!(r < 0.0, "right turn forward must be a negative yaw rate, got {r}");
        assert!((r - expected(3.0, -LEFT)).abs() < 1e-5);
    }

    #[test]
    fn reverse_flips_the_sign() {
        // Backing up with the wheels turned left swings the nose right.
        let r = heading_rate(-1.5, Some(LEFT), WHEEL_BASE);
        assert!(r < 0.0, "left wheels in reverse must be a negative yaw rate, got {r}");
        assert!((r - expected(-1.5, LEFT)).abs() < 1e-5);
    }

    #[test]
    fn stale_steering_reports_zero() {
        assert_eq!(heading_rate(3.0, None, WHEEL_BASE), 0.0);
    }

    #[test]
    fn stopped_is_zero_whatever_the_wheels_do() {
        assert_eq!(heading_rate(0.0, Some(LEFT), WHEEL_BASE), 0.0);
    }
}

#[cfg(test)]
mod longitudinal_tests {
    //! A negative acceleration becomes `Target_Deceleration > 0`, and any
    //! deceleration makes the VCU (and `build_frames`) cut motor torque. The
    //! deadband keeps Autoware's PID jitter around zero at cruise from
    //! toggling the motor.

    use super::{split_longitudinal, Longitudinal};
    use crate::can_io::motor_frame_enabled;
    use crate::dbc::Gear;
    use crate::state::CommandState;
    use std::time::Instant;

    const MAX_ACCEL: f32 = 2.0;
    const MAX_DECEL: f32 = 4.0;
    const DEADBAND: f32 = 0.2;

    fn split(a: f32) -> Longitudinal {
        split_longitudinal(a, MAX_ACCEL, MAX_DECEL, DEADBAND)
    }

    /// Whether the motor frame goes out enabled for this split while driving.
    fn motor_on(l: Longitudinal) -> bool {
        let mut c = CommandState::default();
        c.last_control_at = Some(Instant::now());
        c.target_speed_mps = 2.0;
        c.target_acceleration_mps2 = l.accel;
        c.target_deceleration_mps2 = l.decel;
        motor_frame_enabled(&c, Gear::Drive)
    }

    #[test]
    fn small_decel_is_inside_the_deadband() {
        let l = split(-0.1);
        assert_eq!(l, Longitudinal { accel: 0.0, decel: 0.0 });
        assert!(motor_on(l), "jitter below the deadband must not cut the motor");
    }

    #[test]
    fn decel_beyond_the_deadband_passes_through() {
        let l = split(-0.5);
        assert_eq!(l.accel, 0.0);
        assert!((l.decel - 0.5).abs() < 1e-6, "not remapped, got {}", l.decel);
        assert!(!motor_on(l), "a real decel cuts the motor, as the VCU does");
    }

    #[test]
    fn positive_is_acceleration() {
        let l = split(0.3);
        assert!((l.accel - 0.3).abs() < 1e-6);
        assert_eq!(l.decel, 0.0);
        assert!(motor_on(l));
    }

    #[test]
    fn zero_deadband_restores_pass_through() {
        let l = split_longitudinal(-0.1, MAX_ACCEL, MAX_DECEL, 0.0);
        assert!((l.decel - 0.1).abs() < 1e-6);
    }

    #[test]
    fn decel_is_still_capped() {
        assert_eq!(split(-10.0).decel, MAX_DECEL);
    }
}
