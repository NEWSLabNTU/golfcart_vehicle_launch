#!/usr/bin/env python3
"""Tkinter GUI teleop companion for `golfcart_vehicle_interface`.

Streams `autoware_control_msgs/msg/Control` at a configurable rate so the
vehicle interface stays out of its `control_timeout_ms` window. Buttons drive
gear, blinker, hazard, ControlMode service, and the operator e-stop topic.

Designed for bench/garage runs alongside the vehicle interface — set
`tx_enabled:=false` on the interface for a dry run, `tx_enabled:=true` once
the rig is wired up.
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    GearCommand,
    HazardLightsCommand,
    TurnIndicatorsCommand,
)
from autoware_vehicle_msgs.srv import ControlModeCommand
from std_msgs.msg import Bool


CMD_RATE_HZ = 50.0
SPEED_RANGE_MPS = (-3.0, 3.0)
ACCEL_RANGE_MPS2 = (-4.0, 2.0)
STEER_RANGE_RAD = (-0.349, 0.349)


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("golfcart_teleop_gui")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_ctrl = self.create_publisher(
            Control, "/control/command/control_cmd", qos
        )
        self.pub_gear = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", qos
        )
        self.pub_turn = self.create_publisher(
            TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", qos
        )
        self.pub_hazard = self.create_publisher(
            HazardLightsCommand, "/control/command/hazard_lights_cmd", qos
        )
        self.pub_estop = self.create_publisher(Bool, "/vehicle/emergency_stop", qos)
        self.cli_mode = self.create_client(
            ControlModeCommand, "/control/control_mode_request"
        )

        self.target_speed = 0.0
        self.target_accel = 0.5
        self.target_steer = 0.0

        self.create_timer(1.0 / CMD_RATE_HZ, self._tick)

    def _tick(self) -> None:
        msg = Control()
        msg.lateral.steering_tire_angle = float(self.target_steer)
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.longitudinal.velocity = float(self.target_speed)
        msg.longitudinal.acceleration = float(self.target_accel)
        msg.longitudinal.jerk = 0.0
        self.pub_ctrl.publish(msg)

    def send_gear(self, gear: int) -> None:
        m = GearCommand()
        m.command = gear
        self.pub_gear.publish(m)

    def send_turn(self, val: int) -> None:
        m = TurnIndicatorsCommand()
        m.command = val
        self.pub_turn.publish(m)

    def send_hazard(self, on: bool) -> None:
        m = HazardLightsCommand()
        m.command = (
            HazardLightsCommand.ENABLE if on else HazardLightsCommand.DISABLE
        )
        self.pub_hazard.publish(m)

    def send_estop(self, on: bool) -> None:
        self.pub_estop.publish(Bool(data=on))

    def request_mode(self, mode: int) -> bool:
        if not self.cli_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("control_mode_request service not available")
            return False
        req = ControlModeCommand.Request()
        req.mode = mode
        future = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None:
            self.get_logger().warn("control_mode_request timed out")
            return False
        return bool(future.result().success)


class TeleopGUI:
    def __init__(self, node: TeleopNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self.root.title("Golf Cart Teleop")
        self.root.geometry("520x520")
        self._build()

    def _build(self) -> None:
        pad = {"padx": 8, "pady": 4}

        speed_frame = ttk.LabelFrame(self.root, text="Speed (m/s)")
        speed_frame.pack(fill="x", **pad)
        self.speed_var = tk.DoubleVar(value=0.0)
        self.speed_lbl = ttk.Label(speed_frame, text="0.00")
        self.speed_lbl.pack(side="right", padx=8)
        ttk.Scale(
            speed_frame,
            from_=SPEED_RANGE_MPS[0],
            to=SPEED_RANGE_MPS[1],
            orient="horizontal",
            variable=self.speed_var,
            command=self._on_speed,
        ).pack(fill="x", padx=8, pady=4)

        accel_frame = ttk.LabelFrame(self.root, text="Accel target (m/s²)")
        accel_frame.pack(fill="x", **pad)
        self.accel_var = tk.DoubleVar(value=0.5)
        self.accel_lbl = ttk.Label(accel_frame, text="0.50")
        self.accel_lbl.pack(side="right", padx=8)
        ttk.Scale(
            accel_frame,
            from_=ACCEL_RANGE_MPS2[0],
            to=ACCEL_RANGE_MPS2[1],
            orient="horizontal",
            variable=self.accel_var,
            command=self._on_accel,
        ).pack(fill="x", padx=8, pady=4)

        steer_frame = ttk.LabelFrame(self.root, text="Steering tire angle (rad)")
        steer_frame.pack(fill="x", **pad)
        self.steer_var = tk.DoubleVar(value=0.0)
        self.steer_lbl = ttk.Label(steer_frame, text="0.000")
        self.steer_lbl.pack(side="right", padx=8)
        ttk.Scale(
            steer_frame,
            from_=STEER_RANGE_RAD[0],
            to=STEER_RANGE_RAD[1],
            orient="horizontal",
            variable=self.steer_var,
            command=self._on_steer,
        ).pack(fill="x", padx=8, pady=4)

        gear_frame = ttk.LabelFrame(self.root, text="Gear")
        gear_frame.pack(fill="x", **pad)
        for label, val in [
            ("PARK", GearCommand.PARK),
            ("REVERSE", GearCommand.REVERSE),
            ("NEUTRAL", GearCommand.NEUTRAL),
            ("DRIVE", GearCommand.DRIVE),
        ]:
            ttk.Button(
                gear_frame, text=label, command=lambda v=val: self.node.send_gear(v)
            ).pack(side="left", expand=True, fill="x", padx=4, pady=4)

        mode_frame = ttk.LabelFrame(self.root, text="Control mode")
        mode_frame.pack(fill="x", **pad)
        self.mode_status = ttk.Label(mode_frame, text="—")
        self.mode_status.pack(side="right", padx=8)
        # The driver engages autonomous on the vehicle itself; this button only
        # asks the interface whether the vehicle is already there.
        ttk.Button(
            mode_frame,
            text="Check AUTONOMOUS",
            command=lambda: self._on_mode(ControlModeCommand.Request.AUTONOMOUS),
        ).pack(side="left", expand=True, fill="x", padx=4, pady=4)
        ttk.Button(
            mode_frame,
            text="Clear fault (MANUAL)",
            command=lambda: self._on_mode(ControlModeCommand.Request.MANUAL),
        ).pack(side="left", expand=True, fill="x", padx=4, pady=4)

        sig_frame = ttk.LabelFrame(self.root, text="Signals")
        sig_frame.pack(fill="x", **pad)
        ttk.Button(
            sig_frame,
            text="Left",
            command=lambda: self.node.send_turn(TurnIndicatorsCommand.ENABLE_LEFT),
        ).pack(side="left", expand=True, fill="x", padx=4, pady=4)
        ttk.Button(
            sig_frame,
            text="Right",
            command=lambda: self.node.send_turn(TurnIndicatorsCommand.ENABLE_RIGHT),
        ).pack(side="left", expand=True, fill="x", padx=4, pady=4)
        ttk.Button(
            sig_frame,
            text="Off",
            command=lambda: self.node.send_turn(TurnIndicatorsCommand.DISABLE),
        ).pack(side="left", expand=True, fill="x", padx=4, pady=4)
        self.hazard_on = False
        self.hazard_btn = ttk.Button(
            sig_frame, text="Hazard: OFF", command=self._toggle_hazard
        )
        self.hazard_btn.pack(side="left", expand=True, fill="x", padx=4, pady=4)

        estop_frame = ttk.LabelFrame(self.root, text="Operator E-Stop")
        estop_frame.pack(fill="x", **pad)
        self.estop_on = False
        self.estop_btn = tk.Button(
            estop_frame,
            text="E-STOP (release)",
            bg="#cc3333",
            fg="white",
            font=("TkDefaultFont", 12, "bold"),
            command=self._toggle_estop,
        )
        self.estop_btn.pack(fill="x", padx=8, pady=8)

        zero_btn = ttk.Button(self.root, text="Zero speed + steer", command=self._zero)
        zero_btn.pack(fill="x", **pad)

    def _on_speed(self, _v: str) -> None:
        v = self.speed_var.get()
        self.node.target_speed = v
        self.speed_lbl.config(text=f"{v:+.2f}")

    def _on_accel(self, _v: str) -> None:
        a = self.accel_var.get()
        self.node.target_accel = a
        self.accel_lbl.config(text=f"{a:+.2f}")

    def _on_steer(self, _v: str) -> None:
        s = self.steer_var.get()
        self.node.target_steer = s
        self.steer_lbl.config(text=f"{s:+.3f}")

    def _on_mode(self, mode: int) -> None:
        ok = self.node.request_mode(mode)
        if mode == ControlModeCommand.Request.AUTONOMOUS:
            # success == "the VCU reports all four subsystems autonomous and no
            # fault is latched", not "engaged just now".
            text = "AUTONOMOUS" if ok else "not in AUTONOMOUS"
        else:
            text = "fault cleared" if ok else "clear FAILED"
        self.mode_status.config(text=text)

    def _toggle_hazard(self) -> None:
        self.hazard_on = not self.hazard_on
        self.node.send_hazard(self.hazard_on)
        self.hazard_btn.config(text=f"Hazard: {'ON' if self.hazard_on else 'OFF'}")

    def _toggle_estop(self) -> None:
        self.estop_on = not self.estop_on
        self.node.send_estop(self.estop_on)
        self.estop_btn.config(
            text="E-STOP ACTIVE — click to release"
            if self.estop_on
            else "E-STOP (release)",
            bg="#aa0000" if self.estop_on else "#cc3333",
        )

    def _zero(self) -> None:
        self.speed_var.set(0.0)
        self.steer_var.set(0.0)
        self._on_speed("")
        self._on_steer("")

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    rclpy.init()
    node = TeleopNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        TeleopGUI(node).run()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
