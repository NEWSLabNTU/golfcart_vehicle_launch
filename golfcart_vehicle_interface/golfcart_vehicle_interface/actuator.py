#!/usr/bin/env python3
"""
Actuator stub for Golf Cart.

Placeholder node that accepts Autoware control commands but does not
actuate hardware. Will be replaced by the Turing Drive DBW interface.
"""
import sys

import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control


class GolfCartActuator(Node):
    """
    Stub actuator node that subscribes to Autoware control commands.

    Logs received commands periodically. No hardware actuation is performed.
    Replace this with the Turing Drive DBW interface when available.
    """

    def __init__(self):
        super().__init__("golfcart_actuator_node")

        self.declare_parameter("rate", 20.0)
        rate = self.get_parameter("rate").get_parameter_value().double_value

        self.create_subscription(Control, "~/input/control_cmd", self._on_control, 1)
        self.create_timer(1.0 / rate, self._tick)

        self._last_cmd = None
        self._cmd_count = 0

        self.get_logger().info("Actuator stub initialized (no hardware output)")

    def _on_control(self, msg):
        self._last_cmd = msg
        self._cmd_count += 1

    def _tick(self):
        if self._last_cmd is None:
            return
        # Log once per second (at 20 Hz, every 20 ticks)
        if self._cmd_count % 20 == 0:
            cmd = self._last_cmd
            self.get_logger().info(
                f"[stub] vel={cmd.longitudinal.velocity:.2f} m/s, "
                f"steer={cmd.lateral.steering_tire_angle:.3f} rad"
            )


def main():
    rclpy.init(args=sys.argv)
    node = GolfCartActuator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
