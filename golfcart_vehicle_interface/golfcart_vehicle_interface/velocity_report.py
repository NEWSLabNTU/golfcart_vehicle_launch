#!/usr/bin/env python3
"""
Velocity report stub for Golf Cart.

Placeholder node that publishes zero-velocity reports to satisfy
Autoware's vehicle status requirements. Will be replaced by the
Turing Drive DBW interface which provides real velocity feedback.
"""
import sys

import rclpy
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport


class GolfCartVelocityReportNode(Node):
    """
    Stub velocity report node that publishes zero velocity.

    Autoware requires /vehicle/status/velocity_status to be published
    for the control stack to function. This stub satisfies that requirement.
    Replace with Turing Drive DBW velocity feedback when available.
    """

    def __init__(self):
        super().__init__("golfcart_velocity_report_node")

        self.declare_parameter("rate", 20.0)
        self.declare_parameter("frame_id", "base_link")

        rate = self.get_parameter("rate").get_parameter_value().double_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value

        self._pub = self.create_publisher(VelocityReport, "~/output/velocity_status", 1)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info("Velocity report stub initialized (publishing zero velocity)")

    def _publish(self):
        msg = VelocityReport()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.longitudinal_velocity = 0.0
        msg.lateral_velocity = 0.0
        msg.heading_rate = 0.0
        self._pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = GolfCartVelocityReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
