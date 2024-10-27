import math
import sys

import Jetson.GPIO as GPIO

import rclpy
from rclyp.node import Node
from autoware_auto_vehicle_msgs.msg import VelocityReport


class F1eighthVelocityReportNode(Node):
    def __init__(self) -> None:
        super().__init__("f1eighth_velocity_report_node")

        # Configure the node
        self.declare_parameter("pin", 13, description="The input GPIO pin")
        self.declare_parameter(
            "rate",
            20,
            description="The frequency of velocity publication",
        )
        self.declare_parameter(
            "wheel_diameter", 5, description="The wheel diameter in centermeters"
        )
        self.declare_parameter(
            "markers_per_rotation", 12, description="The number of markers per rotation"
        )
        publisher = self.create_publisher(
            VelocityReport, "/vehicle/status/velocity_status", 1
        )

        # Setup the pin
        pin = self.get_parameter("pin").get_parameter_value().integer_value
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=self.on_gpio_rising)

        # Start a periodic call
        rate = self.get_parameter("rate").get_parameter_value().double_value
        timer = self.create_timer(1.0 / rate, self.publish_callback)

        # Save variables
        self.wheel_circumference_meters = (
            self.get_parameter("wheel_diameter").value / 100.0 * math.pi
        )
        self.markers_per_rotation = self.get_parameter("markers_per_rotation").value
        self.pin = pin
        self.publisher = publisher
        self.timer = timer
        self.count = 0
        self.prev_time = self.now()

    def publish_callback(self) -> None:
        # Compute the elapsed time since the last measurement
        curr_time = self.now()
        elapsed_secs = (curr_time - self.prev_time).nanoseconds / (10 ** 9)

        # Compute the speed
        markers_pers_sec = self.count / elapsed_secs
        rotations_pers_sec = markers_pers_sec / self.markers_per_rotation
        speed = rotations_pers_sec * self.wheel_circumference_meters

        # Reset state variables
        self.count = 0
        self.prev_time = curr_time

        # Publish the speed
        msg = VelocityReport()
        msg.header.stamp = curr_time
        msg.longitudinal_velocity = speed
        msg.lateral_velocity = 0.0
        msg.heading_rate = 0.0
        self.publisher.publish(msg)

    def on_gpio_rising(self) -> None:
        # Increase the magnet marker count
        self.count += 1


def main():
    rclpy.init(sys.argv)
    node = F1eighthVelocityReportNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Gracefully shutdown on keyboard interrupt
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
