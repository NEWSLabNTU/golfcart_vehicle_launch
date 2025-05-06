import math
import sys

import Jetson.GPIO as GPIO

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport


class AutoSdvVelocityReportNode(Node):
    def __init__(self) -> None:
        super().__init__("autosdv_velocity_report_node")

        # Configure the node
        self.declare_parameter("pin", Parameter.Type.INTEGER)
        self.declare_parameter("rate", Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_diameter", Parameter.Type.DOUBLE)
        self.declare_parameter("markers_per_rotation", Parameter.Type.INTEGER)
        self.declare_parameter("frame_id", Parameter.Type.STRING)

        # Create the publisher
        publisher = self.create_publisher(VelocityReport, "~/input/velocity_status", 1)

        # Setup the pin
        pin = self.get_parameter("pin").get_parameter_value().integer_value
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=self.on_gpio_rising)

        # Start a periodic call
        rate = self.get_parameter("rate").get_parameter_value().double_value
        timer = self.create_timer(1.0 / rate, self.publish_callback)

        # Compute wheel parameters
        wheel_diameter_cm = (
            self.get_parameter("wheel_diameter").get_parameter_value().double_value
        )
        wheel_circumference_meters = wheel_diameter_cm / 100.0 * math.pi

        clock = self.get_clock()

        # Save variables
        self.wheel_circumference_meters = wheel_circumference_meters
        self.markers_per_rotation = (
            self.get_parameter("markers_per_rotation")
            .get_parameter_value()
            .integer_value
        )
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
        self.pin = pin
        self.publisher = publisher
        self.timer = timer
        self.count = 0
        self.prev_time = clock.now()
        self.clock = clock

    def publish_callback(self) -> None:
        # Compute the elapsed time since the last measurement
        curr_time = self.clock.now()
        elapsed_secs = (curr_time - self.prev_time).nanoseconds / (10**9)

        # Compute the speed
        markers_pers_sec = self.count / elapsed_secs
        rotations_pers_sec = markers_pers_sec / self.markers_per_rotation
        speed = rotations_pers_sec * self.wheel_circumference_meters

        # Reset state variables
        self.count = 0
        self.prev_time = curr_time

        # Publish the speed
        msg = VelocityReport()
        msg.header.stamp = curr_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.longitudinal_velocity = speed
        msg.lateral_velocity = 0.0
        msg.heading_rate = 0.0
        self.publisher.publish(msg)

    def on_gpio_rising(self, channel) -> None:
        # Increase the magnet marker count
        self.count += 1


def main():
    rclpy.init(args=sys.argv)
    node = AutoSdvVelocityReportNode()
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
