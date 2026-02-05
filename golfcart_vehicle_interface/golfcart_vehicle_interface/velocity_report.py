#!/usr/bin/env python3
"""
Velocity report node for AutoSDV.
This module measures and reports the vehicle's velocity using wheel rotation sensors.
"""
import math
import sys

import Jetson.GPIO as GPIO

import rclpy
from rclpy import Parameter
from rclpy.node import Node
from autoware_vehicle_msgs.msg import VelocityReport


class AutoSdvVelocityReportNode(Node):
    """
    Node that measures and reports vehicle velocity.

    Uses GPIO input from wheel rotation sensors to calculate speed.
    Publishes this information as VelocityReport messages which are used
    by the control system for feedback.
    """
    def __init__(self) -> None:
        super().__init__("autosdv_velocity_report_node")

        # Configure the node parameters
        self.declare_parameter("pin", Parameter.Type.INTEGER)
        self.declare_parameter("rate", Parameter.Type.DOUBLE)
        self.declare_parameter("wheel_diameter", Parameter.Type.DOUBLE)
        self.declare_parameter("markers_per_rotation", Parameter.Type.INTEGER)
        self.declare_parameter("frame_id", Parameter.Type.STRING)

        # Create the publisher for velocity reports
        publisher = self.create_publisher(VelocityReport, "~/output/velocity_status", 1)

        # Setup the GPIO pin for wheel rotation detection
        pin = self.get_parameter("pin").get_parameter_value().integer_value
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.IN)
        GPIO.add_event_detect(pin, GPIO.RISING, callback=self.on_gpio_rising)

        # Start a periodic call for publishing velocity
        rate = self.get_parameter("rate").get_parameter_value().double_value
        timer = self.create_timer(1.0 / rate, self.publish_callback)

        # Compute wheel parameters for speed calculation
        wheel_diameter_cm = (
            self.get_parameter("wheel_diameter").get_parameter_value().double_value
        )
        # Convert wheel diameter from cm to meters and calculate circumference
        wheel_circumference_meters = wheel_diameter_cm / 100.0 * math.pi

        # Get reference to ROS clock
        clock = self.get_clock()

        # Store all required variables as instance attributes
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
        self.count = 0  # Counter for wheel markers detected
        self.prev_time = clock.now()  # Time of last measurement
        self.clock = clock

    def publish_callback(self) -> None:
        """
        Timer callback that calculates and publishes the vehicle velocity.

        Computes speed based on wheel rotation markers detected since last call.
        """
        # Compute the elapsed time since the last measurement
        curr_time = self.clock.now()
        elapsed_secs = (curr_time - self.prev_time).nanoseconds / (10**9)

        # Compute the speed based on marker count and elapsed time
        markers_per_sec = self.count / elapsed_secs
        rotations_per_sec = markers_per_sec / self.markers_per_rotation
        speed = rotations_per_sec * self.wheel_circumference_meters

        # Reset state variables for next measurement
        self.count = 0
        self.prev_time = curr_time

        # Create and publish the velocity report message
        msg = VelocityReport()
        msg.header.stamp = curr_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.longitudinal_velocity = speed  # Forward/backward velocity
        msg.lateral_velocity = 0.0  # Side-to-side velocity (always 0 for this vehicle)
        msg.heading_rate = 0.0  # Rate of heading change (not measured here)
        self.publisher.publish(msg)

    def on_gpio_rising(self, channel) -> None:
        """
        GPIO event callback that is triggered when a wheel marker is detected.

        Increments the marker count for speed calculation.

        Args:
            channel: GPIO channel that triggered the event
        """
        # Increment the marker count for each detected rising edge
        self.count += 1


def main():
    """
    Main function to initialize and run the node.
    """
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
