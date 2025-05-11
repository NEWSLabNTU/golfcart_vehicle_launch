#!/usr/bin/env python3
"""
Actuator control node for AutoSDV.
This module controls the vehicle's motor and steering using PID controllers.
"""
import sys
import math
from typing import Optional
from dataclasses import dataclass

from Adafruit_PCA9685 import PCA9685
from simple_pid import PID

import rclpy
from rclpy.node import Node
from rclpy import Parameter
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport


class AutoSdvActuator(Node):
    """
    Node that controls the vehicle's motor and steering servo.

    Subscribes to:
    - Control commands from Autoware
    - IMU data for angular velocity
    - Velocity reports for current speed

    Uses PID controllers to compute PWM values for motor and steering servo.
    """
    def __init__(self):
        super().__init__("autosdv_actuator_node")


        # Register motor controller parameters
        self.declare_parameter("i2c_address", Parameter.Type.INTEGER)
        self.declare_parameter("i2c_busnum", Parameter.Type.INTEGER)
        self.declare_parameter("pwm_freq", Parameter.Type.INTEGER)

        # Publication rate
        self.declare_parameter("rate", Parameter.Type.DOUBLE)

        # Register longitudinal control parameters
        self.declare_parameter("kp_speed", Parameter.Type.DOUBLE)
        self.declare_parameter("ki_speed", Parameter.Type.DOUBLE)
        self.declare_parameter("kd_speed", Parameter.Type.DOUBLE)

        self.declare_parameter("min_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("init_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("max_pwm", Parameter.Type.INTEGER)

        # Register steering control parameters
        self.declare_parameter("min_steer", Parameter.Type.INTEGER)
        self.declare_parameter("init_steer", Parameter.Type.INTEGER)
        self.declare_parameter("max_steer", Parameter.Type.INTEGER)

        self.declare_parameter("tire_angle_to_steer_ratio", Parameter.Type.DOUBLE)



        # Build configuration from parameters
        config = Config(
            init_pwm=self.get_parameter("init_pwm").get_parameter_value().integer_value,
            min_pwm=self.get_parameter("min_pwm").get_parameter_value().integer_value,
            max_pwm=self.get_parameter("max_pwm").get_parameter_value().integer_value,
            init_steer=(
                self.get_parameter("init_steer").get_parameter_value().integer_value
            ),
            min_steer=self.get_parameter("min_steer")
            .get_parameter_value()
            .integer_value,
            max_steer=self.get_parameter("max_steer")
            .get_parameter_value()
            .integer_value,
            tire_angle_to_steer_ratio=self.get_parameter("tire_angle_to_steer_ratio")
            .get_parameter_value()
            .double_value,
        )

        # Get PID controller parameters
        kp_speed = self.get_parameter("kp_speed").get_parameter_value().double_value
        ki_speed = self.get_parameter("ki_speed").get_parameter_value().double_value
        kd_speed = self.get_parameter("kd_speed").get_parameter_value().double_value

        kp_steer = self.get_parameter("kp_steer").get_parameter_value().double_value
        ki_steer = self.get_parameter("ki_steer").get_parameter_value().double_value
        kd_steer = self.get_parameter("kd_steer").get_parameter_value().double_value

        # Initialize the speed PID controller
        min_pid_output = config.min_pwm - config.init_pwm
        max_pid_output = config.max_pwm - config.init_pwm
        self.speed_pid = PID(
            Kp=kp_speed,
            Ki=ki_speed,
            Kd=kd_speed,
            output_limits=(min_pid_output, max_pid_output),
        )

        # Initialize the steering PID controller
        min_pid_output = config.min_steer - config.init_steer
        max_pid_output = config.max_steer - config.init_steer
        self.steer_pid = PID(
            Kp=kp_steer,
            Ki=ki_steer,
            Kd=kd_steer,
            output_limits=(min_pid_output, max_pid_output),
        )

        # Initialize the controller state
        state = State(
            target_speed=None,
            current_speed=None,
            target_tire_angle=None,
            current_tire_angle=None,
            angular_speed=None,
        )

        # Initialize the PCA9685 PWM driver
        pwm_freq = self.get_parameter("pwm_freq").get_parameter_value().integer_value
        i2c_address = (
            self.get_parameter("i2c_address").get_parameter_value().integer_value
        )
        i2c_busnum = (
            self.get_parameter("i2c_busnum").get_parameter_value().integer_value
        )
        driver = PCA9685(address=i2c_address, busnum=i2c_busnum)
        driver.set_pwm_freq(pwm_freq)

        # Subscribe to control commands
        control_cmd_subscription = self.create_subscription(
            Control,
            "~/input/control_cmd",
            self.control_callback,
            1,
        )

        # Subscribe to IMU data
        imu_subscription = self.create_subscription(
            Imu,
            "/sensing/imu/imu_data",
            self.imu_callback,
            1,
        )

        # Subscribe to speed data
        speed_subscription = self.create_subscription(
            VelocityReport,
            "/vehicle/status/velocity_status",
            self.velocity_callback,
            1,
        )

        # Start periodic calls
        publication_period = (
            1.0 / self.get_parameter("rate").get_parameter_value().double_value
        )
        timer = self.create_timer(publication_period, self.timer_callback)

        # Save variables
        self.config = config
        self.state = state
        self.control_cmd_subscription = control_cmd_subscription
        self.imu_subscription = imu_subscription
        self.driver = driver
        self.timer = timer

    def imu_callback(self, msg):
        """
        Callback for IMU messages.
        Updates the angular velocity state.

        Args:
            msg: Imu message containing angular velocity data
        """
        angular_speed = msg.angular_velocity.z
        self.state.angular_speed = angular_speed

    def velocity_callback(self, msg):
        """
        Callback for velocity report messages.
        Updates the current speed state.

        Args:
            msg: VelocityReport message containing current vehicle speed
        """
        speed = msg.longitudinal_velocity
        self.state.current_speed = speed

    def control_callback(self, msg):
        """
        Callback for control command messages.
        Updates the target speed and steering angle state.

        Args:
            msg: Control message containing target velocity and steering angle
        """
        self.state.target_speed = msg.longitudinal.velocity
        self.state.target_tire_angle = msg.lateral.steering_tire_angle

    def timer_callback(self):
        """
        Timer callback that runs at the configured rate.
        Computes and sets PWM values for motor and steering servo.
        """
        # Set the power of the DC motor
        pwm_value = self.compute_pwm_value()
        self.driver.set_pwm(0, 0, pwm_value)

        # Set angle of the steering servo
        steer_value = self.compute_steer_value()
        self.driver.set_pwm(1, 0, steer_value)

    def compute_pwm_value(self) -> int:
        """
        Computes PWM value for the motor based on speed error and PID control.

        Returns:
            int: PWM value for the motor
        """
        # If no target or current speed available, return initial PWM (stopped)
        if self.state.target_speed is None or self.state.target_speed == 0 or self.state.current_speed is None:
            return self.config.init_pwm

        # Calculate speed error (current - target)
        error = self.state.current_speed - self.state.target_speed

        # Calculate PID output
        pid_value = self.speed_pid(error)

        # Calculate final PWM value
        pwm_value = self.config.init_pwm + int(pid_value)

        # Limit PWM value to min/max range
        pwm_value = max(min(pwm_value, self.config.max_pwm), self.config.min_pwm)

        return pwm_value

    def compute_steer_value(self) -> int:
        """
        Computes PWM value for the steering servo based on target tire angle.

        Returns:
            int: PWM value for the steering servo
        """
        # If no target tire angle available, return initial steering value (straight)
        if self.state.target_tire_angle is None:
            return self.config.init_steer

        # Convert tire angle to steering PWM offset using the configured ratio
        steer = self.state.target_tire_angle * self.config.tire_angle_to_steer_ratio

        # Calculate final steering PWM value
        steer_pwm = self.config.init_steer + int(steer)

        # Limit steering PWM value to min/max range
        steer_pwm = max(self.config.min_steer, min(self.config.max_steer, steer_pwm))

        return steer_pwm


@dataclass
class State:
    """
    Dataclass to hold the current state of the controller.

    Attributes:
        target_speed: Target longitudinal velocity from control commands
        current_speed: Current measured vehicle speed
        target_tire_angle: Target steering tire angle from control commands
        current_tire_angle: Current measured tire angle (if available)
        angular_speed: Current angular velocity from IMU
    """
    target_speed: Optional[float]
    current_speed: Optional[float]

    target_tire_angle: Optional[float]
    current_tire_angle: Optional[float]
    angular_speed: Optional[float]

    # Additional state variables could be added here for enhanced control algorithms


@dataclass
class Config:
    """
    Dataclass to hold the configuration parameters.

    Attributes:
        min_pwm: Minimum PWM value for backward movement
        init_pwm: Initial PWM value (stopped)
        max_pwm: Maximum PWM value for forward movement
        min_steer: Minimum steering PWM value (left turn)
        init_steer: Initial steering PWM value (straight)
        max_steer: Maximum steering PWM value (right turn)
        tire_angle_to_steer_ratio: Conversion ratio from tire angle to steering PWM
    """
    min_pwm: int
    init_pwm: int
    max_pwm: int

    min_steer: int
    init_steer: int
    max_steer: int

    tire_angle_to_steer_ratio: float


def main():
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=sys.argv)
    node = AutoSdvActuator()
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
