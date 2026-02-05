#!/usr/bin/env python3
"""
Actuator control node for AutoSDV using Ackermann control.
This module implements a cascaded PID controller for vehicle's motor and steering using Ackermann steering model.
"""
import sys
import math
import time
from typing import Optional, Tuple
from dataclasses import dataclass

from Adafruit_PCA9685 import PCA9685
from simple_pid import PID

import rclpy
from rclpy.node import Node
from rclpy import Parameter
from geometry_msgs.msg import TwistWithCovarianceStamped
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import VelocityReport
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from autoware_internal_debug_msgs.msg import Float32MultiArrayStamped


class AckermannPID:
    """
    Custom PID controller for Ackermann control with anti-windup protection.
    Similar to CARLA's PID implementation but adapted for Python and ROS 2.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        min_output: float = -1.0,
        max_output: float = 1.0,
    ):
        """
        Initialize a new PID controller.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            min_output: Minimum output value
            max_output: Maximum output value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_output = min_output
        self.max_output = max_output

        self.set_point = 0.0

        # Internal state
        self.proportional = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        self.last_error = 0.0
        self.last_input = 0.0

    def set_target_point(self, point: float) -> None:
        """
        Set the target point for the controller.

        Args:
            point: Target value
        """
        self.set_point = point

    def run(self, input_value: float, delta_time: float) -> float:
        """
        Run the PID controller for one step.

        Args:
            input_value: Current value
            delta_time: Time since last update in seconds

        Returns:
            float: Controller output
        """
        error = self.set_point - input_value

        self.proportional = self.kp * error
        self.integral += self.ki * error * delta_time
        # Avoid integral windup
        self.integral = max(self.min_output, min(self.integral, self.max_output))

        if delta_time > 0:
            self.derivative = (-self.kd * (input_value - self.last_input)) / delta_time
        else:
            self.derivative = 0.0

        output = self.proportional + self.integral + self.derivative
        output = max(self.min_output, min(output, self.max_output))

        # Keep track of state
        self.last_error = error
        self.last_input = input_value

        return output

    def reset(self) -> None:
        """Reset the controller state."""
        self.proportional = 0.0
        self.integral = 0.0
        self.integral = max(self.min_output, min(self.integral, self.max_output))
        self.derivative = 0.0

        self.last_error = 0.0
        self.last_input = 0.0


class AutoSdvActuator(Node):
    """
    Node that controls the vehicle's motor and steering servo using an Ackermann control model.

    Subscribes to:
    - Control commands from Autoware
    - Velocity reports for current speed

    Uses a cascaded PID controller similar to CARLA's Ackermann implementation:
    - Outer loop: Speed controller that produces target acceleration
    - Inner loop: Acceleration controller that produces throttle/brake commands
    """

    def __init__(self):
        super().__init__("autosdv_actuator_node")

        # Declare all parameters
        self.declare_all_parameters()

        # Get parameter values
        parameter_values = self.get_all_parameter_values()

        # Create configuration
        self.config = self.create_config(parameter_values)

        # Initialize PID controllers
        self.initialize_pid_controllers(parameter_values)

        # Initialize controller state
        self.state = self.initialize_state()

        # Initialize hardware
        self.driver = self.initialize_pwm_driver(parameter_values)

        # Set up subscriptions
        self.setup_subscriptions()

        # Create timer
        publication_period = 1.0 / parameter_values["rate"]
        self.timer = self.create_timer(publication_period, self.timer_callback)

        # Debug log
        self.get_logger().info("Ackermann controller initialized")

    def declare_all_parameters(self):
        """Declare all parameters used by the node."""
        # Hardware parameters
        self.declare_parameter("i2c_address", Parameter.Type.INTEGER)
        self.declare_parameter("i2c_busnum", Parameter.Type.INTEGER)
        self.declare_parameter("pwm_freq", Parameter.Type.INTEGER)

        # Publication rate
        self.declare_parameter("rate", Parameter.Type.DOUBLE)

        # Debugging options
        self.declare_parameter("enable_debug_publishing", Parameter.Type.BOOL)

        # Longitudinal control parameters
        self.declare_parameter("kp_speed", Parameter.Type.DOUBLE)
        self.declare_parameter("ki_speed", Parameter.Type.DOUBLE)
        self.declare_parameter("kd_speed", Parameter.Type.DOUBLE)

        self.declare_parameter("kp_accel", Parameter.Type.DOUBLE)
        self.declare_parameter("ki_accel", Parameter.Type.DOUBLE)
        self.declare_parameter("kd_accel", Parameter.Type.DOUBLE)

        self.declare_parameter("max_accel", Parameter.Type.DOUBLE)
        self.declare_parameter("max_decel", Parameter.Type.DOUBLE)

        self.declare_parameter("min_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("init_pwm", Parameter.Type.INTEGER)
        self.declare_parameter("max_pwm", Parameter.Type.INTEGER)

        # Steering control parameters
        self.declare_parameter("steering_speed", Parameter.Type.DOUBLE)
        self.declare_parameter("min_steer", Parameter.Type.INTEGER)
        self.declare_parameter("init_steer", Parameter.Type.INTEGER)
        self.declare_parameter("max_steer", Parameter.Type.INTEGER)
        self.declare_parameter("max_steering_angle", Parameter.Type.DOUBLE)

        self.declare_parameter("tire_angle_to_steer_ratio", Parameter.Type.DOUBLE)

    def get_all_parameter_values(self):
        """
        Get all parameter values.

        Returns:
            dict: Dictionary with parameter names as keys and their values
        """
        params = {}

        # Hardware parameters
        params["i2c_address"] = (
            self.get_parameter("i2c_address").get_parameter_value().integer_value
        )
        params["i2c_busnum"] = (
            self.get_parameter("i2c_busnum").get_parameter_value().integer_value
        )
        params["pwm_freq"] = (
            self.get_parameter("pwm_freq").get_parameter_value().integer_value
        )

        # Publication rate
        params["rate"] = self.get_parameter("rate").get_parameter_value().double_value

        # Debug settings
        params["enable_debug_publishing"] = (
            self.get_parameter("enable_debug_publishing")
            .get_parameter_value()
            .bool_value
        )

        # PWM parameters
        params["min_pwm"] = (
            self.get_parameter("min_pwm").get_parameter_value().integer_value
        )
        params["init_pwm"] = (
            self.get_parameter("init_pwm").get_parameter_value().integer_value
        )
        params["max_pwm"] = (
            self.get_parameter("max_pwm").get_parameter_value().integer_value
        )

        # Steering parameters
        params["min_steer"] = (
            self.get_parameter("min_steer").get_parameter_value().integer_value
        )
        params["init_steer"] = (
            self.get_parameter("init_steer").get_parameter_value().integer_value
        )
        params["max_steer"] = (
            self.get_parameter("max_steer").get_parameter_value().integer_value
        )
        params["tire_angle_to_steer_ratio"] = (
            self.get_parameter("tire_angle_to_steer_ratio")
            .get_parameter_value()
            .double_value
        )

        # Ackermann parameters
        params["max_accel"] = (
            self.get_parameter("max_accel").get_parameter_value().double_value
        )
        params["max_decel"] = (
            self.get_parameter("max_decel").get_parameter_value().double_value
        )
        params["steering_speed"] = (
            self.get_parameter("steering_speed").get_parameter_value().double_value
        )
        params["max_steering_angle"] = (
            self.get_parameter("max_steering_angle").get_parameter_value().double_value
        )

        # PID controller parameters - speed
        params["kp_speed"] = (
            self.get_parameter("kp_speed").get_parameter_value().double_value
        )
        params["ki_speed"] = (
            self.get_parameter("ki_speed").get_parameter_value().double_value
        )
        params["kd_speed"] = (
            self.get_parameter("kd_speed").get_parameter_value().double_value
        )

        # PID controller parameters - acceleration
        params["kp_accel"] = (
            self.get_parameter("kp_accel").get_parameter_value().double_value
        )
        params["ki_accel"] = (
            self.get_parameter("ki_accel").get_parameter_value().double_value
        )
        params["kd_accel"] = (
            self.get_parameter("kd_accel").get_parameter_value().double_value
        )

        return params

    def create_config(self, params):
        """
        Create the configuration object.

        Args:
            params: Dictionary with parameter values

        Returns:
            Config: Configuration object
        """
        return Config(
            min_pwm=params["min_pwm"],
            init_pwm=params["init_pwm"],
            max_pwm=params["max_pwm"],
            min_steer=params["min_steer"],
            init_steer=params["init_steer"],
            max_steer=params["max_steer"],
            tire_angle_to_steer_ratio=params["tire_angle_to_steer_ratio"],
            max_accel=params["max_accel"],
            max_decel=params["max_decel"],
            steering_speed=params["steering_speed"],
            max_steering_angle=params["max_steering_angle"],
        )

    def initialize_pid_controllers(self, params):
        """
        Initialize the PID controllers.

        Args:
            params: Dictionary with parameter values
        """
        # 1. Speed controller (outer loop) - determines target acceleration
        self.speed_controller = AckermannPID(
            kp=params["kp_speed"],
            ki=params["ki_speed"],
            kd=params["kd_speed"],
            min_output=-params["max_decel"],
            max_output=params["max_accel"],
        )

        # 2. Acceleration controller (inner loop) - determines throttle/brake
        self.accel_controller = AckermannPID(
            kp=params["kp_accel"],
            ki=params["ki_accel"],
            kd=params["kd_accel"],
            min_output=-1.0,  # Full brake
            max_output=1.0,  # Full throttle
        )

    def initialize_state(self):
        """
        Initialize the controller state.

        Returns:
            State: State object
        """
        return State(
            target_speed=None,
            current_speed=None,
            target_tire_angle=None,
            current_tire_angle=None,
            # Ackermann specific state
            speed_control_accel_target=0.0,
            accel_control_pedal_target=0.0,
            current_acceleration=0.0,
            last_speed=0.0,
            last_acceleration=0.0,
            in_reverse=False,
            last_update_time=time.time(),
        )

    def initialize_pwm_driver(self, params):
        """
        Initialize the PCA9685 PWM driver.

        Args:
            params: Dictionary with parameter values

        Returns:
            PCA9685: PWM driver object
        """
        driver = PCA9685(address=params["i2c_address"], busnum=params["i2c_busnum"])
        driver.set_pwm_freq(params["pwm_freq"])
        return driver

    def setup_subscriptions(self):
        """Set up subscriptions to ROS topics."""
        # Subscribe to control commands
        self.control_cmd_subscription = self.create_subscription(
            Control,
            "~/input/control_cmd",
            self.control_callback,
            1,
        )

        # Subscribe to speed data
        self.speed_subscription = self.create_subscription(
            VelocityReport,
            "~/input/velocity_status",
            self.velocity_callback,
            1,
        )

        # Create debug publishers
        self.debug_control_publisher = self.create_publisher(
            Float32MultiArrayStamped, "~/debug/control_values", 10
        )

        self.debug_pwm_publisher = self.create_publisher(
            Float32MultiArrayStamped, "~/debug/pwm_values", 10
        )

        self.debug_pid_publisher = self.create_publisher(
            Float32MultiArrayStamped, "~/debug/pid_values", 10
        )

    def velocity_callback(self, msg):
        """
        Callback for velocity report messages.
        Updates the current speed state and calculates acceleration.

        Args:
            msg: VelocityReport message containing current vehicle speed
        """
        # Store the last speed before updating
        self.state.last_speed = (
            self.state.current_speed if self.state.current_speed is not None else 0.0
        )

        # Update current speed
        speed = msg.longitudinal_velocity
        self.state.current_speed = speed

        # Calculate time delta
        current_time = time.time()
        delta_time = current_time - self.state.last_update_time

        # Calculate acceleration with smoothing (using a low-pass filter like CARLA does)
        if delta_time > 0 and self.state.current_speed is not None:
            raw_accel = (self.state.current_speed - self.state.last_speed) / delta_time
            # Apply low-pass filter: new_value = 0.8*old_value + 0.2*new_measurement
            self.state.current_acceleration = (
                0.8 * self.state.last_acceleration + 0.2 * raw_accel
            )
            self.state.last_acceleration = self.state.current_acceleration

        # Update timestamp
        self.state.last_update_time = current_time

    def control_callback(self, msg):
        """
        Callback for control command messages.
        Updates the target speed and steering angle state.

        Args:
            msg: Control message containing target velocity and steering angle
        """
        self.state.target_speed = msg.longitudinal.velocity
        self.state.target_tire_angle = msg.lateral.steering_tire_angle

        # Clamp the target steering angle to the vehicle's maximum steering capability
        max_angle = self.config.max_steering_angle
        self.state.target_tire_angle = max(
            -max_angle, min(self.state.target_tire_angle, max_angle)
        )

    def timer_callback(self):
        """
        Timer callback that runs at the configured rate.
        Implements the Ackermann control loop similar to CARLA.
        """
        # Calculate delta time
        current_time = time.time()
        delta_time = current_time - self.state.last_update_time

        # Lateral Control (Steering)
        steer_value = self.run_steering_control(delta_time)

        # Longitudinal Control (Throttle/Brake)
        throttle, brake, in_reverse = self.run_longitudinal_control(delta_time)

        # Update PWM values for the hardware
        self.state.in_reverse = in_reverse

        # Calculate the actual PWM values
        pwm_value = self.convert_throttle_brake_to_pwm(throttle, brake, in_reverse)

        # Set the power of the DC motor
        self.driver.set_pwm(0, 0, pwm_value)

        # Set angle of the steering servo
        self.driver.set_pwm(1, 0, steer_value)

        # Publish debug information if enabled
        if (
            self.get_parameter("enable_debug_publishing")
            .get_parameter_value()
            .bool_value
        ):
            self.publish_debug_control_values(throttle, brake, in_reverse)
            self.publish_debug_pwm_values(pwm_value, steer_value)
            self.publish_debug_pid_values()

    def run_steering_control(self, delta_time: float) -> int:
        """
        Implements the steering control logic using the Ackermann model.

        Args:
            delta_time: Time since last update in seconds

        Returns:
            int: PWM value for the steering servo
        """
        # If no target tire angle available, return initial steering value (straight)
        if self.state.target_tire_angle is None:
            return self.config.init_steer

        # Get the current steering angle
        current_angle = self.state.current_tire_angle or 0.0

        # Determine the target steering angle
        target_angle = self.state.target_tire_angle

        # If steering speed limit is very small, set steering angle directly
        # Otherwise, limit the steering rate to simulate more realistic vehicle dynamics
        if self.config.steering_speed < 0.001:
            # Instant steering (not realistic but simple)
            steer_angle = target_angle
        else:
            # Compute maximum angle change for this time step
            max_angle_change = self.config.steering_speed * delta_time

            # Determine steering direction
            if abs(target_angle - current_angle) < max_angle_change:
                # Close enough to target, set directly
                steer_angle = target_angle
            else:
                # Apply limited steering rate
                steer_direction = 1.0 if target_angle > current_angle else -1.0
                steer_angle = current_angle + steer_direction * max_angle_change

        # Convert the steering angle to PWM value
        steer = steer_angle * self.config.tire_angle_to_steer_ratio
        steer_pwm = self.config.init_steer + int(steer)

        # Limit steering PWM value to min/max range
        steer_pwm = max(self.config.min_steer, min(self.config.max_steer, steer_pwm))

        return steer_pwm

    def run_longitudinal_control(self, delta_time: float) -> Tuple[float, float, bool]:
        """
        Implements the longitudinal control logic (throttle/brake) using cascaded PID controllers.

        Args:
            delta_time: Time since last update in seconds

        Returns:
            Tuple[float, float, bool]: Throttle (0-1), brake (0-1), and reverse flag
        """
        # Return zeros if we don't have necessary state information
        if self.state.target_speed is None or self.state.current_speed is None:
            return 0.0, 0.0, False

        # Check for full stop condition
        if self.run_control_full_stop():
            return 0.0, 1.0, False  # Zero throttle, full brake

        # Handle reverse control
        self.run_control_reverse()

        # Run speed controller to get target acceleration
        self.run_control_speed(delta_time)

        # Run acceleration controller to get pedal position
        self.run_control_acceleration(delta_time)

        # Convert to throttle and brake commands
        throttle, brake = self.update_throttle_brake()

        return throttle, brake, self.state.in_reverse

    def run_control_full_stop(self) -> bool:
        """
        Check if vehicle should be in full stop mode.

        Returns:
            bool: True if full stop should be applied
        """
        # Epsilon threshold for considering vehicle stopped
        full_stop_epsilon = 0.1  # [m/s]

        # Check if both current and target speeds are near zero
        if (
            abs(self.state.current_speed) < full_stop_epsilon
            and abs(self.state.target_speed) < full_stop_epsilon
        ):
            return True

        return False

    def run_control_reverse(self) -> None:
        """
        Handle the logic for changing between forward and reverse gears.
        """
        # Epsilon threshold for considering vehicle stopped
        standing_still_epsilon = 0.1  # [m/s]

        if abs(self.state.current_speed) < standing_still_epsilon:
            # Vehicle is standing still, can change direction
            if self.state.target_speed < 0:
                # Change to reverse
                self.state.in_reverse = True
            elif self.state.target_speed >= 0:
                # Change to forward
                self.state.in_reverse = False
        else:
            # Vehicle is moving, check if we need to stop before changing direction
            if (self.state.current_speed * self.state.target_speed) < 0:
                # Signs are different, which means we're requesting a direction change
                # First we need to stop
                self.state.target_speed = 0.0

    def run_control_speed(self, delta_time: float) -> None:
        """
        Run the speed controller to calculate target acceleration.

        Args:
            delta_time: Time since last update in seconds
        """
        # Set target point for speed controller
        self.speed_controller.set_target_point(self.state.target_speed)

        # Run controller to get acceleration delta
        speed_control_accel_delta = self.speed_controller.run(
            self.state.current_speed, delta_time
        )

        # Update target acceleration
        self.state.speed_control_accel_target += speed_control_accel_delta

        # Limit acceleration to configured constraints
        max_accel = self.config.max_accel
        max_decel = self.config.max_decel

        self.state.speed_control_accel_target = max(
            -max_decel, min(self.state.speed_control_accel_target, max_accel)
        )

    def run_control_acceleration(self, delta_time: float) -> None:
        """
        Run the acceleration controller to calculate pedal position.

        Args:
            delta_time: Time since last update in seconds
        """
        # Set target for acceleration controller
        self.accel_controller.set_target_point(self.state.speed_control_accel_target)

        # Run controller to get pedal delta
        accel_control_pedal_delta = self.accel_controller.run(
            self.state.current_acceleration, delta_time
        )

        # Update pedal target
        self.state.accel_control_pedal_target += accel_control_pedal_delta

        # Limit pedal position to [-1, 1] range
        self.state.accel_control_pedal_target = max(
            -1.0, min(self.state.accel_control_pedal_target, 1.0)
        )

    def update_throttle_brake(self) -> Tuple[float, float]:
        """
        Convert the pedal target to throttle and brake commands.

        Returns:
            Tuple[float, float]: Throttle value (0-1) and brake value (0-1)
        """
        throttle = 0.0
        brake = 0.0

        # Convert pedal target to throttle/brake based on direction
        if self.state.accel_control_pedal_target < 0.0:
            if self.state.in_reverse:
                # In reverse, negative pedal = throttle
                throttle = abs(self.state.accel_control_pedal_target)
                brake = 0.0
            else:
                # In forward, negative pedal = brake
                throttle = 0.0
                brake = abs(self.state.accel_control_pedal_target)
        else:
            if self.state.in_reverse:
                # In reverse, positive pedal = brake
                throttle = 0.0
                brake = abs(self.state.accel_control_pedal_target)
            else:
                # In forward, positive pedal = throttle
                throttle = abs(self.state.accel_control_pedal_target)
                brake = 0.0

        # Limit values to [0, 1] range
        throttle = max(0.0, min(throttle, 1.0))
        brake = max(0.0, min(brake, 1.0))

        return throttle, brake

    def convert_throttle_brake_to_pwm(
        self, throttle: float, brake: float, in_reverse: bool
    ) -> int:
        """
        Convert throttle and brake values to a PWM value for the motor.

        Args:
            throttle: Throttle value (0-1)
            brake: Brake value (0-1)
            in_reverse: Whether vehicle is in reverse gear

        Returns:
            int: PWM value for the motor
        """
        # Calculate the PWM range
        forward_range = self.config.max_pwm - self.config.init_pwm
        backward_range = self.config.init_pwm - self.config.min_pwm

        if in_reverse:
            # In reverse gear
            if throttle > 0:
                # Apply reverse throttle
                pwm_value = self.config.init_pwm - int(throttle * backward_range)
            else:
                # Apply brake while in reverse (slows down reverse movement)
                pwm_value = self.config.init_pwm + int(
                    brake * forward_range * 0.5
                )  # Use less brake when reversing
        else:
            # In forward gear
            if throttle > 0:
                # Apply forward throttle
                pwm_value = self.config.init_pwm + int(throttle * forward_range)
            else:
                # Apply brake
                pwm_value = self.config.init_pwm - int(
                    brake * backward_range * 0.5
                )  # Use less reverse PWM for braking

        # Ensure the PWM value is within the allowed range
        pwm_value = max(self.config.min_pwm, min(pwm_value, self.config.max_pwm))

        return pwm_value

    def reset_controllers(self) -> None:
        """
        Reset all PID controllers to their initial state.
        """
        self.speed_controller.reset()
        self.accel_controller.reset()

        # Reset state variables
        self.state.speed_control_accel_target = 0.0
        self.state.accel_control_pedal_target = 0.0
        self.state.last_acceleration = 0.0
        self.state.in_reverse = False

    def publish_debug_control_values(
        self, throttle: float, brake: float, in_reverse: bool
    ):
        """
        Publish debug information about control values.

        Args:
            throttle: Current throttle value (0-1)
            brake: Current brake value (0-1)
            in_reverse: Current gear state
        """
        msg = Float32MultiArrayStamped()

        # Add timestamp
        msg.stamp = self.get_clock().now().to_msg()

        # Create descriptive layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "control_values"
        msg.layout.dim[0].size = 3
        msg.layout.dim[0].stride = 3

        # Add data: [throttle, brake, in_reverse]
        msg.data = [throttle, brake, 1.0 if in_reverse else 0.0]

        # Publish message
        self.debug_control_publisher.publish(msg)

    def publish_debug_pwm_values(self, motor_pwm: int, steer_pwm: int):
        """
        Publish debug information about PWM values.

        Args:
            motor_pwm: Current motor PWM value
            steer_pwm: Current steering servo PWM value
        """
        msg = Float32MultiArrayStamped()

        # Add timestamp
        msg.stamp = self.get_clock().now().to_msg()

        # Create descriptive layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "pwm_values"
        msg.layout.dim[0].size = 2
        msg.layout.dim[0].stride = 2

        # Add data: [motor_pwm, steer_pwm]
        msg.data = [float(motor_pwm), float(steer_pwm)]

        # Publish message
        self.debug_pwm_publisher.publish(msg)

    def publish_debug_pid_values(self):
        """
        Publish debug information about PID controller values.
        """
        msg = Float32MultiArrayStamped()

        # Add timestamp
        msg.stamp = self.get_clock().now().to_msg()

        # Create descriptive layout
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = "pid_values"
        msg.layout.dim[0].size = 11
        msg.layout.dim[0].stride = 11

        # Add data with the following format:
        # [target_speed, current_speed, target_tire_angle, current_tire_angle,
        #  speed_p, speed_i, speed_d, accel_p, accel_i, accel_d, accel_target]
        target_speed = (
            self.state.target_speed if self.state.target_speed is not None else 0.0
        )
        current_speed = (
            self.state.current_speed if self.state.current_speed is not None else 0.0
        )
        target_tire_angle = (
            self.state.target_tire_angle
            if self.state.target_tire_angle is not None
            else 0.0
        )
        current_tire_angle = (
            self.state.current_tire_angle
            if self.state.current_tire_angle is not None
            else 0.0
        )

        msg.data = [
            target_speed,
            current_speed,
            target_tire_angle,
            current_tire_angle,
            self.speed_controller.proportional,
            self.speed_controller.integral,
            self.speed_controller.derivative,
            self.accel_controller.proportional,
            self.accel_controller.integral,
            self.accel_controller.derivative,
            self.state.speed_control_accel_target,
        ]

        # Publish message
        self.debug_pid_publisher.publish(msg)


@dataclass
class State:
    """
    Dataclass to hold the current state of the controller.

    Attributes:
        target_speed: Target longitudinal velocity from control commands
        current_speed: Current measured vehicle speed
        target_tire_angle: Target steering tire angle from control commands
        current_tire_angle: Current measured tire angle (if available)

        speed_control_accel_target: Target acceleration from speed controller
        accel_control_pedal_target: Target pedal position from acceleration controller
        current_acceleration: Current measured acceleration
        last_speed: Previous measured speed
        last_acceleration: Previous measured acceleration
        in_reverse: Whether vehicle is in reverse gear
        last_update_time: Timestamp of last update
    """

    target_speed: Optional[float]
    current_speed: Optional[float]

    target_tire_angle: Optional[float]
    current_tire_angle: Optional[float]

    # Ackermann-specific state variables
    speed_control_accel_target: float
    accel_control_pedal_target: float
    current_acceleration: float
    last_speed: float
    last_acceleration: float
    in_reverse: bool
    last_update_time: float


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
        max_accel: Maximum allowed acceleration [m/s²]
        max_decel: Maximum allowed deceleration [m/s²]
        steering_speed: Maximum steering speed [rad/s]
        max_steering_angle: Maximum steering angle [rad]
    """

    min_pwm: int
    init_pwm: int
    max_pwm: int

    min_steer: int
    init_steer: int
    max_steer: int

    tire_angle_to_steer_ratio: float

    # Ackermann-specific parameters
    max_accel: float
    max_decel: float
    steering_speed: float
    max_steering_angle: float


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
