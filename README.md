# Golf Cart Vehicle Interface

This package provides the vehicle interface components for the Golf Cart autonomous vehicle platform. It enables communication between Autoware's control commands and the vehicle's hardware actuators, as well as reporting the vehicle's velocity back to the Autoware stack.

## Overview

The Golf Cart vehicle interface consists of:

1. **Vehicle Description**: URDF and configuration files describing the vehicle's physical properties
2. **Vehicle Interface**: ROS2 nodes that interface with the vehicle's hardware
3. **Launch Files**: Configuration for launching the vehicle interface components

## Components

### 1. Golf Cart Vehicle Description

Located in `golfcart_vehicle_description/`, this package contains:

- URDF models for the vehicle (`urdf/vehicle.xacro`)
- Vehicle information parameters (`config/vehicle_info.param.yaml`)
- 3D meshes of the vehicle (`mesh/`)

### 2. Golf Cart Vehicle Interface

Located in `golfcart_vehicle_interface/`, this package contains:

- **Actuator Node**: Controls the vehicle's motor and steering servo using PWM signals via a PCA9685 PWM driver
- **Velocity Report Node**: Reads hall effect sensor data to calculate and report the vehicle's velocity

#### Actuator Node (`actuator.py`)

The actuator node subscribes to Autoware control commands and converts them into PWM signals for the vehicle's:
- Throttle/brake control (DC motor)
- Steering control (servo motor)

Features:
- Uses PID controllers for accurate speed and steering control
- Configurable parameters for different vehicles and motor setups
- Interfaces with the PCA9685 PWM driver connected via I2C

Key parameters (in `params/actuator.yaml`):
- PWM frequency and I2C settings
- PID controller parameters for speed and steering
- PWM range settings for motors and servos

#### Velocity Report Node (`velocity_report.py`)

This node:
- Reads from a hall effect sensor connected to a GPIO pin
- Calculates vehicle velocity based on wheel rotation
- Publishes velocity reports to the Autoware stack

Key parameters (in `params/velocity_report.yaml`):
- GPIO pin configuration
- Wheel diameter and markers per rotation
- Publication rate settings

### 3. Golf Cart Vehicle Launch

Located in `golfcart_vehicle_launch/`, this package contains:
- `launch/vehicle_interface.launch.xml` — the interface node and its topic
  remappings. Included by the Autoware stack and by the standalone launch below,
  so the node is defined in exactly one place.
- `launch/vehicle_interface_standalone.launch.xml` — the interface on its own,
  for bench work and bring-up, with optional robot description and velocity
  converter.
- `scripts/teleop_gui.py` — Tk teleop, runnable with
  `ros2 run golfcart_vehicle_launch teleop_gui.py` when a display is available.

## Usage

### Launching the Vehicle Interface

Inside the Autoware stack the interface comes up with the rest of the system.
On its own, use the recipe:

```bash
just vehicle-interface                # CAN RX only, nothing can move
just vehicle-interface can=vcan0      # bench, against mock_vcu
just vehicle-interface tx=on          # TX live: this drives the cart
just vehicle-interface converter=on   # + robot_state_publisher + velocity converter
```

which wraps:

```bash
ros2 launch golfcart_vehicle_launch vehicle_interface_standalone.launch.xml \
    can_interface:=can0 tx_enabled:=false
```

`tx_enabled` defaults to false everywhere: the node runs its full logic and
publishes `/vehicle/status/*`, but skips the socket write, so the cart cannot be
commanded into motion until you ask for it.

Keyboard control is a separate recipe, run in a second terminal:

```bash
just manual-control
```

It is not a node in this launch file. Keys are read from a raw tty, which no
launch-managed process owns — and `play_launch`, which runs the full stack, does
not support `launch-prefix`, so a terminal cannot be handed to it that way either.

See [docs/design/vehicle_interface_standalone.md](../../../docs/design/vehicle_interface_standalone.md).

### Integration with Autoware

The vehicle interface integrates with Autoware through the following topics:

- Input: `/control/command/control_cmd` - Control commands from Autoware
- Output: `/vehicle/status/velocity_status` - Vehicle velocity information

## Configuration

The behavior of the vehicle interface can be customized by modifying the parameter files:

- `params/actuator.yaml`: Configure motor control settings, PID parameters, and PWM ranges
- `params/velocity_report.yaml`: Configure velocity reporting settings, wheel dimensions, and sensor settings

## Hardware Requirements

The vehicle interface is designed to work with:

- PCA9685 PWM driver for motor and servo control (connected via I2C)
- Hall effect sensor (KY-003) for velocity measurement (connected to GPIO)
- DC motor and servo motor for vehicle movement and steering

## Dependencies

- ROS 2 Humble
- Adafruit_PCA9685 Python library
- simple_pid Python library
- Jetson.GPIO Python library
