# robot_bringup

## Overview

Hardware interface and launch package for the Ibex robot using ros2_control. Provides integration between ROS2 controllers and Phidget BLDC motor controllers for differential drive operation.

## Hardware Setup

### Hub Configuration

**Hub 1 (Serial: 527103):**
- Port 2: Back Left wheel
- Port 3: Back Right wheel

**Hub 2 (Serial: 527164):**
- Port 2: Front Left wheel
- Port 3: Front Right wheel

**Note:** port configuration might be wrong

## Dependencies

### ROS2 Dependencies
- `rclcpp` - ROS2 C++ client library
- `rclcpp_lifecycle` - Lifecycle node support
- `hardware_interface` - ros2_control hardware interface
- `pluginlib` - Plugin loading system
- `controller_manager` - Controller lifecycle management
- `diff_drive_controller` - Differential drive controller
- `joint_state_broadcaster` - Joint state publisher
- `robot_state_publisher` - TF broadcaster
- `robot_description` - Robot URDF package
- `xacro` - URDF processing

### Custom Package Dependencies
- `phidgets_test` - Phidget motor driver library
- `robot_description` - URDF for the IBEX
- `robot_teleop` - Converts gamepad into linear and angular velocities


### System Dependencies
- Phidget22 C library (`libphidget22`)

### Install Dependencies

**Using rosdep (recommended):**
```bash
cd ~/ros2_dev_ws
rosdep install --from-paths src/robot_bringup --ignore-src -r -y
```

**Manual Phidget22 installation:**
View README.md for phidgets test if required

**Note:** Ensure `phidgets_test` and `robot_description` packages are built before building `robot_bringup`.

## Usage

### Building the Package

```bash
cd ~/ros2_dev_ws
colcon build --packages-select robot_teleop phidgets_test robot_description robot_bringup
source install/setup.bash
```

### Launching the Robot
**Note:** Review README.md for the phidgets_test package, ideally motors should be homed before driving

**Terminal 1:** Start joy node (requires gamepad connected)
```bash
ros2 launch robot_teleop teleop_joy_launch.py
```
**Note:** Review robot_teleop README.md for more information about the launch file.

**Note:** Ensure the controller is behaving correctly before moving on.

**Terminal 2:** Basic launch (hardware + visualization):
```bash
ros2 launch robot_bringup ibex.launch.py
```

This starts:
- **Controller Manager** - Manages ros2_control lifecycle
- **Hardware Interface** - Connects to Phidget motors
- **Joint State Broadcaster** - Publishes joint states to `/joint_states`
- **Diff Drive Controller** - Provides velocity control via `/diff_drive_base_controller/cmd_vel`
- **Robot State Publisher** - Publishes TF transforms
- **RViz2** - Visualization

### Visualization Tools

**Joint velocity tracking:**
```bash
ros2 run robot_bringup plot_joint_velocities.py
```
Shows actual vs commanded velocities for all 4 wheels with tracking error statistics.

**Robot base velocity tracking:**
```bash
ros2 run robot_bringup plot_robot_velocities.py
```
Shows actual vs commanded linear and angular velocities from odometry.

### Checking System Status

**List active controllers:**
```bash
ros2 control list_controllers
```

**Check controller status:**
```bash
ros2 control list_hardware_interfaces
```

**Monitor joint states:**
```bash
ros2 topic echo /joint_states
```

**Monitor odometry:**
```bash
ros2 topic echo /diff_drive_base_controller/odom
```

## Controller Configuration

### Differential Drive Parameters

Edit `config/diff_drive_controller.yaml`:

```yaml
diff_drive_base_controller:
  ros__parameters:
    # Wheel configuration
    left_wheel_names: ["front_left_wheel_joint", "rear_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "rear_right_wheel_joint"]
    
    # Physical dimensions
    wheel_separation: 0.609  # meters (distance between left and right wheels)
    wheel_radius: 0.102      # meters
    
    # Calibration multipliers
    wheel_separation_multiplier: 2.40  # Adjust for slip/skid
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    
    # Velocity limits (matching motor capabilities)
    linear.x.max_velocity: 0.88    # m/s (100% motor speed)
    angular.z.max_velocity: 1.2    # rad/s
```

### Update Rate

```yaml
controller_manager:
  ros__parameters:
    update_rate: 10  # Hz - frequency of read/write cycles
```

**Note:** Higher update rates provide smoother control but is limited by the Phidgets Hubs

## Hardware Interface

### Implementation

The hardware interface (`hardware/diffbot_system.cpp`) provides:

**Lifecycle States:**
- `on_configure()` - Initialize Phidget motor controllers
- `on_activate()` - Enable motors and reset positions
- `on_deactivate()` - Stop motors and cleanup

**Control Loop:**
- `read()` - Read encoder positions, calculate velocities
- `write()` - Send velocity commands to motors

## Troubleshooting

**TODO**

## Topics and Services

### Published Topics

- `/joint_states` (sensor_msgs/msg/JointState)
  - Current state of all joints
  - Published by joint_state_broadcaster

- `/diff_drive_base_controller/odom` (nav_msgs/msg/Odometry)
  - Robot odometry (position and velocity)
  - Published by diff_drive_controller

- `/tf` (tf2_msgs/msg/TFMessage)
  - Transform tree
  - Published by robot_state_publisher and diff_drive_controller

### Subscribed Topics

- `/diff_drive_base_controller/cmd_vel` (geometry_msgs/msg/TwistStamped)
  - Velocity commands for robot base
  - Subscribed by diff_drive_controller

### Services

- `/controller_manager/*` - Controller lifecycle management
- `/diff_drive_base_controller/*` - Controller-specific services

## Package Structure

```
robot_bringup/
├── config/
│   └── diff_drive_controller.yaml    # Controller configuration
├── hardware/
│   ├── include/
│   │   └── diffbot_system.hpp        # Hardware interface header
│   └── diffbot_system.cpp            # Hardware interface implementation
├── launch/
│   └── ibex.launch.py                # Main launch file
├── scripts/
│   ├── plot_joint_velocities.py      # Joint velocity visualization
│   └── plot_robot_velocities.py      # Robot velocity visualization
├── ibex_control.xml                  # Plugin description
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Integration with Other Packages

### Required Packages

**Build-time:**
- `phidgets_test` - Motor driver headers
- `robot_description` - URDF for controller configuration

**Runtime:**
- `robot_teleop` - Joystick control (optional)

