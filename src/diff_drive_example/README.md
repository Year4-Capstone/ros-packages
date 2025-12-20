[« Back to ros2_ws README](../../README.md)
# diff_drive_example

## Overview

Simple differential drive robot example using ros2_control and Gazebo simulation. Based on the [Gazebo ROS2 Integration tutorial](https://gazebosim.org/docs/latest/ros2_integration/).

## Dependencies

**Using rosdep (recommended):**
```bash
cd ~/ros2_dev_ws
rosdep install --from-paths src/robot_bringup --ignore-src -r -y
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select diff_drive_example
source install/setup.bash
```

## Usage

### Launch the simulation:
```bash
ros2 launch diff_drive_example diff_drive.launch.py
```

### Control the robot:
**Turn in place**
```bash
ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0}, angular: {z: 0.5}}}"
```

## Useful Commands

```bash
# List controllers
ros2 control list_controllers

# Monitor odometry
ros2 topic echo /diff_drive_base_controller/odom

# Monitor joint states
ros2 topic echo /joint_states
```