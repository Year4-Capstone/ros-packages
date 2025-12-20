[« Back to ros2_ws README](../../README.md)
# robot_description

## Overview

URDF description package for the Ibex robot platform. Provides robot visualization in RViz with support for ros2_control integration.

## Hardware

### Robot Platform
- **Name:** Ibex
- **Type:** 4-wheel differential drive with fixed flipper tracks
- **Sensors:** IMU, 2D LiDAR

### Coordinate Frames
- `base_footprint` - Ground projection
- `base_link` - Main chassis
- `{front/rear}_{left/right}_wheel_link` - Four wheels
- `{front/rear}_{left/right}_flipper` - Four flipper tracks
- `imu_link` - IMU sensor (0.45, -0.30, 0.07)
- `laser` - LiDAR sensor (0.45, 0, 0.12)

## Dependencies

### ROS2 Dependencies
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher` - Publishes joint states
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - Robot visualization
- `xacro` - URDF macro processing

### Install Dependencies

**Using rosdep (recommended):**
```bash
cd ~/ros2_dev_ws
rosdep install --from-paths src/robot_description --ignore-src -r -y
```

**Manual installation:**
```bash
sudo apt install ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 \
                 ros-jazzy-xacro
```

## Usage

### Building the Package

```bash
cd ~/ros2_dev_ws
colcon build --packages-select robot_description
source install/setup.bash
```

### Visualizing the Robot

**Basic visualization:**
```bash
ros2 launch robot_description view_robot_rviz.launch.py
```

This opens:
- **RViz2** - 3D visualization of robot model
- **Joint State Publisher GUI** - Sliders to control wheel positions

## Customization

### Adjusting Sensor Positions

**IMU position:**
```xml
<joint name="imu_joint" type="fixed">
    <origin xyz="0.45 -0.30 0.07" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="imu_link" />
</joint>
```

**LiDAR position:**
```xml
<joint name="laser_joint" type="fixed">
    <origin xyz="0.45 0 0.12" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="laser" />
</joint>
```

## Troubleshooting

### Robot Not Displaying in RViz

1. **Verify launch file started correctly:**
   ```bash
   ros2 launch robot_description view_robot_rviz.launch.py
   ```

2. **Check RViz configuration:**
   - Fixed Frame should be `base_footprint` or `base_link`
   - RobotModel display is enabled
   - No red errors in Displays panel

3. **Verify URDF syntax:**
   ```bash
   check_urdf $(ros2 pkg prefix robot_description)/share/robot_description/urdf/ibex.urdf.xacro
   ```

### Missing Meshes

If error messages mention missing mesh files:

1. **Verify meshes installed:**
   ```bash
   ls $(ros2 pkg prefix robot_description)/share/robot_description/meshes/
   ```

2. **Check mesh paths use correct format:**
   ```xml
   package://robot_description/meshes/filename.stl
   ```

3. **Rebuild package:**
   ```bash
   cd ~/ros2_dev_ws
   colcon build --packages-select robot_description --cmake-clean-cache
   source install/setup.bash
   ```

### TF Transform Errors

1. **Check robot_state_publisher running:**
   ```bash
   ros2 node list | grep robot_state_publisher
   ```

2. **Verify joint states publishing:**
   ```bash
   ros2 topic echo /joint_states
   ```

3. **View TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   # Opens frames.pdf showing transform tree
   ```

### URDF Processing Errors

1. **Check xacro syntax**
2. **Verify included files exist:**
   - `ibex_core.urdf.xacro`
   - `ibex_ros2_control.urdf.xacro`

## ros2_control Integration

### Hardware Interface

Uses custom hardware plugin:
```xml
<plugin>ibex_control/DiffBotSystemHardware</plugin>
```

### Motor Controller Configuration

**Phidget Hub 1 (Serial: 527103):**
- Port 2: Back Left wheel
- Port 3: Back Right wheel

**Phidget Hub 2 (Serial: 527164):**
- Port 2: Front Left wheel
- Port 3: Front Right wheel

### Control Interfaces

**Command Interfaces:**
- Velocity control for each wheel (duty cycle: -1 to 1)

**State Interfaces:**
- Position (radians)
- Velocity (rad/s)

## URDF Structure

### File Organization

**ibex.urdf.xacro** - Main entry point
- Includes `ibex_core.urdf.xacro`
- Includes `ibex_ros2_control.urdf.xacro`

**ibex_core.urdf.xacro** - Robot description
- Link and joint definitions
- Macros for wheels and flippers
- Mesh files and materials
- Inertial properties

**ibex_ros2_control.urdf.xacro** - Control interface
- Hardware plugin configuration
- Motor controller parameters
- Command/state interfaces

## Package Structure

```python
robot_description/
├── launch/
│   └── view_robot_rviz.launch.py    # Visualization launch file
├── meshes/
│   ├── base_link.stl                # Main chassis mesh
│   ├── wheel_*.stl                  # Wheel meshes (4 files)
│   └── flipper_*.stl                # Flipper meshes (4 files)
├── rviz/
│   └── ibex.rviz                    # RViz configuration
├── urdf/
│   ├── ibex.urdf.xacro              # Main URDF (includes other files)
│   ├── ibex_core.urdf.xacro         # Robot geometry and properties
│   └── ibex_ros2_control.urdf.xacro # Hardware interface configuration
├── CMakeLists.txt
├── package.xml
└── README.md
```