# phidgets_test

## Overview

ROS2 package for controlling Phidget BLDC motor controllers and digital input limit switches. This package provides motor control for both drive and actuation systems, with automated homing sequences for actuation motors using limit switches.

**Key Features:**
- Unified motor driver supporting both drive and actuation motors
- Limit switch integration for homing sequences
- ROS2 action-based homing with real-time feedback
- Python visualization tools for homing diagnostics

## Hardware Setup

### Hardware
- 2x Phidget VINT Hub (Serial Numbers: 527103, 527164)
- 8x Phidget BLDC Motor Controllers
  - 4x Drive motors (ports 2-3 on each hub)
  - 4x Actuation motors (ports 0-1 on each hub)
- 4x Phidget Digital Input limit switches (ports 4-5 on each hub)

TODO: Add model #s

### Port Configuration

**Hub 1 (Serial: 527103):**
- Port 0: Back Left actuation motor
- Port 1: Back Right actuation motor
- Port 2: Back Left drive motor
- Port 3: Back Right drive motor
- Port 4: Front Right limit switch
- Port 5: Back Left limit switch

**Hub 2 (Serial: 527164):**
- Port 0: Front Left actuation motor
- Port 1: Front Right actuation motor
- Port 2: Front Left drive motor
- Port 3: Front Right drive motor
- Port 4: Back Right limit switch
- Port 5: Front Left limit switch

TODO: Verify if the port config locations are correct, type of device is correct
## Dependencies

### ROS2 Dependencies
- `rclcpp` - ROS2 C++ client library
- `rclcpp_action` - ROS2 action server/client support
- `geometry_msgs` - For Twist messages (drive control)
- `sensor_msgs` - For Joy messages (manual actuation control)
- `rclpy` - ROS2 Python client library (for visualization)
- `rosidl_default_generators` - For custom action messages
- `robot_teleop` - Custom package for gamepad teleoperation


### System Dependencies
- Phidget22 C library (`libphidget22`)

### Install Dependencies

### ROS2 Dependencies
**Using rosdep (recommended):**
```bash
cd ~/ros2_dev_ws
rosdep install --from-paths src/phidgets_test --ignore-src -r -y
```

**ROS2 dependencies (manual):**
```bash
sudo apt install ros-jazzy-rclcpp ros-jazzy-rclcpp-action \
                 ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs
```

### System Dependencies
**Manual Phidget22 installation:**

Assuming the terminal window is Non-Root
```bash
# Download and install Phidget22 libraries
# Visit: https://www.phidgets.com/docs/Operating_System_Support
# For Ubuntu/Debian:
curl -fsSL https://www.phidgets.com/downloads/setup_linux | sudo -E bash -
sudo apt install -y libphidget22 libphidget22-dev
```

**Note:** Ensure `robot_teleop` package is built before building `phidgets_test` if using manual actuation control.

## Phidget Control Panel (GUI)

Phidgets provides a graphical control application for testing and configuration:

**Installation:**
- Download from: https://www.phidgets.com/docs/OS_-_Windows#Getting_started_with_Windows
- Available for Windows 10/11, not Ubuntu

**Features:**
- Real-time motor control and monitoring
- Limit switch state testing
- Firmware updates for Phidget devices

## Usage

### Building the Package

```bash
cd ~/ros2_dev_ws
colcon build --packages-select robot_teleop phidgets_test
source install/setup.bash
```

### Testing Individual Components

**Test Motor Driver:**
```bash
ros2 run phidgets_test test_phidgets_motors
```
Tests initialization and basic velocity control of all 8 motors.

**Note:** Robot treads must be off the ground to perform this test.

**Test Limit Switches:**
```bash
ros2 run phidgets_test limit_switch_test
```
Displays real-time state of all 4 limit switches. Press switches to see state changes.

### Manual Actuation Control

For manual testing or positioning of actuation motors:

**Terminal 1:** Start joy node (requires gamepad connected)
```bash
ros2 launch robot_teleop teleop_joy_launch.py
```
**Note:** Review robot_teleop README.md for more information about the launch file

**Terminal 2:**
```bash
ros2 run phidgets_test actuation_sub
```

**Default Controls:**
- Button 0 ('A'): Move legs up
- Button 1 ('B'): Move legs down
- Speed: 0.4 duty cycle

**Note:** Update button mappings in `actuation_sub.cpp` to match robot_teleop.

### Running Automated Homing

The recommended way to run homing is using the visualization tool:

**Terminal 1: Start homing action server**
```bash
ros2 run phidgets_test actuation_homing_node
```
**Terminal 2: Run visualization (automatically sends homing goal)**
```bash
ros2 run phidgets_test plot_homing_positions.py
```

The visualization will:
1. Connect to the homing action server
2. Send a homing goal with default speed (0.1 duty cycle)
3. Display live position plots for all 4 actuators
4. Show status indicators (Not Started, In Progress, Completed, Failed)
5. Display final positions after homing completes

## Homing Sequence Details

### Homing Algorithm

For each actuator:
1. Move toward limit switch (negative direction) at specified speed
2. Wait for limit switch activation
3. Stop motor immediately upon switch activation
4. Back off by 7.5° in positive direction
5. Stop motor
6. Reset encoder position to 0°

All 4 actuators home in parallel.

### Action Interface

**Goal:**
```python
float64 homing_speed  # Speed for homing (duty cycle: 0.0-1.0)
```

**Feedback:**
```python
uint8[] status        # Per-leg status: 0=not started, 1=in progress, 2=completed, 3=failed
float64[] positions   # Current positions in degrees
```

**Result:**
```python
bool[] success        # Success flag for each leg
float64[] positions   # Final positions after homing
```

### Customizing Homing Parameters

Edit `actuation_homing_node.cpp`:

```cpp
const double BACKOFF_ANGLE = 7.5;  // Degrees to back off from limit switch
```

Change homing speed when calling:
```cpp
node.send_homing_goal(homing_speed=0.1); // Adjust between 0.0-1.0
```
**TODO:** Bracking is not enabled in current firmware, this limits the safe max homing speed.  

## Motor Configuration

Motor parameters are defined in `phidgets_motor_driver.hpp`:

### Drive Motor Configuration
```cpp
static constexpr MotorConfig DRIVE_CONFIG {
    82,                    // max_rpm
    0.638297872340426,     // rescale_factor (comms/s to deg/s)
    5.0,                   // stall_velocity (deg/s)
    1.0,                   // duty_limit (max duty cycle)
};
```

### Actuation Motor Configuration
```cpp
static constexpr MotorConfig ACT_CONFIG {
    170,                   // max_rpm
    0.02173913,            // rescale_factor
    2.0,                   // stall_velocity (deg/s)
    1.0,                   // duty_limit (max duty cycle)
};
```
**TODO:** stall velocity is not working with current firmware, or current error checking is wrong. If motors sound like they are struggling stop what you are doing.

## API Reference

### PhidgetMotorController

**Constructor:**
```cpp
PhidgetMotorController(int serial_number, int hub_port, MotorType type, int motor_direction)
```

**Methods:**
```cpp
void init()                              // Initialize and attach to motor
void setVelocityDuty(double duty)       // Set velocity as duty cycle (-1.0 to 1.0)
void setVelocityRPM(double rpm)         // Set velocity in RPM
void setVelocityRads(double rads_per_sec) // Set velocity in rad/s
double getPositionDegs()                 // Get position in degrees
double getPositionRads()                 // Get position in radians
void resetPosition()                     // Reset encoder to 0
void cleanup()                           // Stop motor and release resources
```

### PhidgetLimitSwitch

**Constructor:**
```cpp
PhidgetLimitSwitch(int serial_number, int hub_port)
```

**Methods:**
```cpp
void init()                              // Initialize and attach to switch
bool read() const                        // Read current state (true=pressed)
void setCallback(std::function<void(int)> cb) // Set state change callback
void cleanup()                           // Release resources
```

## Troubleshooting

### Motors Not Responding

1. **Check Phidget connections:**
   ```bash
   # List connected Phidget devices
   lsusb | grep Phidget
   ```

2. **Verify serial numbers and ports:**
   - Use Phidget Control Center (Windows) to identify devices
   - Check that serial numbers in code match physical hubs
   - Verify hub port connections (0-5)

3. **Check permissions:**
   ```bash
   # Add user to dialout group for USB access
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

4. **Test with simple program:**
   ```bash
   ros2 run phidgets_test test_phidgets_motors
   ```

### Limit Switches Not Triggering

1. **Verify switch connections:**
   ```bash
   ros2 run phidgets_test limit_switch_test
   ```
   Manually press switches and verify state changes are printed.

2. **Check wiring:** Ensure limit switches are properly wired to digital input ports 4-5

### Homing Fails or Doesn't Complete

1. **Check limit switch functionality first:**
   ```bash
   ros2 run phidgets_test limit_switch_test
   ```

2. **Verify motor direction:** Motors should move toward limit switches in negative direction

4. **Check limit switch cams:** Ensure actuators shafts have limit siwthc cams in the correct orientation

5. **Review feedback:** Use `plot_homing_positions.py` to see which leg is failing

### Homing Action Server Not Available

```bash
# Check if node is running
ros2 node list | grep homing

# Check if action is available
ros2 action list

# Check action server info
ros2 action info /homing_sequence
```

### USB Communication Errors

If you see "Failed to attach" or timeout errors:

1. **Unplug and replug USB connections**
2. **Restart the Phidget VINT hubs** (power cycle robot)

## Topics and Actions

### Actions

- `/homing_sequence` (phidgets_test/action/HomingSequence)
  - Server: `actuation_homing_node`
  - Client: `plot_homing_positions.py`

### Subscribed Topics

- `/joy` (sensor_msgs/msg/Joy)
  - Subscriber: `actuation_sub`
  - Used for manual actuation control

## Package Structure

```python
phidgets_test/
├── action/
│   └── HomingSequence.action        # Custom action for homing sequence
├── include/phidgets_test/
│   ├── phidgets_motor_driver.hpp    # [ACTIVE] Unified motor driver
│   ├── phidgets_limit_switch.hpp    # [ACTIVE] Limit switch dirver
│   ├── phidgets_bldc_driver.hpp     # [DEPRECATED] Old motor driver
│   └── phidgets_actuaction_driver.hpp # [LEGACY] Actuator motor specific driver
├── src/
│   ├── actuation_homing_node.cpp    # [ACTIVE] Homing action server
│   ├── actuation_sub.cpp            # [LEGACY] Manual actuation control via joy
│   ├── plot_homing_positions.py     # [ACTIVE] Visualization for homing sequence
│   ├── homing_client.cpp            # [DEPRECATED] Replaced by plot_homing_positions.py
│   ├── motor_sub.cpp                # [DEPRECATED] Old drive motor subscriber
│   ├── test_bldc_motors.cpp         # [DEPRECATED] Old test file
│   ├── test_phidgets_motor_driver.cpp # [TEST ONLY] Motor driver testing
│   └── test_limit_switch.cpp        # [TEST ONLY] Limit switch testing
├── CMakeLists.txt
├── package.xml
└── README.md
```
**File Status Legend:**
- **[ACTIVE]** - Current implementation, use for new development
- **[LEGACY]** - Still functional but being phased out
- **[DEPRECATED]** - Has been phased out, do not use
- **[TEST ONLY]** - Standalone test programs for development/debugging

## Core Components

### Active Headers

**`phidgets_motor_driver.hpp`**
- Unified interface for both drive and actuation motors
- Supports velocity control in duty cycle, RPM, or rad/s
- Position reading in degrees or radians
- Configuration per motor type (drive vs actuation)

**`phidgets_limit_switch.hpp`**
- Digital input interface for limit switches
- Callback-based state change notifications

### Active Nodes

**`actuation_homing_node.cpp`**
- ROS2 action server for automated homing sequences
- Homes all 4 actuation motors in parallel
- Provides real-time feedback on homing progress
- Configurable homing speed

**`plot_homing_positions.py`**
- Real-time visualization of homing sequence
- Plots position vs time for all 4 actuators
- Status indicators for each leg
- Action client + live matplotlib plotting

**`actuation_sub.cpp`**
- Manual control of actuation motors via joystick
- Subscribes to `/joy` topic
- Allows manual leg positioning for testing
- Controls all four legs at once, using duty-cycle control