[« Back to ros2_ws README](../../README.md)
# robot_teleop

## Overview

This package uses the `joy` and `teleop_twist_joy` packages to enable gamepad control, for both Gazebo and the real robot.

## Dependencies

- `joy` package
- `teleop_twist_joy` package

### Install dependencies:

**Using rosdep (recommended):**
```bash
cd ~/ros2_dev_ws
rosdep install --from-paths src/robot_teleop --ignore-src -r -y
```

**Manual installation (alternative):**
```bash
sudo apt update
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
```

## Controller Configuration

### Logitech F710 Controls

| Control | Function |
|---------|----------|
| **Left Stick (Vertical)** | Linear velocity (forward/backward) |
| **Left Stick (Horizontal)** | Angular velocity (rotation) |
| **RB (Right Bumper)** | Enable button - hold to allow movement |
| **LB (Left Bumper)** | Turbo button - hold to allow faster movement |

### Velocity Limits

**Normal Mode, 50% max speed (RB held):**
- Linear: 0.44 m/s
- Angular: 0.6 rad/s

**Turbo Mode, 100% max speed (LB held):**
- Linear: 0.88 m/s
- Angular: 1.2 rad/s

## Usage

### Basic Launch (Default Settings)

```bash
ros2 launch robot_teleop teleop_joy_launch.py
```

**Defaults:**
- `publish_stamped`: `true`
- `use_sim_time`: `false`
- `cmd_vel_topic`: `/diff_drive_base_controller/cmd_vel`

### Launch with Custom Parameters

**Simulation (Gazebo):**
```bash
ros2 launch robot_teleop teleop_joy_launch.py use_sim_time:=true
```

**Physical Robot:**
```bash
ros2 launch robot_teleop teleop_joy_launch.py use_sim_time:=false
```

**Multiple parameters:**
```bash
ros2 launch robot_teleop teleop_joy_launch.py \
    publish_stamped:=false \
    use_sim_time:=false \
    cmd_vel_topic:=/custom_controller/cmd_vel
```

## Launch Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `publish_stamped` | bool | `true` | Publish stamped Twist messages (TwistStamped) |
| `use_sim_time` | bool | `false` | Use simulation time instead of system time |
| `cmd_vel_topic` | string | `/diff_drive_base_controller/cmd_vel` | Target topic for velocity commands |

## Customization

### Adjusting Velocity Limits

Edit `config/f710_config.yaml` to modify speed limits:

```python
teleop_twist_joy_node:
  ros__parameters:
    scale_linear:
      x: 0.44  # Normal linear speed (m/s)
    scale_linear_turbo:
      x: 0.88  # Turbo linear speed (m/s)
    
    scale_angular:
      yaw: 0.6   # Normal angular speed (rad/s)
    scale_angular_turbo:
      yaw: 1.2   # Turbo angular speed (rad/s)
```
Note: When changing velocity limits make sure to look at controller config file in the robot_bringup package

### Changing Button Mappings

Modify button indices in `config/f710_config.yaml`:

```python
enable_button: 5        # Left bumper (LB)
enable_turbo_button: 4  # Right bumper (RB)
```

**F710 Button Reference:**
- 0: A
- 1: B
- 2: X
- 3: Y
- 4: LB (Left Bumper)
- 5: RB (Right Bumper)
- 6: Back
- 7: Start

### Changing Joystick Axes

Modify axis indices in `config/f710_config.yaml`:

```python
axis_linear:
  x: 1   # Left stick vertical
axis_angular:
  yaw: 0 # Left stick horizontal
```

**F710 Axis Reference (DirectInput mode):**
| Axis | Control | Range |
|------|---------|-------|
| 0 | Left stick horizontal | +1.0 (left) to -1.0 (right) |
| 1 | Left stick vertical | +1.0 (up) to -1.0 (down) |
| 2 | Left trigger | +1.0 (up) to -1.0 (down) |
| 3 | Right stick horizontal | +1.0 (left) to -1.0 (right) |
| 4 | Right stick vertical | +1.0 (up) to -1.0 (down) |
| 5 | Right trigger | +1.0 (up) to -1.0 (down) |
| 6 | D-pad horizontal | +1.0 (left) to -1.0 (right) |
| 7 | D-pad vertical | +1.0 (up) to -1.0 (down) |

Note: some axis or signs may be wrong, tested on 8bitdo gamepad

## Troubleshooting

### Controller Not Detected

```bash
# Check if joystick is recognized
ls -l /dev/input/js*

# Test joystick input
ros2 run joy joy_node
ros2 topic echo /joy
```
### Gamepad only outputting max values
Sometimes when the controller is first connected, the joysticks may output maximum values instead of a reading based on its position.

Solution: Flick both joysticks in all directions. The controller should then output values proportional to stick movement rather than only the maximum.

### No Movement

1. **Check enable button**: Hold RB (right bumper) or LB (left bumper) while moving joystick
2. **Verify topic name**: Ensure `cmd_vel_topic` matches your controller's expected topic
3. **Test velocity output**: Check if commands are being published:
   ```bash
   ros2 topic echo /diff_drive_base_controller/cmd_vel
   ```
   You should see velocity values when moving the joystick with the enable or turbo button held.
3. **Check permissions**: Add user to input group if needed:
   ```bash
   sudo usermod -a -G input $USER
   # Log out and log back in
   ```

### Wrong Gamepad Mode

Ensure F710 switch on back is set to **DirectInput (D)** mode, not XInput (X). The switch is located at the top of the device.

Ensure the mode light on the front of the controller is off, may flash when the battery is low.

## Topics

**Published:**
- `/cmd_vel` or `/cmd_vel_stamped` &rarr; Remapped to configured `cmd_vel_topic`

**Subscribed:**
- `/joy` (from joy_node)

## Package Structure

```
robot_teleop/
├── config/
│   └── f710_config.yaml      # Controller configuration
├── launch/
│   └── teleop_joy_launch.py  # Launch file
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
