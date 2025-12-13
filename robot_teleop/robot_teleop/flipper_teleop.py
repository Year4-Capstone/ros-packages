import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class FlipperTeleop(Node):
    def __init__(self):
        super().__init__('flipper_teleop')

        # Subscribe to Joystick
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Subscribe to Joint States (to sync initial position)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.state_callback, 10)

        # Publisher to the controller defined in Step 1
        self.publisher_ = self.create_publisher(Float64MultiArray, '/flipper_position_controller/commands', 10)

        # Flipper targets: [FL, FR, RL, RR]
        self.target_positions = [0.0, 0.0, 0.0, 0.0] 
        self.initialized = False
        
        # Mapping for Logitech F710 (XInput Mode usually)
        # Verify these with "ros2 topic echo /joy"
        self.BTN_A = 0
        self.BTN_B = 1
        self.BTN_X = 2
        self.BTN_Y = 3
        self.AXIS_DPAD_LR = 6  # Left/Right
        self.AXIS_DPAD_UD = 7  # Up/Down

        self.MOVEMENT_STEP = 0.05  # Radians per button press loop
        
        # Mapping indices to joints
        # 0: FL (A), 1: FR (B), 2: RL (X), 3: RR (Y)
        self.joint_indices = {
            'front_left_flipper_flipper_joint': 0,
            'front_right_flipper_flipper_joint': 1,
            'rear_left_flipper_flipper_joint': 2,
            'rear_right_flipper_flipper_joint': 3
        }

    def state_callback(self, msg):
        # One-time initialization to prevent jumping to 0.0 on startup
        if not self.initialized:
            for i, name in enumerate(msg.name):
                if name in self.joint_indices:
                    idx = self.joint_indices[name]
                    self.target_positions[idx] = msg.position[i]
            self.initialized = True
            self.get_logger().info("Flippers synced to current hardware positions.")

    def joy_callback(self, msg):
        if not self.initialized:
            return

        # 1. Reset Logic (Left/Right)
        if msg.axes[self.AXIS_DPAD_LR] > 0.5: # Left
            self.target_positions = [0.0, 0.0, 0.0, 0.0] # Preset 1 (Flat)
        elif msg.axes[self.AXIS_DPAD_LR] < -0.5: # Right
            self.target_positions = [1.57, 1.57, 1.57, 1.57] # Preset 2 (Up)
        
        # 2. Incremental Move Logic (Up/Down)
        dpad_val = msg.axes[self.AXIS_DPAD_UD]
        
        if abs(dpad_val) > 0.5:
            delta = self.MOVEMENT_STEP if dpad_val > 0 else -self.MOVEMENT_STEP
            
            # Check buttons for individual selection
            fl_active = msg.buttons[self.BTN_A]
            fr_active = msg.buttons[self.BTN_B]
            rl_active = msg.buttons[self.BTN_X]
            rr_active = msg.buttons[self.BTN_Y]
            
            any_button_pressed = fl_active or fr_active or rl_active or rr_active

            # If NO buttons pressed, move ALL
            if not any_button_pressed:
                for i in range(4):
                    self.target_positions[i] += delta
            else:
                # Move only selected
                if fl_active: self.target_positions[0] += delta
                if fr_active: self.target_positions[1] += delta
                if rl_active: self.target_positions[2] += delta
                if rr_active: self.target_positions[3] += delta

        # 3. Publish Command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.target_positions
        self.publisher_.publish(cmd_msg)

def main():
    rclpy.init()
    node = FlipperTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()