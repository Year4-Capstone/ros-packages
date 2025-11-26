#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

class JointVelocityPlotter(Node):
    def __init__(self):
        super().__init__('joint_velocity_plotter')
        
        # Parameters from your config
        self.wheel_radius = 0.102  # meters
        self.wheel_separation = 0.609  # meters
        self.wheel_separation_multiplier = 1.6
        self.effective_wheel_separation = self.wheel_separation * self.wheel_separation_multiplier
        
        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Data storage (keep last 200 points for better visualization)
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        
        # Actual velocities
        self.actual_fl = deque(maxlen=self.max_points)
        self.actual_rl = deque(maxlen=self.max_points)
        self.actual_fr = deque(maxlen=self.max_points)
        self.actual_rr = deque(maxlen=self.max_points)
        
        # Commanded velocities (calculated from cmd_vel)
        self.cmd_fl = deque(maxlen=self.max_points)
        self.cmd_rl = deque(maxlen=self.max_points)
        self.cmd_fr = deque(maxlen=self.max_points)
        self.cmd_rr = deque(maxlen=self.max_points)
        
        # Latest cmd_vel
        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        self.cmd_received = False
        
        # Start time
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Joint Velocity Plotter initialized')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Effective wheel separation: {self.effective_wheel_separation} m')
        self.get_logger().info('Waiting for data...')
        
    def cmd_vel_callback(self, msg):
        """Process stamped cmd_vel"""
        self.latest_linear_x = msg.twist.linear.x
        self.latest_angular_z = msg.twist.angular.z
        self.cmd_received = True
        
    def joint_state_callback(self, msg):
        """Process joint states and calculate commanded velocities"""
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(current_time)
        
        # Extract actual velocities from joint_states
        joint_names = msg.name
        velocities = msg.velocity
        
        # Create a mapping
        vel_map = dict(zip(joint_names, velocities))
        
        # Get actual velocities (rad/s)
        actual_fl_vel = vel_map.get('front_left_wheel_joint', 0.0)
        actual_rl_vel = vel_map.get('rear_left_wheel_joint', 0.0)
        actual_fr_vel = vel_map.get('front_right_wheel_joint', 0.0)
        actual_rr_vel = vel_map.get('rear_right_wheel_joint', 0.0)
        
        self.actual_fl.append(actual_fl_vel)
        self.actual_rl.append(actual_rl_vel)
        self.actual_fr.append(actual_fr_vel)
        self.actual_rr.append(actual_rr_vel)
        
        # Calculate commanded wheel velocities from cmd_vel using differential drive kinematics
        # v_left = v_x - ω * L/2
        # v_right = v_x + ω * L/2
        # wheel_angular_vel = linear_vel / wheel_radius
        
        v_left_linear = self.latest_linear_x - (self.latest_angular_z * self.effective_wheel_separation / 2.0)
        v_right_linear = self.latest_linear_x + (self.latest_angular_z * self.effective_wheel_separation / 2.0)
        
        # Convert to wheel angular velocity (rad/s)
        cmd_left_vel = v_left_linear / self.wheel_radius
        cmd_right_vel = v_right_linear / self.wheel_radius
        
        # Both wheels on same side get same command
        self.cmd_fl.append(cmd_left_vel)
        self.cmd_rl.append(cmd_left_vel)
        self.cmd_fr.append(cmd_right_vel)
        self.cmd_rr.append(cmd_right_vel)

def main():
    rclpy.init()
    node = JointVelocityPlotter()
    
    # Setup matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Joint Velocities: Actual vs Commanded (rad/s)', fontsize=16, fontweight='bold')
    
    # Initialize lines for each subplot
    lines = {}
    
    joint_configs = [
        (ax1, 'Front Left', 'front_left'),
        (ax2, 'Rear Left', 'rear_left'),
        (ax3, 'Front Right', 'front_right'),
        (ax4, 'Rear Right', 'rear_right')
    ]
    
    for ax, joint_name, key in joint_configs:
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Velocity (rad/s)', fontsize=11)
        ax.set_title(joint_name, fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        line_actual, = ax.plot([], [], 'b-', label='Actual', linewidth=2)
        line_cmd, = ax.plot([], [], 'r--', label='Commanded', linewidth=2.5, alpha=0.8)
        ax.legend(loc='upper right', fontsize=10)
        
        lines[key] = (line_actual, line_cmd, ax)
    
    # Add text box for statistics
    stats_text = fig.text(0.02, 0.02, '', fontsize=9, family='monospace',
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # Add current command display
    cmd_text = fig.text(0.98, 0.02, '', fontsize=10, family='monospace',
                        bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5),
                        ha='right')
    
    def update_plot(frame):
        """Animation update function"""
        rclpy.spin_once(node, timeout_sec=0.001)
        
        if len(node.times) < 2:
            return []
        
        times = list(node.times)
        
        # Update each subplot
        datasets = [
            ('front_left', node.actual_fl, node.cmd_fl),
            ('rear_left', node.actual_rl, node.cmd_rl),
            ('front_right', node.actual_fr, node.cmd_fr),
            ('rear_right', node.actual_rr, node.cmd_rr),
        ]
        
        all_lines = []
        errors = []
        
        for key, actual_data, cmd_data in datasets:
            line_actual, line_cmd, ax = lines[key]
            
            actual_list = list(actual_data)
            cmd_list = list(cmd_data)
            
            line_actual.set_data(times, actual_list)
            line_cmd.set_data(times, cmd_list)
            
            # Calculate error (last 50 points)
            if len(actual_list) > 10 and len(cmd_list) > 10:
                recent_actual = np.array(actual_list[-50:])
                recent_cmd = np.array(cmd_list[-50:])
                error = recent_actual - recent_cmd
                rmse = np.sqrt(np.mean(error**2))
                mae = np.mean(np.abs(error))
                errors.append((key.replace('_', ' ').title(), rmse, mae))
            
            # Auto-scale axes
            ax.relim()
            ax.autoscale_view()
            
            all_lines.extend([line_actual, line_cmd])
        
        # Update statistics
        if errors:
            stats_str = "Tracking Error (last 50 samples):\n"
            stats_str += "Joint          RMSE    MAE\n"
            stats_str += "-" * 32 + "\n"
            for name, rmse, mae in errors:
                stats_str += f"{name:12s} {rmse:6.4f} {mae:6.4f}\n"
            stats_text.set_text(stats_str)
        
        # Update current command display
        if node.cmd_received:
            cmd_str = "Current Command:\n"
            cmd_str += f"Linear:  {node.latest_linear_x:+.3f} m/s\n"
            cmd_str += f"Angular: {node.latest_angular_z:+.3f} rad/s"
            cmd_text.set_text(cmd_str)
        
        return all_lines
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.tight_layout(rect=(0, 0.05, 1, 0.96))
        node.get_logger().info('Plot window opened.')
        node.get_logger().info('Send cmd_vel commands to see velocities.')
        node.get_logger().info('Example: Use your gamepad or run:')
        node.get_logger().info('  ros2 topic pub /diff_drive_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'\'}, twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"')
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()