#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

class RobotVelocityPlotter(Node):
    def __init__(self):
        super().__init__('robot_velocity_plotter')
        
        # Subscriptions
        # Odometry provides the actual robot velocity
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_base_controller/odom',
            self.odom_callback,
            10
        )
        
        # cmd_vel provides the commanded velocity
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Data storage (keep last 200 points)
        self.max_points = 200
        self.times = deque(maxlen=self.max_points)
        
        # Actual velocities from odometry
        self.actual_linear_x = deque(maxlen=self.max_points)
        self.actual_angular_z = deque(maxlen=self.max_points)
        
        # Commanded velocities from cmd_vel
        self.cmd_linear_x = deque(maxlen=self.max_points)
        self.cmd_angular_z = deque(maxlen=self.max_points)
        
        # Latest values
        self.latest_cmd_linear = 0.0
        self.latest_cmd_angular = 0.0
        self.cmd_received = False
        self.odom_received = False
        
        # Start time
        self.start_time = self.get_clock().now()
        
        self.get_logger().info('Robot Velocity Plotter initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /diff_drive_base_controller/odom (actual velocity)')
        self.get_logger().info('  - /diff_drive_base_controller/cmd_vel (commanded velocity)')
        self.get_logger().info('Waiting for data...')
        
    def cmd_vel_callback(self, msg):
        """Process commanded velocity"""
        self.latest_cmd_linear = msg.twist.linear.x
        self.latest_cmd_angular = msg.twist.angular.z
        self.cmd_received = True
        
    def odom_callback(self, msg):
        """Process odometry to get actual robot velocity"""
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(current_time)
        
        # Extract actual velocities from odometry
        actual_linear = msg.twist.twist.linear.x
        actual_angular = msg.twist.twist.angular.z
        
        self.actual_linear_x.append(actual_linear)
        self.actual_angular_z.append(actual_angular)
        
        # Store commanded velocities at same timestamp
        self.cmd_linear_x.append(self.latest_cmd_linear)
        self.cmd_angular_z.append(self.latest_cmd_angular)
        
        self.odom_received = True

def main():
    rclpy.init()
    node = RobotVelocityPlotter()
    
    # Setup matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, (ax_linear, ax_angular) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle('Robot Base Velocities: Actual vs Commanded', fontsize=16, fontweight='bold')
    
    # Linear velocity subplot
    ax_linear.set_xlabel('Time (s)', fontsize=11)
    ax_linear.set_ylabel('Linear Velocity (m/s)', fontsize=11)
    ax_linear.set_title('Linear Velocity (X-axis)', fontsize=12, fontweight='bold')
    ax_linear.grid(True, alpha=0.3)
    
    line_linear_actual, = ax_linear.plot([], [], 'b-', label='Actual', linewidth=2)
    line_linear_cmd, = ax_linear.plot([], [], 'r--', label='Commanded', linewidth=2.5, alpha=0.8)
    ax_linear.legend(loc='upper right', fontsize=10)
    ax_linear.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    
    # Angular velocity subplot
    ax_angular.set_xlabel('Time (s)', fontsize=11)
    ax_angular.set_ylabel('Angular Velocity (rad/s)', fontsize=11)
    ax_angular.set_title('Angular Velocity (Z-axis)', fontsize=12, fontweight='bold')
    ax_angular.grid(True, alpha=0.3)
    
    line_angular_actual, = ax_angular.plot([], [], 'b-', label='Actual', linewidth=2)
    line_angular_cmd, = ax_angular.plot([], [], 'r--', label='Commanded', linewidth=2.5, alpha=0.8)
    ax_angular.legend(loc='upper right', fontsize=10)
    ax_angular.axhline(y=0, color='k', linestyle='-', linewidth=0.5, alpha=0.3)
    
    # Add text box for statistics
    stats_text = fig.text(0.02, 0.02, '', fontsize=10, family='monospace',
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
        
        # Get data
        actual_linear = list(node.actual_linear_x)
        cmd_linear = list(node.cmd_linear_x)
        actual_angular = list(node.actual_angular_z)
        cmd_angular = list(node.cmd_angular_z)
        
        # Update linear velocity plot
        line_linear_actual.set_data(times, actual_linear)
        line_linear_cmd.set_data(times, cmd_linear)
        ax_linear.relim()
        ax_linear.autoscale_view()
        
        # Update angular velocity plot
        line_angular_actual.set_data(times, actual_angular)
        line_angular_cmd.set_data(times, cmd_angular)
        ax_angular.relim()
        ax_angular.autoscale_view()
        
        # Calculate statistics (last 50 points)
        if len(actual_linear) > 10:
            # Linear velocity error
            recent_actual_lin = np.array(actual_linear[-50:])
            recent_cmd_lin = np.array(cmd_linear[-50:])
            error_lin = recent_actual_lin - recent_cmd_lin
            rmse_lin = np.sqrt(np.mean(error_lin**2))
            mae_lin = np.mean(np.abs(error_lin))
            
            # Angular velocity error
            recent_actual_ang = np.array(actual_angular[-50:])
            recent_cmd_ang = np.array(cmd_angular[-50:])
            error_ang = recent_actual_ang - recent_cmd_ang
            rmse_ang = np.sqrt(np.mean(error_ang**2))
            mae_ang = np.mean(np.abs(error_ang))
            
            # Current values
            curr_actual_lin = actual_linear[-1]
            curr_cmd_lin = cmd_linear[-1]
            curr_actual_ang = actual_angular[-1]
            curr_cmd_ang = cmd_angular[-1]
            
            # Update statistics display
            stats_str = "Tracking Error (last 50 samples):\n"
            stats_str += "=" * 40 + "\n"
            stats_str += f"Linear Velocity:\n"
            stats_str += f"  RMSE: {rmse_lin:.4f} m/s\n"
            stats_str += f"  MAE:  {mae_lin:.4f} m/s\n"
            stats_str += f"  Current: {curr_actual_lin:+.3f} m/s (cmd: {curr_cmd_lin:+.3f})\n"
            stats_str += "\n"
            stats_str += f"Angular Velocity:\n"
            stats_str += f"  RMSE: {rmse_ang:.4f} rad/s\n"
            stats_str += f"  MAE:  {mae_ang:.4f} rad/s\n"
            stats_str += f"  Current: {curr_actual_ang:+.3f} rad/s (cmd: {curr_cmd_ang:+.3f})"
            stats_text.set_text(stats_str)
        
        # Update current command display
        if node.cmd_received and node.odom_received:
            cmd_str = "Current Command:\n"
            cmd_str += f"Linear:  {node.latest_cmd_linear:+.3f} m/s\n"
            cmd_str += f"Angular: {node.latest_cmd_angular:+.3f} rad/s\n"
            cmd_str += "\n"
            if len(actual_linear) > 0:
                cmd_str += "Current Actual:\n"
                cmd_str += f"Linear:  {actual_linear[-1]:+.3f} m/s\n"
                cmd_str += f"Angular: {actual_angular[-1]:+.3f} rad/s"
            cmd_text.set_text(cmd_str)
        
        return [line_linear_actual, line_linear_cmd, line_angular_actual, line_angular_cmd]
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.tight_layout(rect=(0, 0.06, 1, 0.96))
        node.get_logger().info('Plot window opened.')
        node.get_logger().info('Send cmd_vel commands to see robot velocities.')
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()