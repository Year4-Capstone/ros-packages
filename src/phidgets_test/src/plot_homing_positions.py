#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from phidgets_test.action import HomingSequence
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

class HomingPositionPlotter(Node):
    def __init__(self):
        super().__init__('homing_position_plotter')
        
        # Action client to monitor homing sequence
        self._action_client = ActionClient(
            self,
            HomingSequence,
            'homing_sequence'
        )
        
        # Data storage (keep last 500 points for better visualization)
        self.max_points = 500
        self.times = deque(maxlen=self.max_points)
        
        # Position data for each leg
        self.leg_positions = [
            deque(maxlen=self.max_points),  # Leg 0
            deque(maxlen=self.max_points),  # Leg 1
            deque(maxlen=self.max_points),  # Leg 2
            deque(maxlen=self.max_points),  # Leg 3
        ]
        
        # Status tracking
        self.leg_status = [0, 0, 0, 0]  # 0=not started, 1=in progress, 2=completed, 3=failed
        self.homing_active = False
        self.homing_complete = False
        
        # Start time
        self.start_time = None
        
        self.get_logger().info('Homing Position Plotter initialized')
        self.get_logger().info('Waiting for homing action server...')
        
    def send_homing_goal(self, homing_speed=0.1):
        """Send a homing goal to start the sequence"""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
        
        goal_msg = HomingSequence.Goal()
        goal_msg.homing_speed = homing_speed
        
        self.get_logger().info(f'Sending homing goal with speed: {homing_speed}')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.start_time = self.get_clock().now()
        self.homing_active = True
        
        return True
    
    def goal_response_callback(self, future):
        """Called when goal is accepted or rejected"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            self.homing_active = False
            return
        
        self.get_logger().info('Goal accepted, homing sequence started')
        
        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Process feedback from homing action"""
        feedback = feedback_msg.feedback
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.times.append(current_time)
        
        # Update positions and status for each leg
        for i in range(4):
            self.leg_positions[i].append(feedback.positions[i])
            self.leg_status[i] = feedback.status[i]
    
    def get_result_callback(self, future):
        """Called when homing sequence completes"""
        result = future.result().result
        
        self.get_logger().info('Homing sequence completed!')
        for i in range(4):
            status = 'Success' if result.success[i] else 'Failed'
            self.get_logger().info(f'Leg {i}: {status} - Final position: {result.positions[i]:.2f}°')
            
            # UPDATE LEG STATUS BASED ON RESULT
            if result.success[i]:
                self.leg_status[i] = 2  # Completed
            else:
                self.leg_status[i] = 3  # Failed
        
        self.homing_active = False
        self.homing_complete = True

def main():
    rclpy.init()
    node = HomingPositionPlotter()
    
    # Setup matplotlib
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Actuation Motor Homing Positions', fontsize=16, fontweight='bold')
    
    # Flatten axes for easier indexing
    axes = axes.flatten()
    
    # Initialize lines for each leg
    lines = []
    leg_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
    colors = ['blue', 'red', 'green', 'orange']
    
    for i, (ax, name, color) in enumerate(zip(axes, leg_names, colors)):
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Position (degrees)', fontsize=11)
        ax.set_title(f'Leg {i}: {name}', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', linewidth=1, alpha=0.5, label='Home Position')
        
        line, = ax.plot([], [], color=color, linewidth=2, label='Position')
        ax.legend(loc='lower left', fontsize=9)
        lines.append((line, ax))
    
    # Status text boxes
    status_texts = []
    positions = [(0.02, 0.95), (0.52, 0.95), (0.02, 0.47), (0.52, 0.47)]
    for i, pos in enumerate(positions):
        text = fig.text(pos[0], pos[1], '', fontsize=9, family='monospace',
                       bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7),
                       verticalalignment='top')
        status_texts.append(text)
    
    # Overall status text
    overall_status_text = fig.text(0.5, 0.02, '', fontsize=11, family='monospace',
                                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7),
                                   ha='center')
    
    # Flag to track if goal has been sent
    goal_sent = False
    

    def update_plot(frame):
        """Animation update function"""
        nonlocal goal_sent
        
        rclpy.spin_once(node, timeout_sec=0.001)
        
        # Send goal on first update
        if not goal_sent:
            node.send_homing_goal(homing_speed=0.1)
            goal_sent = True
        
        if len(node.times) < 2:
            return []
        
        times = list(node.times)
        all_lines = []
        
        # Update each subplot
        for i, (line, ax) in enumerate(lines):
            positions = list(node.leg_positions[i])
            line.set_data(times, positions)
            
            # Auto-scale axes
            ax.relim()
            ax.autoscale_view()
            
            # Update status text for this leg
            status_map = {
                0: "Not Started",
                1: "In Progress",
                2: "Completed",
                3: "Failed"
            }
            current_pos = positions[-1] if positions else 0.0
            status_str = f"Status: {status_map.get(node.leg_status[i], 'Unknown')}\n"
            status_str += f"Position: {current_pos:.2f}°"
            status_texts[i].set_text(status_str)
            
            # Color code status text
            bbox = status_texts[i].get_bbox_patch()
            if bbox:
                if node.leg_status[i] == 2:  # Completed
                    bbox.set_facecolor('lightgreen')
                elif node.leg_status[i] == 3:  # Failed
                    bbox.set_facecolor('lightcoral')
                elif node.leg_status[i] == 1:  # In progress
                    bbox.set_facecolor('lightyellow')
                else:  # Not started
                    bbox.set_facecolor('lightgray')
            
            all_lines.append(line)
        
        # Update overall status (MOVED OUTSIDE THE FOR LOOP)
        bbox = overall_status_text.get_bbox_patch()
        if bbox:
            if node.homing_complete:
                overall_status_text.set_text('Homing Sequence: COMPLETE')
                bbox.set_facecolor('lightgreen')
            elif node.homing_active:
                overall_status_text.set_text('Homing Sequence: IN PROGRESS')
                bbox.set_facecolor('lightyellow')
            else:
                overall_status_text.set_text('Homing Sequence: WAITING TO START')
                bbox.set_facecolor('lightgray')
        
        return all_lines
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.tight_layout(rect=(0, 0.05, 1, 0.98))
        node.get_logger().info('Plot window opened.')
        node.get_logger().info('Homing sequence will start automatically...')
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()