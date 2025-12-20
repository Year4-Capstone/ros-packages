#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray

class GTRobotExtractor(Node):
    def __init__(self):
        super().__init__('gt_robot_extractor')

        self.sub = self.create_subscription(
            PoseArray,
            '/world/default/dynamic_pose/info',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PoseStamped,
            '/ground_truth_pose',
            10
        )

    def callback(self, msg):
        if len(msg.poses) == 0:
            return

        robot_pose = msg.poses[0]

        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "world"
        out.pose = robot_pose

        self.pub.publish(out)

def main():
    rclpy.init()
    node = GTRobotExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
