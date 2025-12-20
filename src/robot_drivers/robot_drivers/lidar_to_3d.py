import rclpy
from rclpy.node import Node

import numpy as np
import math

from sensor_msgs.msg import LaserScan, Imu, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

class ScanTo3DAccum(Node):
    def __init__(self):
        super().__init__("scan_to_3d_accum")
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.cb_scan, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, "/imu/data", self.cb_imu, 10
        )

        self.pub_cloud = self.create_publisher(
            PointCloud2, "/scan_cloud_accum", 10
        )

        self.q = [0, 0, 0, 1]

        self.points = []

        self.get_logger().info("3D SCAN accumulator started")

    def cb_imu(self, msg: Imu):
        self.q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]

    def cb_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        zs = np.zeros_like(xs)

        pts = np.vstack((xs, ys, zs)).T

        pts_rot = self.apply_quaternion(self.q, pts)

        self.points.append(pts_rot)

        all_pts = np.vstack(self.points)

        header = Header(frame_id="map")
        cloud = pc2.create_cloud_xyz32(header, all_pts.tolist())
        self.pub_cloud.publish(cloud)

    def apply_quaternion(self, q, pts):
        x, y, z, w = q

        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,       2*x*z + 2*y*w],
            [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,   2*y*z - 2*x*w],
            [2*x*z - 2*y*w,         2*y*z + 2*x*w,       1 - 2*x*x - 2*y*y],
        ])

        return pts @ R.T


def main(args=None):
    rclpy.init(args=args)
    node = ScanTo3DAccum()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
