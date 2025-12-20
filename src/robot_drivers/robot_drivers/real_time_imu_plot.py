#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import math
import threading
import time


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion into roll, pitch, yaw (radians)."""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, +1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class ImuPlotter(Node):
    def __init__(self):
        super().__init__("imu_plotter")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            Imu, "/imu/data", self.cb, qos
        )

        self.N = 500
        self.roll = deque(maxlen=self.N)
        self.pitch = deque(maxlen=self.N)
        self.yaw = deque(maxlen=self.N)

        self.ax = deque(maxlen=self.N)
        self.ay = deque(maxlen=self.N)
        self.az = deque(maxlen=self.N)

        self.gx = deque(maxlen=self.N)
        self.gy = deque(maxlen=self.N)
        self.gz = deque(maxlen=self.N)

        plot_thread = threading.Thread(target=self.plot_thread, daemon=True)
        plot_thread.start()

        self.get_logger().info("IMU Plotter started. Listening on /imu/data")

    def cb(self, msg):
        r, p, y = quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.roll.append(r)
        self.pitch.append(p)
        self.yaw.append(y)

        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)

        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)

    def plot_thread(self):
        plt.ion()
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))

        while True:
            if len(self.roll) > 5:
                axs[0].cla()
                axs[1].cla()
                axs[2].cla()

                axs[0].plot(self.roll, label="roll")
                axs[0].plot(self.pitch, label="pitch")
                axs[0].plot(self.yaw, label="yaw")
                axs[0].legend()
                axs[0].set_title("Orientation (rad)")

                axs[1].plot(self.ax, label="ax")
                axs[1].plot(self.ay, label="ay")
                axs[1].plot(self.az, label="az")
                axs[1].legend()
                axs[1].set_title("Acceleration (m/s^2)")

                axs[2].plot(self.gx, label="gx")
                axs[2].plot(self.gy, label="gy")
                axs[2].plot(self.gz, label="gz")
                axs[2].legend()
                axs[2].set_title("Angular Velocity (rad/s)")

                plt.tight_layout()
                plt.pause(0.001)

            time.sleep(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
