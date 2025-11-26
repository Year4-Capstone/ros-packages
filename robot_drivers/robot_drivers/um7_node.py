import math
import glob
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from rsl_comm_py import rsl_autodetect, UM7Serial
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import threading

# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSReliabilityPolicy
# from rclpy.qos import QoSHistoryPolicy

from rclpy.qos import QoSProfile, ReliabilityPolicy


class UM7Node(Node):
    def __init__(self):
        super().__init__("um7_driver")

        self.get_logger().info("Starting UM7 IMU driver using rsl_comm_py...")

        device_json = self.autodetect_um7()
        self.um7 = self.connect_um7(device_json)

        # Publishers
        # qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos)
        self.mag_pub = self.create_publisher(MagneticField, "imu/magnetic_field", 10)

        # Looping at 100 Hz
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        # Gyro: + or - 2000 deg/s
        self.gyro_scale = math.radians(2000.0) / 32768.0 # rad/s per LSB
        # Accel: + or - 8 g
        self.accel_scale = (8.0 * 9.80665) / 32768.0 # m/s^2 per LSB
        # Mag: + or - 12 gauss (1 gauss = 1e-4 Tesla)
        self.mag_scale = (12.0 * 1e-4) / 32768.0 # Tesla per LSB
        self.tf_broadcaster = TransformBroadcaster(self)

    def read_loop(self):
        stream = self.um7.recv_euler_broadcast(num_packets=-1)
        for pkt in stream:
            self.publish_euler(pkt)

    def autodetect_um7(self):
        self.get_logger().info("Running autodetect()...")
        try:
            rsl_autodetect()
        except Exception as e:
            self.get_logger().warn(f"Autodetect exception: {e}")

        files = glob.glob("rsl_*.json")
        if not files:
            self.get_logger().warn("No JSON found → using /dev/ttyUSB0")
            return None

        files.sort()
        device_json = files[-1]
        self.get_logger().info(f"Using device file: {device_json}")
        return device_json

    def connect_um7(self, device_json):
        if device_json:
            self.get_logger().info(f"Connecting UM7 using {device_json}")
            return UM7Serial(device=device_json)
        self.get_logger().info("Connecting UM7 on /dev/ttyUSB0")
        return UM7Serial(port_name="/dev/ttyUSB0")

    def publish_euler(self, pkt):
        now = self.get_clock().now()

        roll  = math.radians(pkt.roll)
        pitch = math.radians(pkt.pitch)
        yaw   = math.radians(pkt.yaw)

        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "imu_link"
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        self.imu_pub.publish(msg)

        # TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "imu_link"
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        roll, pitch, yaw in radians → (x, y, z, w)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    node = UM7Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
