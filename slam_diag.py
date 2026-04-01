#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener

class SlamDiag(Node):
    def __init__(self):
        super().__init__('slam_diag')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.scan_stamp = None
        self.odom_stamp = None
        self.map_received = False

        self.timer = self.create_timer(1.0, self.check_status)

    def scan_cb(self, msg):
        self.scan_stamp = msg.header.stamp

    def odom_cb(self, msg):
        self.odom_stamp = msg.header.stamp

    def map_cb(self, msg):
        self.map_received = True

    def check_status(self):
        print("\n=== SLAM DIAGNOSTICS ===")

        # 1. Check scan
        if self.scan_stamp:
            print(f"Scan timestamp: {self.scan_stamp.sec}.{self.scan_stamp.nanosec}")
        else:
            print("❌ No /scan messages received")

        # 2. Check odom
        if self.odom_stamp:
            print(f"Odom timestamp: {self.odom_stamp.sec}.{self.odom_stamp.nanosec}")
        else:
            print("❌ No /odom messages received")

        # 3. Check timestamp difference
        if self.scan_stamp and self.odom_stamp:
            scan_t = self.scan_stamp.sec + self.scan_stamp.nanosec * 1e-9
            odom_t = self.odom_stamp.sec + self.odom_stamp.nanosec * 1e-9
            diff = abs(scan_t - odom_t)
            print(f"Timestamp difference: {diff:.3f} sec")

            if diff > 0.2:
                print("❌ Timestamps too far apart — SLAM cannot compute pose")
            else:
                print("✔ Timestamps OK")

        # 4. Check TF transformability
        try:
            self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time())
            print("✔ TF: laser → base_link OK")
        except:
            print("❌ TF missing: laser → base_link")

        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            print("✔ TF: odom → base_link OK")
        except:
            print("❌ TF missing: odom → base_link")

        # 5. Check map
        if self.map_received:
            print("✔ Map messages received")
        else:
            print("❌ No map messages received")

def main():
    rclpy.init()
    node = SlamDiag()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
