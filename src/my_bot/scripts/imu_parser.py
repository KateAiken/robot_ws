#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math
import re

class ImuParserNode(Node):
    def __init__(self):
        super().__init__('imu_parser')

        self.sub = self.create_subscription(
            String,
            '/arduino/raw',
            self.callback,
            10
        )

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        # regex to match IMU lines
        self.imu_regex = re.compile(
            r'([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)\s*g.*?([-0-9.]+)\s+([-0-9.]+)\s+([-0-9.]+)'
        )

        self.get_logger().info("IMU parser started")

    def callback(self, msg):
        line = msg.data.strip()

        match = self.imu_regex.search(line)
        if not match:
            return  # ignore non‑IMU lines

        ax_g, ay_g, az_g, gx_deg, gy_deg, gz_deg = map(float, match.groups())

        # Convert units
        ax = ax_g * 9.80665
        ay = ay_g * 9.80665
        az = az_g * 9.80665

        gx = math.radians(gx_deg)
        gy = math.radians(gy_deg)
        gz = math.radians(gz_deg)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        self.pub.publish(imu)

def main(args=None):
    rclpy.init(args=args)
    node = ImuParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
