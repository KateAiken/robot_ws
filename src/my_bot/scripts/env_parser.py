#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import re

class EnvParserNode(Node):
    def __init__(self):
        super().__init__('env_parser')

        self.sub = self.create_subscription(
            String, '/arduino/raw', self.callback, 10
        )

        self.pub_tvoc = self.create_publisher(Float32, '/env/tvoc_ppb', 10)
        self.pub_eco2 = self.create_publisher(Float32, '/env/eco2_ppm', 10)
        self.pub_temp = self.create_publisher(Float32, '/env/temp_c', 10)
        self.pub_rh   = self.create_publisher(Float32, '/env/humidity', 10)
        self.pub_co2  = self.create_publisher(Float32, '/env/co2_ppm', 10)

        self.re_tvoc = re.compile(r'TVOC\s+(\d+)\s+ppb')
        self.re_eco2 = re.compile(r'eCO2\s+(\d+)\s+ppm')
        self.re_temp = re.compile(r'Temp:\s*([0-9.]+)')
        self.re_rh   = re.compile(r'RH:\s*([0-9.]+)')
        self.re_co2  = re.compile(r'CO2:\s*([0-9.]+)')

        self.get_logger().info("Environmental parser started")

    def callback(self, msg):
        line = msg.data

        if m := self.re_tvoc.search(line):
            self.pub_tvoc.publish(Float32(data=float(m.group(1))))

        if m := self.re_eco2.search(line):
            self.pub_eco2.publish(Float32(data=float(m.group(1))))

        if m := self.re_temp.search(line):
            self.pub_temp.publish(Float32(data=float(m.group(1))))

        if m := self.re_rh.search(line):
            self.pub_rh.publish(Float32(data=float(m.group(1))))

        if m := self.re_co2.search(line):
            self.pub_co2.publish(Float32(data=float(m.group(1))))

def main(args=None):
    rclpy.init(args=args)
    node = EnvParserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
