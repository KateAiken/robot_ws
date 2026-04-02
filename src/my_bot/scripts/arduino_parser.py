#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import re
import math
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class ArduinoParser(Node):
    def __init__(self):
        super().__init__('arduino_parser')

        # Serial port parameters
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baud = self.declare_parameter('baud', 115200).value
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Raw serial publisher (keep for debugging)
        self.pub_raw = self.create_publisher(String, '/arduino/raw', 10)

        # Environmental publishers
        self.pub_temp = self.create_publisher(Float32, '/env/temp_c',    10)
        self.pub_rh   = self.create_publisher(Float32, '/env/humidity',  10)
        self.pub_co2  = self.create_publisher(Float32, '/env/co2_ppm',   10)
        self.pub_tvoc = self.create_publisher(Float32, '/env/tvoc_ppb',  10)
        self.pub_eco2 = self.create_publisher(Float32, '/env/eco2_ppm',  10)

        # Odometry publisher + TF
        self.odom_pub       = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Odometry state
        self.wheel_base      = 0.22      # meters
        self.meters_per_tick = (2 * math.pi * 0.0625) / 3000  # 0.000982
        self.prev_left       = None
        self.prev_right      = None
        self.x               = 0.0
        self.y               = 0.0
        self.yaw             = 0.0

        # Regexes
        self.re_encoder = re.compile(
            r'LEFT ticks:\s*(-?\d+)\s+RIGHT ticks:\s*(-?\d+)'
        )
        self.re_temp = re.compile(r'Temp:\s*([0-9.]+)')
        self.re_rh   = re.compile(r'RH:\s*([0-9.]+)')
        self.re_co2  = re.compile(r'CO2:\s*([0-9.]+)')
        self.re_tvoc = re.compile(r'TVOC\s+(\d+)\s+ppb')
        self.re_eco2 = re.compile(r'eCO2\s+(\d+)\s+ppm')

        # Read serial at 100Hz
        self.timer = self.create_timer(0.01, self.read_serial)
        self.get_logger().info(f'Arduino parser started on {port} at {baud}')

    def read_serial(self):
        if self.ser.in_waiting == 0:
            return

        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return

        # Publish raw for debugging
        self.pub_raw.publish(String(data=line))

        # Try encoder line
        m = self.re_encoder.search(line)
        if m:
            left_ticks  = int(m.group(1))
            right_ticks = int(m.group(2))
            self.update_odom(left_ticks, right_ticks)
            return

        # Try environmental lines
        if m := self.re_temp.search(line):
            self.pub_temp.publish(Float32(data=float(m.group(1))))
        if m := self.re_rh.search(line):
            self.pub_rh.publish(Float32(data=float(m.group(1))))
        if m := self.re_co2.search(line):
            self.pub_co2.publish(Float32(data=float(m.group(1))))
        if m := self.re_tvoc.search(line):
            self.pub_tvoc.publish(Float32(data=float(m.group(1))))
        if m := self.re_eco2.search(line):
            self.pub_eco2.publish(Float32(data=float(m.group(1))))

    def update_odom(self, left_ticks, right_ticks):
        now = self.get_clock().now()

        # First message — just store and return
        if self.prev_left is None:
            self.prev_left  = left_ticks
            self.prev_right = right_ticks
            return

        # delta_left  = left_ticks  - self.prev_left
        # delta_right = right_ticks - self.prev_right
        delta_left  = left_ticks  
        delta_right = right_ticks 
        self.prev_left  = left_ticks
        self.prev_right = right_ticks

        d_left  = delta_left  * self.meters_per_tick
        d_right = delta_right * self.meters_per_tick

        d      = (d_left + d_right) / 2.0
        dtheta = (d_right - d_left) / self.wheel_base

        self.x   += d * math.cos(self.yaw + dtheta / 2.0)
        self.y   += d * math.sin(self.yaw + dtheta / 2.0)
        self.yaw += dtheta
        self.yaw  = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        self.get_logger().debug(
            f'L={left_ticks} R={right_ticks} | '
            f'x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}deg'
        )

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # TF odom -> base_link
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()