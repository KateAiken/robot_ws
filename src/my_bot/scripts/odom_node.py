#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
import math
import RPi.GPIO as GPIO


class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.ticks = 0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self.callback)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self.callback)

    def callback(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        if a == b:
            self.ticks += 1
        else:
            self.ticks -= 1


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        self.left_encoder = Encoder(17, 27)
        self.right_encoder = Encoder(22, 23)

        self.prev_left = 0
        self.prev_right = 0

        self.wheel_base = 0.8
        self.meters_per_tick = 0.000982

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)



        # 20 Hz update rate
        self.timer = self.create_timer(0.05, self.update)


    def update(self):
        now = self.get_clock().now()


        left_ticks = self.left_encoder.ticks
        right_ticks = self.right_encoder.ticks

        delta_left = left_ticks - self.prev_left
        delta_right = right_ticks - self.prev_right

        self.prev_left = left_ticks
        self.prev_right = right_ticks

        d_left = delta_left * self.meters_per_tick
        d_right = delta_right * self.meters_per_tick

        d = (d_left + d_right) / 2.0
        dtheta = (d_right - d_left) / self.wheel_base

        self.x += d * math.cos(self.yaw + dtheta / 2.0)
        self.y += d * math.sin(self.yaw + dtheta / 2.0)
        self.yaw += dtheta
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Publish odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
