#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import tf2_ros
import math

class WheelOdom(Node):
    def __init__(self):
        super().__init__("wheel_odom")

        # Robot parameters
        self.wheel_radius = 0.0625
        self.wheel_base   = 0.08
        self.ticks_per_rev = 3000

        # Internal state
        self.last_left_ticks  = None
        self.last_right_ticks = None
        self.delta_left = None
        self.delta_right = None

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(Int32, "encoder1_ticks", self.left_cb, 10)
        self.create_subscription(Int32, "encoder2_ticks", self.right_cb, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Wheel odometry node started.")

    def left_cb(self, msg):
        ticks = msg.data
        if self.last_left_ticks is None:
            self.last_left_ticks = ticks
            return
        self.delta_left = ticks - self.last_left_ticks
        self.last_left_ticks = ticks
        self.try_compute_odom()

    def right_cb(self, msg):
        ticks = msg.data
        if self.last_right_ticks is None:
            self.last_right_ticks = ticks
            return
        self.delta_right = ticks - self.last_right_ticks
        self.last_right_ticks = ticks
        self.try_compute_odom()

    def try_compute_odom(self):
        # Need both deltas
        if self.delta_left is None or self.delta_right is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return
        self.last_time = now

        # Convert ticks to distance
        meters_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left  = self.delta_left  * meters_per_tick
        d_right = self.delta_right * meters_per_tick

        # Reset deltas
        self.delta_left = None
        self.delta_right = None

        # Differential drive kinematics
        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / self.wheel_base

        # Integrate pose
        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # Velocities
        v = d_center / dt
        w = d_theta / dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
