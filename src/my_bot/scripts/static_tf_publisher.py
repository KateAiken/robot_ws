#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from builtin_interfaces.msg import Time

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)

        # Use stamp = 0 for static transforms - means "always valid"
        t1 = TransformStamped()
        t1.header.stamp = Time(sec=0, nanosec=0)
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'laser'
        t1.transform.translation.x = 0.05
        t1.transform.translation.y = 0.05
        t1.transform.translation.z = 0.07
        t1.transform.rotation.w = 1.0

        self.broadcaster.sendTransform([t1])
        self.get_logger().info('Published static transforms')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()