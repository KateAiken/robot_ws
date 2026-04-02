#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import pigpio

# Encoder 1 pins (BCM)
ENC1_A = 23
ENC1_B = 24

# Encoder 2 pins (BCM)
ENC2_A = 17
ENC2_B = 27

# Tick counters
encoder1_ticks = 0
encoder2_ticks = 0

class DualEncoderNode(Node):
    def __init__(self):
        super().__init__('dual_encoder_node')

        # ROS publishers
        self.pub_enc1 = self.create_publisher(Int32, 'encoder1_ticks', 10)
        self.pub_enc2 = self.create_publisher(Int32, 'encoder2_ticks', 10)

        # Timer to publish periodically
        self.timer = self.create_timer(0.1, self.publish_ticks)
        self.enc1_last = 0
        self.enc2_last = 0

        # Connect to pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio daemon")
            exit(1)

        # Set up pins for encoder 1
        self.pi.set_mode(ENC1_A, pigpio.INPUT)
        self.pi.set_mode(ENC1_B, pigpio.INPUT)
        self.pi.set_pull_up_down(ENC1_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(ENC1_B, pigpio.PUD_UP)
        self.pi.callback(ENC1_A, pigpio.EITHER_EDGE, self.encoder1_callback)
        self.pi.callback(ENC1_B, pigpio.EITHER_EDGE, self.encoder1_callback)

        # Set up pins for encoder 2
        self.pi.set_mode(ENC2_A, pigpio.INPUT)
        self.pi.set_mode(ENC2_B, pigpio.INPUT)
        self.pi.set_pull_up_down(ENC2_A, pigpio.PUD_UP)
        self.pi.set_pull_up_down(ENC2_B, pigpio.PUD_UP)
        self.pi.callback(ENC2_A, pigpio.EITHER_EDGE, self.encoder2_callback)
        self.pi.callback(ENC2_B, pigpio.EITHER_EDGE, self.encoder2_callback)

        # State tracking for quadrature decoding
        self.enc1_last_state = (self.pi.read(ENC1_A), self.pi.read(ENC1_B))
        self.enc2_last_state = (self.pi.read(ENC2_A), self.pi.read(ENC2_B))

        self.get_logger().info("Dual encoder node initialized")

    # Quadrature decode for encoder 1
    def encoder1_callback(self, gpio, level, tick):
        global encoder1_ticks
        a = self.pi.read(ENC1_A)
        b = self.pi.read(ENC1_B)
        last_a, last_b = self.enc1_last_state

        # Determine direction
        if (last_a, last_b) == (0,0):
            if (a,b) == (0,1): encoder1_ticks +=1
            if (a,b) == (1,0): encoder1_ticks -=1
        elif (last_a, last_b) == (0,1):
            if (a,b) == (1,1): encoder1_ticks +=1
            if (a,b) == (0,0): encoder1_ticks -=1
        elif (last_a, last_b) == (1,1):
            if (a,b) == (1,0): encoder1_ticks +=1
            if (a,b) == (0,1): encoder1_ticks -=1
        elif (last_a, last_b) == (1,0):
            if (a,b) == (0,0): encoder1_ticks +=1
            if (a,b) == (1,1): encoder1_ticks -=1

        self.enc1_last_state = (a,b)

    # Quadrature decode for encoder 2
    def encoder2_callback(self, gpio, level, tick):
        global encoder2_ticks
        a = self.pi.read(ENC2_A)
        b = self.pi.read(ENC2_B)
        last_a, last_b = self.enc2_last_state

        if (last_a, last_b) == (0,0):
            if (a,b) == (0,1): encoder2_ticks +=1
            if (a,b) == (1,0): encoder2_ticks -=1
        elif (last_a, last_b) == (0,1):
            if (a,b) == (1,1): encoder2_ticks +=1
            if (a,b) == (0,0): encoder2_ticks -=1
        elif (last_a, last_b) == (1,1):
            if (a,b) == (1,0): encoder2_ticks +=1
            if (a,b) == (0,1): encoder2_ticks -=1
        elif (last_a, last_b) == (1,0):
            if (a,b) == (0,0): encoder2_ticks +=1
            if (a,b) == (1,1): encoder2_ticks -=1

        self.enc2_last_state = (a,b)

    # Publish ticks
    def publish_ticks(self):
        global encoder1_ticks, encoder2_ticks
        if encoder1_ticks != self.enc1_last:
            msg = Int32()
            msg.data = encoder1_ticks
            self.pub_enc1.publish(msg)
            self.get_logger().info(f"Encoder 1 ticks: {encoder1_ticks}")
            self.enc1_last = encoder1_ticks
        if encoder2_ticks != self.enc2_last:
            msg = Int32()
            msg.data = encoder2_ticks
            self.pub_enc2.publish(msg)
            self.get_logger().info(f"Encoder 2 ticks: {encoder2_ticks}")
            self.enc2_last = encoder2_ticks

def main(args=None):
    rclpy.init(args=args)
    node = DualEncoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pi.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()