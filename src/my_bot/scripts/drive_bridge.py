#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class DriveBridge(Node):
    def __init__(self):
        super().__init__('drive_bridge')

        # Parameters
        self.declare_parameter('wheel_base', 0.30)   # meters
        self.declare_parameter('max_speed', 0.5)     # m/s
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Open serial port to Arduino
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # Subscribe to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("Drive bridge ready. Listening to /cmd_vel")

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive kinematics
        v_left  = v - w * (self.wheel_base / 2.0)
        v_right = v + w * (self.wheel_base / 2.0)

        # Convert m/s → PWM
        left_pwm  = int((v_left  / self.max_speed) * self.max_pwm)
        right_pwm = int((v_right / self.max_speed) * self.max_pwm)

        # Clamp
        left_pwm  = max(min(left_pwm,  self.max_pwm), -self.max_pwm)
        right_pwm = max(min(right_pwm, self.max_pwm), -self.max_pwm)

        # Send to Arduino
        command = f"L:{left_pwm} R:{right_pwm}\n"
        self.ser.write(command.encode())

def main():
    rclpy.init()
    node = DriveBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
