#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial')

        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baud = self.declare_parameter('baud', 115200).value

        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.pub = self.create_publisher(String, 'arduino/raw', 10)
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                msg = String()
                msg.data = line
                self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()