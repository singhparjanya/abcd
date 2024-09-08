#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class FakeBatteryPublisher(Node):
    def __init__(self):
        super().__init__('fake_battery_publisher')
        self.publisher_ = self.create_publisher(Float32, '/battery_level', 10)
        self.publish_battery_level()

    def publish_battery_level(self):
        while rclpy.ok():
            try:
                battery_level = float(input("Enter the current battery level (0-100): "))
                if 0 <= battery_level <= 100:
                    msg = Float32()
                    msg.data = battery_level
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publishing: {battery_level}%')
                else:
                    self.get_logger().warn("Please enter a valid battery level between 0 and 100.")
            except ValueError:
                self.get_logger().warn("Invalid input. Please enter a number.")

def main(args=None):
    rclpy.init(args=args)
    node = FakeBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

