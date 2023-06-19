#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import ServoData
import random

class SerialInputNode(Node):
    def __init__(self):
        super().__init__("serial_input")
        self.publisher_ = self.create_publisher(ServoData, '/ServoData', 10)
        self.get_logger().info("serial input has started")
        self.timer_ = self.create_timer(1, self.SendData)


    def SendData(self):
        msg = ServoData()
#        msg.set_pos1 = 314  # Replace with your desired float32 value
        # msg.set_speed1 = random.randint(0,255)
        # msg.max_torque1 = random.randint(0,255)
        # msg.set_pos1 = random.randint(0,1000)
        self.publisher_.publish(msg)
        self.get_logger().info('Published speed: {}'.format(msg.set_speed1))
        self.get_logger().info('Published torque: {}'.format(msg.max_torque1))
        self.get_logger().info('Published position: {}'.format(msg.set_pos1))


def main(args=None):
    rclpy.init(args=args)
    node = SerialInputNode()
    rclpy.spin(node)
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()