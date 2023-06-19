#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import RandomData
import random

class RandomDataNode(Node):
    def __init__(self):
        super().__init__("RandomData")
        self.publisher_ = self.create_publisher(RandomData, '/RandomData', 10)
        self.get_logger().info("RandomData has started")
        self.timer_ = self.create_timer(1, self.SendData)

    def SendData(self):
        msg = RandomData()
        msg.gs1 = '7240Afnemercode4301I7050'
        msg.weight = random.randint(0,1000)
        msg.highband = random.randint(0,255)
        msg.midband = random.randint(0,255)
        msg.lowband = random.randint(0,255)
        msg.batteryvoltage = random.randint(90,120)
        self.publisher_.publish(msg)
#        self.get_logger().info('Published speed: {}'.format(msg.set_speed1))
#        self.get_logger().info('Published torque: {}'.format(msg.max_torque1))
#        self.get_logger().info('Published position: {}'.format(msg.set_pos1))


def main(args=None):
    rclpy.init(args=args)
    RandomData = RandomDataNode()
    rclpy.spin(RandomData)
    RandomData.destroy_node()
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()