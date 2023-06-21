#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class drawCircle(Node):
    # cmv_vel_pub_ = Node.create_publisher()
    def __init__(self):
        super().__init__("circle")
        self.cmv_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_timer(5, self.timer)
        self.get_logger().info("Circle")

    def timer(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -6.0
        self.cmv_vel_pub_.publish(msg)

def main(args = None):
    rclpy.init(args=args)

    node = drawCircle()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()