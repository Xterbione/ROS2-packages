#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist     ## Output
from geometry_msgs.msg import Vector3   ## Input

class myNode(Node):
    # cmv_vel_pub_ = Node.create_publisher()
    def __init__(self):
        super().__init__("output_merge")   # Names the node

        self.base : Twist = Twist()
        
        self.two_dim : Vector3 = Vector3()
        self.three_dim : Vector3 = Vector3()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Datatype, Topic name, buffer size (needs external callback call)
        self.create_subscription(Vector3, "/merge_2d", self.sub_2d, 10)
        self.create_subscription(Vector3, "/merge_3d", self.sub_3d, 10)

        self.create_timer(0.1, self.timer)
        #self.videoCap()

    def timer(self):
        max_diffrence =  10
        max_drive = 127

        dim2 = self.two_dim
        dim3 = self.three_dim

        msg = Twist()

        ## If the angular z of the 3d is within the maximum diffrence of the angular z of the 2d OR The angular z of the 3d is 0
        if ((dim3.z < self.base.angular.z - max_diffrence and dim3.z > self.base.angular.z + max_diffrence) and dim2.z != 0.0) or dim3.z == 0.0:
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = (dim2.z + self.base.angular.z) / 2 * max_drive

        ## If the 2d doesn't detect something but the 3d does
        elif (dim2.z == 0.0 and dim3.z != 0.0):
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = (dim3.z + self.base.angular.z) / 2 * max_drive

        ## Takes the average of both 3d and 2d
        else:
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = (dim2.z + dim3.z + self.base.angular.z) / 3 * max_drive

        ## Average the most recent incoming messages with the base
        msg.linear.x = 0.0
        #msg.linear.y = (ddd.linear.y + self.base.linear.y) / 2 * max_drive
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        self.base = msg     ## Sets last send message as the base
        
        self.cmd_vel_pub.publish(msg)

    def sub_2d(self, msg : Vector3):
        self.two_dim = msg
        #pass

    def sub_3d(self, msg : Vector3):
        self.three_dim = msg
        #pass

def main(args = None):
    rclpy.init(args=args)

    node = myNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()