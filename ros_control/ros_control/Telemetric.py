#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from topics_services.msg import Telemetric
#from topics_services.msg import ServoData

class TelemetricNode(Node):
    def __init__(self):
        super().__init__("Telemetric")
        self.subscription = self.create_subscription(Telemetric, '/Telemetric', self.callback, 10)
#        self.subscription = self.create_subscription(ServoData, '/ServoData', self.callback, 10)
        self.get_logger().info("telementric input has started")

    def callback(self, msg):
        # Set servo position based on received data
        self.get_logger().info('Received position: {}'.format(msg.cur_pos_fr))
        # self.get_logger().info('Received temperature: {}'.format(msg.temp_fr))
        # self.get_logger().info('Received load: {}'.format(msg.load_fr))
        # self.get_logger().info('Received voltage: {}'.format(msg.voltage_fr))


def main(args=None):
    rclpy.init(args=args)
    telementrics = TelemetricNode()
    rclpy.spin(telementrics)
    telementrics.destroy_node()
    rclpy.shutdown()

#if __name__ == '__main__':
#    main()