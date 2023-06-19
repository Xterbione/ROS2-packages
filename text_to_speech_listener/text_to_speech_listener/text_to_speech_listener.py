#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3

#defining the node
class TTSListenerNode(Node):
    def __init__(self):
        super().__init__('text_to_speech_listener')
        self.engine = pyttsx3.init()
        #subscribing and entering callback for message
        self.subscription = self.create_subscription(
            String,
            'ttschannel',
            self.text_callback,
            0
        )
        self.subscription


    #defining the callback
    def text_callback(self, msg):
        self.get_logger().info('Received text: %s' % msg.data)
        self.say(msg.data)


    #function for talking
    def say(self, text):
        self.engine.say(text)
        self.engine.runAndWait()

#dmain function
def main(args=None):
    rclpy.init(args=args)
    node = TTSListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
