#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
import os

#here we define the node
class TTSListenerNode(Node):
    def __init__(self):
        super().__init__('emoteaudioplayer')
        self.subscription = self.create_subscription(
            String,
            'emoteaudioplayer',
            self.text_callback,
            5
        )
        self.subscription
        #initialize the lib
        pygame.init()


        #define callback for the subscription
    def text_callback(self, msg):
        self.get_logger().info('Received text: %s' % msg.data)
        self.play_sound(msg.data)





        #playsound function
    def play_sound(self, text):
        sound_file = self.get_sound_file(text)
        if sound_file:
            pygame.mixer.music.load(sound_file)
            pygame.mixer.music.play()






        #constructing the os path here
    def get_sound_file(self, text):
        sound_files_dir = '/home/ubuntu/ros2_ws/src/text_to_speech_listener/audio/'  
        sound_file = None

        if 'happy' in text:
            sound_file = os.path.join(sound_files_dir, 'happy.mpga')
        elif 'sad' in text:
            sound_file = os.path.join(sound_files_dir, 'sad.mpga')
        elif 'scream' in text:
            sound_file = os.path.join(sound_files_dir, 'scream.mpga')
        elif 'action' in text:
            sound_file = os.path.join(sound_files_dir, 'action.mpga')

        return sound_file

#main function
def main(args=None):
    rclpy.init(args=args)
    node = TTSListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()