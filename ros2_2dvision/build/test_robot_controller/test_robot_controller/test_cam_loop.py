#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray    ## Output
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

import numpy as np
import cv2

class myNode(Node):

    def __init__(self):
        super().__init__("cam_loop")   # Names the node

        self.publisher_frame = self.create_publisher(UInt8MultiArray, "/frame", 10)
        
        # self.publisher_depth = self.create_publisher(UInt8MultiArray, "/depth", 10)
        # Datatype, Topic name, buffer size (needs external callback call)

        #self.create_timer(2.0, self.videoCap)
        self.videoCap()


    def videoCap(self):
        #realsense = False
        cam = 0
        frame_count = 0

        cap = cv2.VideoCapture(cam)

        #if realsense:
            #pipe = rs.pipeline()
            #config = rs.config()

            #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

            #pipe.start(config)

        while cap.isOpened() == True:
            ## Takes a singular frame from the video feed
            ret, frame = cap.read()

            if not ret:
                break

            #if realsense:
                #frame2 = pipe.wait_for_frames()
                #depth = frame2.get_depth_frame()

                #msg_depth = UInt8MultiArray()                 ## Defining the message
                #msg_depth.data = frame.flatten().tolist()     ## Inserting the data into the message

            msg_frame = UInt8MultiArray()                 ## Defining the message
            msg_frame.data = frame.flatten().tolist()     ## Inserting the data into the message

            #region
            ## Definition of the layout of the UInt8MultiArray
            layout = MultiArrayLayout()

            layout.dim.append(MultiArrayDimension())
            layout.dim.append(MultiArrayDimension())
            layout.dim.append(MultiArrayDimension())

            layout.dim[0].label = "height"
            layout.dim[0].size = frame.shape[0]
            layout.dim[0].stride = frame.shape[2] * frame.shape[1] * frame.shape[0]

            layout.dim[1].label = "width"
            layout.dim[1].size = frame.shape[1]
            layout.dim[1].stride = frame.shape[2] * frame.shape[1]

            layout.dim[2].label = "channel"
            layout.dim[2].size = frame.shape[2]
            layout.dim[2].stride = frame.shape[2]

            layout.data_offset = 0
            #endregion

            msg_frame.layout = layout                     ## Sets the above defined Layout
            #msg_depth.layout = layout                     ## Sets the above defined Layout

            if frame_count % 5 == 0:
                self.publisher_frame.publish(msg_frame)      ## Publishes the message
            frame_count = frame_count + 1
            #if realsense: self.publisher_depth.publish(msg_depth)      ## Publishes the message
            #print("published")
            #break

        cap.release()
        cv2.destroyAllWindows()
            
        
def main(args = None):
    rclpy.init(args=args)   # Init ROS2

    node = myNode()         # Creates the node
    rclpy.spin(node)        # Loops the node

    rclpy.shutdown()        # Shutdown ROS2


if __name__ == '__main__':
    main()
