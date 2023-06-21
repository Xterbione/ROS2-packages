#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2
#import realsense2_camera_msgs
from std_msgs.msg import UInt8MultiArray        ## Input data-type
from std_msgs.msg import UInt8MultiArray        ## Output data-type
from std_msgs.msg import String                 ## string type import
from geometry_msgs.msg import Vector3

from std_msgs.msg import MultiArrayLayout, MultiArrayDimension

class myNode(Node):

    def __init__(self):
        super().__init__("cam_obj_detect")   # Names the node
        #isleft_ = False
        self.create_subscription(UInt8MultiArray, "/frame", self.callback, 10)
        #self.create_subscription(UInt8MultiArray, "/camera/color/image_raw", self.callback, 10)

        #self.pub_merge_2d = self.create_publisher(Vector3, "/merge_2d", 10)
        self.pub_tts = self.create_publisher(String, "/ttschannel", 10)
        #self.pub_cutout = self.create_publisher(UInt8MultiArray, "/cutout", 10)
        # Datatype, Topic name, Callback function, buffer size

    def callback(self, msg : UInt8MultiArray):
        print("reached")
        #region Variables
        x_cutoff = 100
        y_cutoff = 50

        x_half = msg.layout.dim[1].size / 2
        y_half = msg.layout.dim[0].size / 2

        dead_zone = 100

        min_size = 300
        print(min_size)
        max_size = 100000
        #endregion

        list = msg.data     ## Gets the list from the msg
        array : cv2.OutputArray = np.asarray(list).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size))    ## Convert the list to an OpenCV format

        ## Step 0: sets the frame to a more narrow FoV
        array = array[0:array.shape[0] - y_cutoff, 0:array.shape[1]]

        ## Step 1: Converting RGB to HSL
        hls = cv2.cvtColor(array, cv2.COLOR_BGR2HLS)

        ## Step 2: Creating a masker, focussed on light parts of the img
        Lchannel = hls[:,:,1]
        mask = cv2.inRange(Lchannel, 200, 255)
        ## Step 3: Blurring the masker to filter out noise
        mask_blur = cv2.boxFilter(mask, -1, (5, 5))

        ## Step 4: Detecting contours within the blurred mask
        ret, thresh = cv2.threshold(mask_blur, 200, 255, cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []

        for c in range(len(contours)):
            area = cv2.contourArea(contours[c])                             ## Area of contour
            #if area < min_size: continue
            perimeter = cv2.arcLength(contours[c], True)                    ## Perimeter of contour
            sides = cv2.approxPolyDP(contours[c], 0.01 * perimeter, True)   ## With len(), amount of sides of contour
            #if len(sides) < 4: continue
            x, y, w, h = cv2.boundingRect(contours[c])                      ## Bounding box of contour
            rat = w / h                                                     ## Width-Height ratio of the bounding box, on a scale of 0 to 1
            if rat > 1: rat = 1 / rat

            ## Step 5: Defined parameters to determine if a contour is a "medicijndoosje"
            corners = len(sides) >= 4 and len(sides) <= 6
            area_size = area > min_size and area < max_size
            has_children = hierarchy[0][c][2] != -1
            ratio = rat >= 0.4

            #isBox = (corners and area_size) or (has_children and area_size)
            isBox = area_size and ratio and (corners or has_children)

            if isBox:
                boxes.append((contours[c], x, y, w, h))

        for b in boxes:
            print("Boxes detected == " + str(len(boxes)))
            x, y, w, h = b[1:5]
            #cropped = array[y:h + y, x:w + x]

            X = x + w / 2
            Y = y + h / 2

            ## Define the message for /rel_pos
            #msg_merge_2d = Vector3()
            direction = ""

            ## If-statements to determine data for the message
            ## Due to the camera currently being mounted upside down, the left and right sides are flipped

            #self.isleft_ = X < x_half - dead_zone
            if  X < x_half - dead_zone:     ## Left side of screen
                Xdif = (-1 + (X / x_half))      ## Normalized value
                print(Xdif)
                #msg_merge_2d.z = Xdif

                direction = direction + " rechts "
                pass
            elif    X > x_half + dead_zone:     ## Right side of screen
                Xdif = ((X - x_half) / x_half)  ## Normalized value
                print(Xdif)
                #msg_merge_2d.z = Xdif

                direction = direction + " links "
                pass
            else:
                direction = direction + " voor "
                pass
            print(direction)
            #if      Y < array.shape[0] / 2:
                #Ydif = 1 - (Y / array.shape[0])
                #msg_rel_pos.angular.x = 50
                #pass
            #elif    Y > array.shape[0] / 2:
                #Ydif = 1 - (array.shape[0] - Y) / array.shape[0]
                #msg_rel_pos.angular.x = -50
                #pass
                
            ## Define the message for /cutout
            #msg_cutout = UInt8MultiArray()
            #msg_cutout.data = cropped.flatten().tolist()

            #layout = MultiArrayLayout()
            #region Layout
            ## Definition of the layout of the UInt8MultiArray
            #layout.dim.append(MultiArrayDimension())
            #layout.dim.append(MultiArrayDimension())
            #layout.dim.append(MultiArrayDimension())

            #layout.dim[0].label = "height"
            #layout.dim[0].size = cropped.shape[0]
            #layout.dim[0].stride = cropped.shape[2] * cropped.shape[1] * cropped.shape[0]

            #layout.dim[1].label = "width"
            #layout.dim[1].size = cropped.shape[1]
            #layout.dim[1].stride = cropped.shape[2] * cropped.shape[1]

            #layout.dim[2].label = "channel"
            #layout.dim[2].size = cropped.shape[2]
            #layout.dim[2].stride = cropped.shape[2]

            #layout.data_offset = 0
            #endregion
            #msg_cutout.layout = layout
            
            #if self.isleft_:
                #direction = " left "
            #else:
                #direction=" right "
            ## Publish the messages
            #self.pub_merge_2d.publish(msg_merge_2d)
            self.pub_tts.publish(String(data=direction))
            #self.get_logger().info(msg_rel_pos.data)
            #self.pub_cutout.publish(msg_cutout)

            break       # For simplicity only handles 1 box
        if len(boxes) == 0:
            self.pub_tts.publish(String(data=" niks "))


        #print("stop")
        
        
def main(args = None):
    rclpy.init(args=args)   # Init ROS2

    node = myNode()         # Creates the node
    rclpy.spin(node)   # Loops the node

    rclpy.shutdown()        # Shutdown ROS2


if __name__ == '__main__':
    main()
