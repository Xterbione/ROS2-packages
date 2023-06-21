#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray    ## Input
from std_msgs.msg import String             ## Output

import numpy as np
import cv2
import datetime

#import pylibdmtx
from pylibdmtx import pylibdmtx
## If the pylibdmtx library doesn't work, pip or another package manager probably installed it at the wrong location
## To move the installed library to the right location use: "sudo mv" (Linux)
## sudo mv /home/marten/.local/lib/python3.10/site-packages/pylibdmtx /usr/lib/python3/dist-packages
## sudo mv /home/marten/.local/lib/python3.10/site-packages/pylibdmtx-0.1.10.dist-info /usr/lib/python3/dist-packages

class myNode(Node):
    def __init__(self):
        super().__init__("cam_gs1")   # Names the node

        self.create_subscription(UInt8MultiArray, "/frame", self.callback, 10)
        #self.create_subscription(UInt8MultiArray, "/camera/color/image_raw", self.callback, 10)
        #self.create_subscription(UInt8, "/trigger", self.callback, 10)

        self.pub_gs1 = self.create_publisher(String, "/gs1", 10)
        # Datatype, Topic name, Callback function, buffer size

    def callback(self, msg : UInt8MultiArray):
        print("reached")

        list = msg.data     ## Gets the list from the msg
        array : cv2.OutputArray = np.asarray(list).reshape((msg.layout.dim[0].size, msg.layout.dim[1].size, msg.layout.dim[2].size))  ## Convert the list to an OpenCV format

        count = 10
        output = str()

        ## Defining the message
        msg_gs1 = String()

        while count > 0:
            ## Converts the given image to grayscale and searches for thresholds
            gray = cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)
            ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_OTSU)

            ## Uses the found thresholds to search for GS1 data-matrixes
            ## Returns a array of tuples, one tuple for each found GS1 data-matrix
            gs1 = pylibdmtx.decode(thresh)

            if gs1:                
                ## Opens file, awaiting appending
                file = open("~/ubuntu/log/GS1/" + str(datetime.date.today()) + ".log", "a")
                for item in gs1:
                    ## Decodes the GS1 data
                    decode = item[0].decode()
                    print(decode)
                    ## Writes timestamp with decoded GS1 data
                    data = '{"timestamp":"' + datetime.datetime.now().strftime("%X") + '","data":"' + decode + '"}'
                    file.write(data + "\n")

                    output = output + decode + ","

                ## Cuts the last comma of from the output string
                output = output[:-1]
                ## Sets data and publishes the message
                msg_gs1.data = output
                self.pub_gs1.publish(msg_gs1)
                ## Closes file
                file.close()
                return
                
            else:                
                print("Nothing detected")

            count = count - 1

        ## If after "count" tries no GS1 has been detected, publishes the string "Nothing" and kills the node
        output = "Nothing"
        msg_gs1.data = output
        self.pub_gs1.publish(msg_gs1)
            
        
def main(args = None):
    rclpy.init(args=args)   # Init ROS2

    node = myNode()         # Creates the node
    rclpy.spin_once(node)   # Loops the node

    rclpy.shutdown()        # Shutdown ROS2


if __name__ == '__main__':
    main()
