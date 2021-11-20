#!/usr/bin/env python
""" A python node to read images from a web camera and process with opencv 
    Subscribes: usb_cam/image_raw (sensor_msgs/Image) - The input raw image
    Publishes: image_processed (sensor_msgs/Image) - The processed image
"""

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class Bridge:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher("ros_image", Image, queue_size=10)
        self.image_subscriber = rospy.Subscriber("usb_cam/image_raw", Image, self.callback)
        self.rospy.init_node("vision_bridge")

    def callback(self,data):

        #convert opencv image to ros image

        image_msg = self.bridge.cv2_to_imgmsg(data, "bgr8")

        self.image_publisher(msg)


if __name__ == "__main__":
    b = Bridge()
    rospy.spin()