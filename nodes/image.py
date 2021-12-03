#!/usr/bin/env python


import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2




class Image:
    
    def __init__(self):
        rospy.init_node("image")
        self.subscriber = rospy.Subscriber("image_processed", Image, self.callback)

    

    def callback(self, data):
        rospy.logerr("receiving video frame")
        


    


