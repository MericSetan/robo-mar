#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from control import Control
from lane_keeping import LaneKeeping
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from traffic_sign_detect import TrafficSignDetectionModule



class MainFunction():

    def __init__(self):    
        self.nsn =TrafficSignDetectionModule()   
        self.nsn2 = LaneKeeping()
        
        rospy.init_node("main_node")       
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.camera_callback)     
        self.pub = rospy.Publisher("Diff_Drive/diff_drive_controller/cmd_vel",Twist,queue_size=10)
        self.speed_msg = Twist() 
        self.nsn3 = Control(speed_msg=self.speed_msg,refAngle=1.26)       
        rospy.spin()

    def camera_callback(self,message):
        image = self.bridge.imgmsg_to_cv2(message,"bgr8")    
        self.nsn.sign_detect(image)
        leftAngle,rightAngle,red = self.nsn2.lane_keep(image)

        self.speed_msg = self.nsn3.sagSeritTakip(rightAngle)
        self.speed_msg = self.nsn3.solSeritTakip(leftAngle)
        #derece = leftAngle*180/np.pi         
        

        cv2.imshow("sa",image)
        cv2.waitKey(1)
        
        
#görüntü alma videoCapture ile yapilacak
nsn = MainFunction()
     
  
