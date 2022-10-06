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
        self.autoControl= [0,1,0,0] #turn_right, turn_left, go_straight, stop
        rospy.init_node("main_node")       
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.camera_callback)     
        self.pub = rospy.Publisher("Diff_Drive/diff_drive_controller/cmd_vel",Twist,queue_size=10)
        self.speed_msg = Twist() 
        self.nsn3 = Control(speed_msg=self.speed_msg,ref_angle=1.26) 
        self.mod = 0  
       
        rospy.spin()

    def camera_callback(self,message):
        image = self.bridge.imgmsg_to_cv2(message,"bgr8")        
        second,third,fourth = self.nsn2.crop_image(image)
        leftAngle,rightAngle,red = self.nsn2.lane_keep(image)
       
        if self.nsn.any_table_filter(second):
            self.speed_msg.linear.x = self.speed_msg.linear.x -0.2
            mod = self.nsn.sign_detect(second)
            if mod is not None:
                self.mod_control(mod)
                print(mod)
        
        for i in range(4):
            if self.autoControl[i] ==1:
                if i ==0:
                    self.speed_msg = self.nsn3.follow_right(right_angle=rightAngle,i=self.autoControl[i])
                    self.pub.publish(self.speed_msg)                 
                if i ==1:
                    self.speed_msg = self.nsn3.follow_left(left_angle=leftAngle,i=self.autoControl[i])
                    self.pub.publish(self.speed_msg)                   
                if i ==2:    # düzeltilecek
                    self.speed_msg = self.nsn3.go_straight(i=1)
                    self.pub.publish(self.speed_msg)
                if i ==3:
                    self.speed_msg = self.nsn3.wait_or_stop(i=self.autoControl[i])
                    self.pub.publish(self.speed_msg)  
        print(self.autoControl[:]) 
                 
        cv2.imshow("sa",image)
        cv2.waitKey(1)
            
        
        #derece = leftAngle*180/np.pi         
    def mod_control(self,mod):
        #düzeltilecek
        if mod ==0:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=1
        if mod ==1:
            self.autoControl[0]=1
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=0 
        if mod ==2:
            self.autoControl[0]=0
            self.autoControl[1]=1
            self.autoControl[2]=0
            self.autoControl[3]=1   
        if mod ==3:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=1
            self.autoControl[3]=0 
        if mod ==4:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=0   
        if mod ==5:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=0  
        if mod ==6:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=0   
        if mod ==7:
            self.autoControl[0]=0
            self.autoControl[1]=0
            self.autoControl[2]=0
            self.autoControl[3]=0                           
     


        
#görüntü alma videoCapture ile yapilacak
nsn = MainFunction()
     
  
