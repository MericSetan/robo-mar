#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from geometry_msgs.msg import Twist



class Control():
    def __init__(self,speed_msg,ref_angle):
        self.ref_angle =  ref_angle #radian
        self.speed_msg = speed_msg  
    
    def follow_left(self,left_angle):  
        if left_angle is not None:  
            diff = left_angle- self.ref_angle

            if diff >0.02:
                
                self.speed_msg.angular.z = diff*5
                self.speed_msg.linear.x = 0.05-diff
                return self.speed_msg
            else:
                self.speed_msg.angular.z = diff*2
                self.speed_msg.linear.x = 0.5
                return self.speed_msg
         
        else:
            self.speed_msg.linear.x  = 0
            self.speed_msg.angular.z  = 0
            return self.speed_msg
                 
        
    def follow_right(self,right_angle):   
        if right_angle is not None:  
            diff = right_angle- self.ref_angle
            
            if diff >0.02:
                
                self.speed_msg.angular.z = -diff*5
                self.speed_msg.linear.x= 0.05+diff
                return self.speed_msg
            else:
                self.speed_msg.angular.z  = -diff*2
                self.speed_msg.linear.x = 0.5
                return self.speed_msg
       
        else:
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            return self.speed_msg 