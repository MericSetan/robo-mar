#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from geometry_msgs.msg import Twist



class Control():
    def __init__(self,speed_msg,ref_angle):
        self.ref_angle =  ref_angle #radian
        self.speed_msg = speed_msg  
    
    def follow_left(self,left_angle,i):  
        if i==1:
            if left_angle is not None:  
                diff = left_angle- self.ref_angle

                if diff >0.02:
                    
                    self.speed_msg.angular.z = diff*5
                    self.speed_msg.linear.x = 0.06-diff
                    return self.speed_msg
                else:
                    self.speed_msg.angular.z = diff*2
                    self.speed_msg.linear.x = 0.4
                    return self.speed_msg
            
            else:
                self.speed_msg.linear.x  = 0
                self.speed_msg.angular.z  = self.speed_msg.angular.z 
                return self.speed_msg
        else :
            pass                 
        
    def follow_right(self,right_angle,i):   
        if i==1:
            if right_angle is not None:  
                diff = right_angle- self.ref_angle
                
                if diff >0.02:
                    
                    self.speed_msg.angular.z = -diff*5
                    self.speed_msg.linear.x= 0.06+diff
                    return self.speed_msg
                else:
                    self.speed_msg.angular.z  = -diff*2
                    self.speed_msg.linear.x = 0.4
                    return self.speed_msg
        
            else:
                self.speed_msg.linear.x = 0
                self.speed_msg.angular.z = self.speed_msg.angular.z 
                return self.speed_msg 
        else :
            pass        


    def wait_or_stop(self,i):
        if i==1:
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            return self.speed_msg 

    def go_straight(self,i):
        if i==1:
            self.speed_msg.linear.x = 0.5
            self.speed_msg.angular.z = 0
            return self.speed_msg 