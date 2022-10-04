
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from geometry_msgs.msg import Twist



class Control():
    def __init__(self,speed_msg,refAngle):
        self.refAngle =  refAngle #radian
        self.speed_msg = speed_msg  
    
    def solSeritTakip(self,aciSol):  
        if aciSol is not None:  
            fark = aciSol- self.refAngle

            if fark >0.02:
                
                self.speed_msg.angular.z = fark*5
                self.speed_msg.linear.x = 0.05-fark
                return self.speed_msg
            else:
                self.speed_msg.angular.z = fark*2
                self.speed_msg.linear.x = 0.5
                return self.speed_msg
         
        else:
            self.speed_msg.linear.x  = 0
            self.speed_msg.angular.z  = 0
            return self.speed_msg
                 
        
    def sagSeritTakip(self,aciSag):   
        if aciSag is not None:  
            fark = aciSag- self.refAngle
            
            if fark >0.02:
                
                self.speed_msg.angular.z = -fark*5
                self.speed_msg.linear.x= 0.05+fark
                return self.speed_msg
            else:
                self.speed_msg.angular.z  = -fark*2
                self.speed_msg.linear.x = 0.5
                return self.speed_msg
       
        else:
            self.speed_msg.linear.x = 0
            self.speed_msg.angular.z = 0
            return self.speed_msg 