#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

#line
low_threshold = 100
high_threshold =200
#red
lowcolor =  (0,0,255)
highcolor = (50,50,255)

class LaneKeeping():

    def lane_keep(self,image):
        second,third,fourth = self.crop_image(image)
        new = cv2.flip(fourth, 1)

        leftAngle = self.get_moments(image,third)    #ort 72^ 1.26rad    
        rightAngle = self.get_moments(image,new)  #ort 72^ 1.26rad cx35
        red = self.detect_red(image)     
        
        
        cv2.imshow("third",third)
        cv2.imshow("fourth",new)      
        cv2.imshow("second",second)   
        cv2.waitKey(1)
        return leftAngle,rightAngle,red

    def crop_image(self,image):
        return image[ 0:240, 320:640],image[300:480, 0:320],image[300:480, 320:640]

    def get_moments(self,image,area):
        hsv = cv2.cvtColor(area,cv2.COLOR_RGB2HSV)
        h,w,_ = area.shape[:]
        a,b,_ = image.shape[:]
        d_black = np.array([0,0,0])
        u_black = np.array([80,150,200])
        mask = cv2.inRange(hsv,d_black,u_black)
        #sonuc = cv2.bitwise_and(area,area,mask=maske)        
        M = cv2.moments(mask)
        cv2.circle(image,(int(b/2),int(a/2)),5,(0,0,0),-1)   
        if M['m00']:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(area,(cx,int(h/2)),5,(255,0,0),-1)
            return np.arctan(((b/2)-cx)/(h/2))

    def detect_red(self,image):
        
        thresh = cv2.inRange(image, lowcolor, highcolor)
        count = np.sum(np.nonzero(thresh)) #other way
        average = cv2.mean(thresh)[0]
        
        if average>100:
            return 1
        return 0
