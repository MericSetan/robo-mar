#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge






#line
low_threshold = 100
high_threshold =200
#red
lowcolor =  (0,0,255)
highcolor = (50,50,255)

mod_list = []

class SeritTakip():

    def __init__(self):       
        rospy.init_node("serit_takip")       
        self.bridge = CvBridge()
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)     
        self.pub = rospy.Publisher("Diff_Drive/diff_drive_controller/cmd_vel",Twist,queue_size=10)
        self.hiz_mesaji = Twist()    
        self.autoControl = [0,0,0]   
        rospy.spin()

    def kameraCallback(self,mesaj):
        
        
        image = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")    
        second,third,fourth = self.crop_image(image)
        new = cv2.flip(fourth, 1)
        aciSol = self.get_moments(image,third)    #ort 72^ 1.26rad    
        aciSag = self.get_moments(image,new)  #ort 72^ 1.26rad cx35
     
      
        self.autoControl[2] =0
        self.autoControl[1] =0
        self.autoControl[0] = self.detectRed(second)         
        self.solSeritTakip(self.autoControl[2],aciSol)
        self.sagSeritTakip(self.autoControl[1],aciSag)

        cv2.imshow("orj",image)
        cv2.imshow("third",third)
        cv2.imshow("fourth",new)      
        cv2.waitKey(1)
     
    def crop_image(self,image):
            return image[ 0:240, 320:640],image[300:480, 0:320],image[300:480, 320:640]

    def get_moments(self,image,area):
        hsv = cv2.cvtColor(area,cv2.COLOR_RGB2HSV)
        h,w,_ = area.shape[:]
        a,b,_ = image.shape[:]
        d_black = np.array([0,0,0])
        u_black = np.array([80,150,200])
        maske = cv2.inRange(hsv,d_black,u_black)
        #sonuc = cv2.bitwise_and(area,area,mask=maske)        
        M = cv2.moments(maske)
        cv2.circle(image,(int(b/2),int(a/2)),5,(0,0,0),-1)
        
        if M['m00']:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(area,(cx,int(h/2)),5,(255,0,0),-1)
            return np.arctan(((b/2)-cx)/(h/2))



    def solSeritTakip(self,i,aciSol):
        istenilen =  1.26
        
        if i == 1:
            if aciSol is not None:  
                fark = aciSol- istenilen
                if fark >0.02:
                    
                    self.hiz_mesaji.angular.z = fark*5
                    self.hiz_mesaji.linear.x = 0.05-fark
                    self.pub.publish(self.hiz_mesaji) 
                else:
                    self.hiz_mesaji.angular.z = fark*2
                    self.hiz_mesaji.linear.x = 0.5
                    self.pub.publish(self.hiz_mesaji) 
                derece = aciSol*180/np.pi         
                print(derece)        
            else:
                self.hiz_mesaji.linear.x = 0
                self.hiz_mesaji.angular.z = 0
                self.pub.publish(self.hiz_mesaji)             
        
    def sagSeritTakip(self,i,aciSag):
        istenilen =  1.26
        
        if i == 1:
            if aciSag is not None:  
                fark = aciSag- istenilen
                if fark >0.02:
                    
                    self.hiz_mesaji.angular.z = -fark*5
                    self.hiz_mesaji.linear.x = 0.05+fark
                    self.pub.publish(self.hiz_mesaji) 
                else:
                    self.hiz_mesaji.angular.z = -fark*2
                    self.hiz_mesaji.linear.x = 0.5
                    self.pub.publish(self.hiz_mesaji) 
                derece = aciSag*180/np.pi         
                print(derece)        
            else:
                self.hiz_mesaji.linear.x = 0
                self.hiz_mesaji.angular.z = 0
                self.pub.publish(self.hiz_mesaji)    

    def detectRed(self,image):
        
        thresh = cv2.inRange(image, lowcolor, highcolor)
        count = np.sum(np.nonzero(thresh)) #other way
        average = cv2.mean(thresh)[0]
        
        if average>100:
            return 1
        return 0


SeritTakip()        

