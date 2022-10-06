#!/usr/bin/env python3s
# -*- coding: utf-8 -*-

from itertools import count
from tflite_runtime.interpreter import Interpreter
import cv2
import numpy as np

#sağa dön ve düz giti karıştırıyor

class TrafficSignDetectionModule():

    def __init__(self):  
        self.model_signs = Interpreter(
        model_path=r'/home/mericsetan/catkin_ws/src/far_mo_v2/src/adsiz_model_8.tflite')
        self.model_directions = Interpreter(
        model_path=r"/home/mericsetan/catkin_ws/src/far_mo_v2/src/sag_sol_modeli.tflite")
        self.input_details_sign = self.model_signs.get_input_details()
        self.output_details_sign = self.model_signs.get_output_details()
        self.input_details_directions = self.model_directions.get_input_details()
        self.output_details_directions = self.model_directions.get_output_details()

        self.counter = 0
        self.mod_list = []     

    def sign_detect(self,image):
        self.counter +=  1
        img = cv2.resize(image, (128, 128))
        array = np.array(np.array([img]) / 255.0, dtype=np.float32)
        self.model_signs.allocate_tensors()
        self.model_signs.set_tensor(self.input_details_sign[0]['index'], array)
        self.model_signs.invoke()
        output = self.model_signs.get_tensor(self.output_details_sign[0]['index'])
        pred = np.where(output == np.max(output))
        if pred[1][0] == 1 or pred[1][0] == 2:
            self.model_directions.allocate_tensors()
            self.model_directions.set_tensor(self.input_details_directions[0]['index'], array)
            self.model_directions.invoke()
            output = self.model_directions.get_tensor(self.output_details_directions[0]['index'])
            if output[0][0] > 0.1:
                pred = [[0], [1]]
            else:
                pred = [[0], [2]]
        self.mod_list.append(pred[1][0])
        if self.counter == 21:
            mod_prediction = np.bincount(self.mod_list).argmax()
            self.mod_list = []
            self.counter = 0
            return mod_prediction

                
        
        
    def any_table_filter(self,image):
        low =  (0,0,0)
        high = (50,50,255)
        thresh = cv2.inRange(image, low, high)
        average = cv2.mean(thresh)[0]
        if average>5:
            return True
        else:
            return False       

