#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from tflite_runtime.interpreter import Interpreter
import cv2
import numpy as np



class TrafficSignDetectionModule():

    def __init__(self):  
        self.model_tabelalar = Interpreter(
        model_path=r'/home/mericsetan/catkin_ws/src/far_mo_v2/src/adsiz_model_8.tflite')
        self.model_yonler = Interpreter(
        model_path=r"/home/mericsetan/catkin_ws/src/far_mo_v2/src/sag_sol_modeli.tflite")
        self.input_details_tabela = self.model_tabelalar.get_input_details()
        self.output_details_tabela = self.model_tabelalar.get_output_details()
        self.input_details_yon = self.model_yonler.get_input_details()
        self.output_details_yon = self.model_yonler.get_output_details()

        self.sayac = 0
        self.mod_list = []     

    def sign_detect(self,image):
        if self.filtre(image):
                self.sayac +=  1
                img = cv2.resize(image, (128, 128))
                array = np.array(np.array([img]) / 255.0, dtype=np.float32)
                self.model_tabelalar.allocate_tensors()
                self.model_tabelalar.set_tensor(self.input_details_tabela[0]['index'], array)
                self.model_tabelalar.invoke()
                output = self.model_tabelalar.get_tensor(self.output_details_tabela[0]['index'])
                pred = np.where(output == np.max(output))
                if pred[1][0] == 1 or pred[1][0] == 2:
                    self.model_yonler.allocate_tensors()
                    self.model_yonler.set_tensor(self.input_details_yon[0]['index'], array)
                    self.model_yonler.invoke()
                    output = self.model_yonler.get_tensor(self.output_details_yon[0]['index'])
                    if output[0][0] > 0.1:
                        pred = [[0], [1]]
                    else:
                        pred = [[0], [2]]
                self.mod_list.append(pred[1][0])
                cv2.imshow('kare', image)
                if self.sayac == 21:
                    mod_prediction = np.bincount(self.mod_list).argmax()
                    print(mod_prediction)
                    self.mod_list = []
                    self.sayac = 0
                cv2.waitKey(1)
        else:
                print("tabela algilanamadı")
        
        
    def filtre(self,image):
        low =  (0,0,255)
        high = (50,50,255)
        hsv_filter = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_filter, low, high)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (8, 8))
        cv2.imshow('kare', image)
        cv2.waitKey(1)
        if np.max(mask) == 255:
            return True
        else:
            return False       

