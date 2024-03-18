#!/usr/bin/env python3

import cv2
import numpy as np
from warper_test import Warper
class Preprocess:
    def __init__(self) -> None:
        self.warper = Warper()

    def preprocess(self, img,warp_flag = 'CL'):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.inRange(img, 100, 255)
        
        if warp_flag == 'CL':
            warp_img = self.warper.left_warp(img)
        else:
            warp_img = self.warper.warp(img)
        # img = cv2.GaussianBlur(img, (3,3), 0)
        # img = cv2.Canny(img, 100, 200)
        return warp_img

    def find_yellow(self,img):
        try:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lower = np.array([20, 100, 100])
            upper = np.array([30, 255, 255])
            mask = cv2.inRange(img, lower, upper)
            finded = cv2.bitwise_and(img, img, mask=mask)
        except:
            finded = None

            
        if finded is not None:
            return True, finded
        else:
            return False, None