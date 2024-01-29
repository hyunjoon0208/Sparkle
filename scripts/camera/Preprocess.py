#!/usr/bin/env python3

import cv2
import os, sys
# sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
from  .warper import Warper

class Preprocess:
    def __init__(self) -> None:
        self.warper = Warper()

    def preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        img = self.warper.warp(img)
        # img = cv2.GaussianBlur(img, (3,3), 0)
        # img = cv2.Canny(img, 100, 200)
        img = cv2.inRange(img, 100, 255)
        return img

    def find_yellow(self,img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        img = cv2.inRange(img, lower_yellow, upper_yellow)
        img = self.warper.warp(img)
        return img