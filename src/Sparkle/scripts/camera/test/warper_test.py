#!/usr/bin/env python3

import cv2
import numpy as np


class Warper:
    def __init__(self):
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)

        self.src = np.float32([
            [0,h*0.56],
            [w, h*0.56],
            [w,h],
            [0, h],
        ])
        self.dst = np.float32([
            [w*(-2),0],
            [w*(3), 0],
            [w*(0.85),h],
            [w*(0.15), h],
        ])
        # dst = np.float32([
        #     [0, 0],
        #     [w, 0],
        #     [0, h],
        #     [w , h],
        # ])
        self.CL_src = np.float32([
            [0, h*0.575],
            [w, h*0.575],
            [w, h],
            [0, h],
        ])
        self.CL_dst = np.float32([
            [w*(-0.6),0],
            [w*(1.4), 0],
            [w*(0.75),h],
            [w*(0.25), h],
        ])

        self.CLM = cv2.getPerspectiveTransform(self.CL_src, self.CL_dst)
        self.CLMinv = cv2.getPerspectiveTransform(self.CL_dst, self.CL_src)

        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)

    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )

    # def unwarp(self, img):
    #     return cv2.warpPersective(
    #         img,
    #         self.Minv,
    #         (img.shape[1], img.shape[0]),
    #         flags=cv2.INTER_LINEAR
    #     )
    
    def left_warp(self, img):
        return cv2.warpPerspective(
            img,
            self.CLM,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )