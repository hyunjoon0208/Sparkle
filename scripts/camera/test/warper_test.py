import cv2
import numpy as np

class warper:
    def __init__(self) -> None:
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)

        self.src = np.float32([ # 4개의 원본 좌표 점
            [w * 1.6, h * 1.3], # [1024, 624]
            [w * (-0.1), h * 1.3], # [-64.0, 624]
            [0, h * 0.62], # [0, 297.6]
            [w, h * 0.62], # [640, 297.6]
        ])
        self.dst = np.float32([ # 4개의 결과 좌표 점
            [w * 0.65, h * 0.98], # [416, 470.4]
            [w * 0.35, h * 0.98], # [224, 470.4]
            [w * (-0.3), 0], # [-192, 0]
            [w * 1.3, 0], # [832, 0]
        ])


        self.M = cv2.getPerspectiveTransform(self.src, self.dst)
        self.Minv = cv2.getPerspectiveTransform(self.dst, self.src)


    def warp(self, img):
        return cv2.warpPerspective(
            img,
            self.M,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )
    
    def unwarp(self, img):
        return cv2.warpPerspective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR
        )