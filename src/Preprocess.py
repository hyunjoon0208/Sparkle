import cv2
from warper import Warper
class Preprocess:
    def __init__(self) -> None:
        self.warper = Warper()

    def preprocess(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # img = cv2.GaussianBlur(img, (3,3), 0)
        img = cv2.Canny(img, 50, 150)
        img = self.warper.warp(img)
        return img
