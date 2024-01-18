#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from Sparkle.scripts.slidewindow import SlideWindow
from pidcal import PidCal
from Preprocess import Preprocess


class LaneDetector:
    def __init__(self) -> None:
        rospy.init_node('lane_detector', anonymous=True)
        self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/lane_detection', Image, queue_size=1)
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()
        self.preprocess = Preprocess()
        self.img = None
        rospy.spin()
    def callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, steering = self.lane_detection(img_bgr)
        if_detect, yellow = self.preprocess.find_yellow(img_bgr)
        cv2.imshow("Image window", img_bgr)
        if if_detect:
            cv2.imshow("yellow", yellow)
        if slideing_img is not None:
            cv2.imshow("Slidinw window", slideing_img)
        cv2.waitKey(1)
        print("steering : ", steering)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
        img, x_location = self.slidewindow.slidewindow(img)
        steering = self.pidcal.pid_control(x_location)
        cv2.imshow("Slidinw window", img)
        cv2.waitKey(1)
        return img, steering
    


if __name__ == '__main__':
    try:
        lane_detector = LaneDetector()

    except rospy.ROSInterruptException:
        pass