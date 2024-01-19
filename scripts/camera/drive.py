#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
import cv2
import numpy as np
import morai_msgs.msg as CtrlCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from slidewindow_test import SlideWindow
from Preprocess import Preprocess
from control.pidcal import PidCal

class Test_drive:
    def __init__(self):
        rospy.init_node('Test_drive', anonymous=True)
        # self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd.ControlCommand, queue_size=1)
        # publiser
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        # subscriber
        self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)

        self.slidewindow = SlideWindow()
        self.preprocess = Preprocess()
        self.pidcal = PidCal()
        self.img = None

    
        

    
    def callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, angle,steering = self.lane_detection(img_bgr)
        #scaling angle to 0 ~ 180 -> 0.0 ~ 1.0
        real_steering = angle / 180.0
        steering /= -1.0
        steering_mean = (real_steering + steering) / 2.0
        print("steering : ", steering)
        print("real_steering : ", real_steering)
        print("steering_mean : ", steering_mean)
        cv2.imshow("Image window", img_bgr)
        # if if_detect:
            # cv2.imshow("yellow", yellow)
        self.speed_pub.publish(1000)
        self.steer_pub.publish(real_steering)

        # 0 ~ 1 -> -19.5 ~ 19.5 
        if slideing_img is not None:
            cv2.imshow("Slidinw window", slideing_img)
        cv2.waitKey(1)
        # print("steering : ", steering)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
        # cv2.imshow("preprocess", img)
        img, angle, center= self.slidewindow.slidewindow(img)
        # if x_location is None:
        #     x_location = 318
        
        steering = -self.pidcal.pid_control(center)
        # print("steering : ", steering) 
        # cv2.imshow("Slidinw window", img)print('degree : ', degree)
        cv2.waitKey(1)
        return img, angle, steering

if __name__ == '__main__':

    try:
        test_drive = Test_drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass