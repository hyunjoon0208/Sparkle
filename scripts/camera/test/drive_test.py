#! /usr/bin/env python3

import rospy, os, sys, cv2, numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import morai_msgs.msg as CtrlCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage, Imu
from slidewindow_test2 import SlideWindow
from Preprocess_test import Preprocess
from pidcal_test import PidCal
from StopDetector import stop_line

class Test_drive:
    def __init__(self):
        rospy.init_node('Test_drive', anonymous=True)
        # self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd.ControlCommand, queue_size=1)
        # publiser
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        # subscriber
        self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)
        self.yaw_cub = rospy.Subscriber('/imu', Imu, self.Imu_callback, queue_size=1)
        self.yaw = 0
        self.slidewindow = SlideWindow()
        self.preprocess = Preprocess()
        self.pidcal = PidCal()
        self.stop_detector = stop_line()
        self.img = None
        self.line_flag = 'CL'
        self.speed = 2400

    def Imu_callback(self, msg):
        self.yaw = abs(msg.orientation.z)


    def callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        img, sliding_img, x_location, line_flag =  self.lane_detection(img_bgr)
        
        stopline = self.stop_detector.isStop(img)
        if stopline:
            self.speed = 0
            steering = 0.5
            print("stopline detected")
        if self.line_flag == 'R':
            pid = self.pidcal.pid_control(x_location)
            steering = abs(pid - 0.5)
        elif self.line_flag == 'L':
            pid = self.pidcal.pid_control(x_location)
            steering = abs(pid - 0.5)
        else:
            pid = self.pidcal.curve_pid_control(x_location)
            steering = abs(pid - 0.5) 
            if steering < 0.5:
                steering *= 1.2
            if (0.68<self.yaw<0.77):
                # steering = 0.5
                self.line_flag = 'R'
                self.speed = 600
                


        print('steering : ', steering)
        self.speed_pub.publish(self.speed)
        self.steer_pub.publish(steering)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img, self.line_flag)
        slideimg, x_location , line_flag = self.slidewindow.slidewindow(img,self.line_flag)
        cv2.imshow("slide img", slideimg)
        cv2.imshow('stopline', img)
        cv2.waitKey(1)
        return img,slideimg, x_location, line_flag


if __name__ == '__main__':
    try:
        Test_drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass