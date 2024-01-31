#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from math import pi
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, LaserScan
#!/usr/bin/env python3
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import CompressedImage
from camera.slidewindow import SlideWindow
from camera.Preprocess import Preprocess
from control.pidcal import PidCal


class DynamicObstacle:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.preprocess = Preprocess()
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()

        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)

        self.scan_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()

        self.speed = 1500
        self.steer = 0.5

    def lidar_callback(self, msg):
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi

        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)] 

        obstacle_degrees = []

        for index, value in enumerate(self.scan_msg.ranges):
            if (-40 < degrees[index] <= 5 and 0 < value < 1.25) or (-5 <= degrees[index] < 70 and 0 < value < 1.5):
                obstacle_degrees.append(degrees[index])

        if len(obstacle_degrees) != 0:
            self.speed = 0
        else: 
            self.speed = 1500


    def cam_callback(self, data):
        img_bgr = cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        steering = abs(pid - 0.5)
        self.steer = steering

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img, "R")
        slideing_img, x_location, _ = self.slidewindow.slidewindow(img, "R")
        return slideing_img, x_location
        
def main():
    try:
        dynamic_obstacle = DynamicObstacle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()
