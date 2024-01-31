#!/usr/bin/env phyton3
#-*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float64, Int16
from math import *
import numpy as np
# from CurveDetector import CurveDetector
from control.yaw_drive_copy import YawDrive
import cv2
import numpy as np

class DynamicObstacle:
    def __init__(self):
        rospy.init_node("dynamic_obstacle")
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.speed_pub = rospy.Publisher('/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/steer', Float64, queue_size=1)

        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)

        self.scan_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()

        self.speed = 2400
        self.steer = 0.5

    def lidar_callback(self, msg):
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi

        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)] 

        for index, value in enumerate(self.scan_msg.ranges):
            if abs(degrees[index]) < 50 and 0 < value < 1.6:
                self.speed = 0
            else:
                self.speed = 2400

        self.speed_pub.publish(self.speed)
        self.steer_pub.publish(self.steer)


    def cam_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            cv2.imshow("Slidinw window", slideing_img)
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        if self.line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            steering = abs(pid - 0.5)
            self.lane_steer = steering

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
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
