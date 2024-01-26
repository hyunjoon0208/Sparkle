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
import morai_msgs.msg as CtrlCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from camera.slidewindow_test import SlideWindow
from camera.Preprocess import Preprocess
from control.pidcal import PidCal
class YawDrive:
    def __init__(self):
        self.yaw = 0
        self.straight_yaw = -1.0
        self.steer = 0.5
        self.current_lane = 1
        self.obstacle_detected = False
        self.side_obstacle_detected = False
        self.sideback_obstacle_detected = False
        self.back_obstacle_detected = False
        self.avoid_state = 0
        self.speed = 1000
        rospy.init_node('yaw_drive', anonymous=True)
        self.preprocess = Preprocess()
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()
        self.lane_steer = 0.5

        
        
        self.IMU_sub = rospy.Subscriber('/imu', Imu, self.yaw_callback)
        self.Lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)

        
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        
    def yaw_callback(self, msg):
        self.yaw = msg.orientation.z

    def lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / pi
        degree_max = self.scan.angle_max * 180 / pi
        degree_increment = self.scan.angle_increment * 180 / pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        # Check for obstacles in the front
        obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.9 and abs(self.degrees[i]) < 20]

        if obstacle_degrees:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

        # Check for obstacles in the side
        side_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 40 < abs(self.degrees[i]) < 50 and self.scan.ranges[i] < 1]

        if side_obstacle_degrees:
            self.side_obstacle_detected = True
        else:
            self.side_obstacle_detected = False
        # Check for obstacles in the sideback
        sideback_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 50 < abs(self.degrees[i]) < 110 and self.scan.ranges[i] < 1]

        if sideback_obstacle_degrees:
            self.sideback_obstacle_detected = True
        else:
            self.sideback_obstacle_detected = False
        # Check for obstacles in the back
        back_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 165 < abs(self.degrees[i]) <= 180 and self.scan.ranges[i] < 2]

        if back_obstacle_degrees:
            self.back_obstacle_detected = True
        else:
            self.back_obstacle_detected = False
        # self.avoid()
        if self.obstacle_detected or self.avoid_state != 0:
            self.speed = 1000
            self.avoid()
            
        else:
            self.steer = self.lane_steer
            self.speed = 2400
        self.speed_pub.publish(self.speed)
        self.steer_pub.publish(self.steer)

    def cam_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location,line_flag = self.lane_detection(img_bgr)
        cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            cv2.imshow("Slidinw window", slideing_img)
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        if line_flag == 'R':
            print("right")
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            print("left")
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        # print('')
            
    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
        slideing_img, x_location,line_flag = self.slidewindow.slidewindow(img,'L')
        return slideing_img, x_location,line_flag

    def avoid(self):
        print('self.avoid_state : ', self.avoid_state)
        print('self.obstacle_detected : ', self.obstacle_detected)
        print('self.side_obstacle_detected : ', self.side_obstacle_detected)
        print('self.sideback_obstacle_detected : ', self.sideback_obstacle_detected)
        print('self.back_obstacle_detected : ', self.back_obstacle_detected)
        print('steer : ', self.steer)
        if self.obstacle_detected:
            if self.avoid_state == 0:
                # self.straight_yaw = round(self.yaw,4)
                self.steer = 0
                self.avoid_state = 1
        elif self.avoid_state == 1:
            self.avoid_state = 2
        elif self.avoid_state == 2:
            if abs(self.yaw - self.straight_yaw) > 0.1:
                print('steer gap : ', abs(self.yaw - self.straight_yaw))
                self.steer = min(self.steer+0.3, 1)
            else:
                self.steer = 0.5
                self.avoid_state = 3
        if self.side_obstacle_detected and self.avoid_state == 3:
            self.steer = 0.5
            # self.straight_yaw = round(self.yaw,4)
        elif self.avoid_state == 3 and self.sideback_obstacle_detected:
            self.steer = 1
        if not self.sideback_obstacle_detected and self.avoid_state == 3:
            self.avoid_state = 4
        elif self.avoid_state == 4:
            error = round(self.yaw,2) - self.straight_yaw
            if abs(error) > 0.001:
                if error > 0:
                    self.steer = 0
                else:
                    self.steer = 1
                print('steer gap : ', abs(self.yaw - self.straight_yaw))
                # self.steer = max(self.steer-0.3, 0)
            else:
                self.avoid_state = 5
                self.steer = 0.5
                self.speed = 1000
        elif self.back_obstacle_detected and self.avoid_state == 5:
            self.avoid_state = 0
            


if __name__ == '__main__':
    try:
        yaw_drive = YawDrive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

