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
from camera.slidewindow_test import SlideWindow
from camera.Preprocess import Preprocess
from control.pidcal import PidCal

class StaticObstacle:
    def __init__(self):
        self.yaw = 0
        self.straight_yaw = 0
        self.steer = 0.5
        self.lane_steer = 0.5 # camera steer
        self.front_obstacle_distance = 0
        self.front_obstacle_detected = False
        self.front_right_obstacle_detected = False
        self.back_right_obstacle_detected = False
        self.back_obstacle_detected = False
        self.narrow_front_obstacle_detected = False
        self.lane_detetced = False
        self.avoid_state = -1
        self.speed = 1400

        self.line_flag = "R"
        self.yellow_lane_detected = False

        self.preprocess = Preprocess()
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()

        rospy.init_node('static_obstacle', anonymous=True)
        
        self.IMU_sub = rospy.Subscriber('/imu', Imu, self.yaw_callback)
        self.Lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)

        
        self.speed_pub = rospy.Publisher('/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/steer', Float64, queue_size=1)
        
    def yaw_callback(self, msg):
        self.yaw = msg.orientation.z

    def lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / pi
        degree_increment = self.scan.angle_increment * 180 / pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        narrow_front_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1.2 and abs(self.degrees[i]) < 1]
        if narrow_front_obstacle_degrees:
            self.narrow_front_obstacle_detected = True
        else:
            self.narrow_front_obstacle_detected = False

        front_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1.2 and abs(self.degrees[i]) < 20]
        if front_obstacle_degrees:
            self.front_obstacle_detected = True
        else:
            self.front_obstacle_detected = False
        front_obstacle_distance_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1.2 and abs(self.degrees[i]) < 20]
        if len(front_obstacle_distance_arr) == 0:
            self.front_obstacle_distance = -1
        else:
            self.front_obstacle_distance = min(front_obstacle_distance_arr)

        # front_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -70 < self.degrees[i] < -60]
        front_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -65 < self.degrees[i] < -55]
        if front_right_obstacle_degrees:
            self.front_right_obstacle_detected = True
        else:
            self.front_right_obstacle_detected = False

        back_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.5 and -110 < self.degrees[i] < -100]
        if back_right_obstacle_degrees:
            self.back_right_obstacle_detected = True
        else:
            self.back_right_obstacle_detected = False   

        back_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -110 < self.degrees[i] < -100]
        if back_obstacle_degrees:
            self.back_obstacle_detected = True
        else:
            self.back_obstacle_detected = False   

        self.avoid()
        self.speed_pub.publish(self.speed)
        self.steer_pub.publish(self.steer)
 
    def cam_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        # cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            self.lane_detetced = True
            # cv2.imshow("Slidinw window", slideing_img)
        else:
            self.lane_detetced = False
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        if self.line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            steering = abs(pid - 0.5)
            self.lane_steer = steering
            # find yellow line
            try:
                img = self.preprocess.find_yellow(img_bgr)
                nonzero = img.nonzero()
                nonzeroy = np.array(nonzero[0])
                nonzerox = np.array(nonzero[1])
                center_left_x = 0
                center_right_x = 640
                center_high_y = 480
                center_low_y = 0
                yellow_lane = ((nonzerox >= center_left_x) & (nonzerox <= center_right_x) & (nonzeroy <= center_high_y) & (nonzeroy >= center_low_y)).nonzero()[0]
                if len(yellow_lane) > 5000:
                    self.yellow_lane_detected = True
                else:
                    self.yellow_lane_detected = False
            except Exception as e:
                print(e)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
        slideing_img, x_location, _ = self.slidewindow.slidewindow(img, self.line_flag)
        return slideing_img, x_location
    
    def avoid(self):
        if self.avoid_state == -1:
            if self.front_obstacle_detected:
                self.avoid_state = 0
                self.steer = 0.3
            else:
                self.steer = self.lane_steer
            if not self.narrow_front_obstacle_detected and abs(self.yaw - self.straight_yaw) < 0.2: # 2차선에서 주행 중 1차선의 장애물이 탐지되지 않게
                self.steer = self.lane_steer
                self.avoid_state = -1
        elif self.avoid_state == 0: # 정면(-20 ~ 20)에서 장애물 발견, 1차선으로 이동
            self.speed = 900
            if 0 < self.front_obstacle_distance < 1.5: # 정면 장애물의 거리를 가지고 steer 계산
                self.steer = min(self.front_obstacle_distance * self.front_obstacle_distance * 0.13, 0.3)
            if self.straight_yaw - self.yaw > 0.1:
                self.line_flag = "L"
                if self.yellow_lane_detected and self.lane_detetced:  
                    self.steer = self.lane_steer
                elif self.straight_yaw - self.yaw > 0.13:
                    self.steer = 0.2
            if self.front_right_obstacle_detected:
                self.avoid_state = 1
        elif self.avoid_state == 1: # 정면 우측(-60 ~ -50)에서 장애물 발견, 1차선에서 차선 맞추는 중
            if abs(self.yaw - self.straight_yaw) > 0.1:
                self.steer = 1
            else:
                self.steer = 0.7
                self.avoid_state = 2
        elif self.avoid_state == 2: # 후면 우측(-110 ~ -100)에서 장애물 발견, 1차선에서 2차선으로 이동
            if self.back_right_obstacle_detected:
                self.steer = 0.9
            if abs(self.yaw - self.straight_yaw) > 0.1: # 너무 확 들어가지 않게 yaw 값을 가지고 조정, 너무 확 들어가면 카메라로 차선 인식을 진행해도 차선에서 벗어남
                self.line_flag = "R"
                self.avoid_state = 3
        elif self.avoid_state == 3: # 2차선에서 차선 맞추는 중
            self.steer = 0.2
            if self.lane_detetced: # 카메라로 차선 인식이 된 경우 PID 제어를 통해 차선 맞춤
                self.speed = 1000
                self.avoid_state = -1
        

if __name__ == '__main__':
    try:
        static_obstacle = StaticObstacle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
