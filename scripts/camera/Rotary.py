#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, cv2, sys, os, time
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import numpy as np
import morai_msgs.msg as CtrlCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage, Imu, LaserScan
from cv_bridge import CvBridge
from .Preprocess import Preprocess
from .StopDetector import stop_line
from .slidewindow import SlideWindow
from control.pidcal import PidCal

class Rotary:
    def __init__(self) -> None:
        # rospy.init_node('rotary', anonymous=True)

        self.obstacle_detected = True
        self.front_car_detected = True
        self.center_detected = True
        self.yellow_lane_detected = False
        self.straight_stop = False
        self.straight_time = 0
        self.state = 0
        self.speed = 1000
        self.steer = 0.5
        self.exit_yaw = 0.90
        self.straight_yaw = -1.0
        self.yaw = 0
        self.preprocess = Preprocess()
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()
        self.Imu_sub = rospy.Subscriber('/imu', Imu, self.Imu_callback, queue_size=1)
        self.Lidar_sub = rospy.Subscriber('/scan', LaserScan, self.Lidar_callback, queue_size=1)
        self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_callback, queue_size=1)

        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.degrees = []
        self.lane_steer = None
        self.line_flag = 'R'
        self.img = None
        self.left_lane_detected = False

    def img_callback(self, data):
        self.img = cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        try:
            pid = self.pidcal.pid_control(self.slidewindow.slidewindow(self.preprocess.preprocess(self.img),'L'))
        except:
            pid = None
        if pid is None:
            self.left_lane_detected = False
        else:
            self.left_lane_detected = True
    def Imu_callback(self, msg):
        self.yaw = msg.orientation.z
    
    def Lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / np.pi
        degree_max = self.scan.angle_max * 180 / np.pi
        degree_increment = self.scan.angle_increment * 180 / np.pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        # Check for obstacles in the rotary
        obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -5 < self.degrees[i] < 70]
        if len(obstacle_degrees) > 0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        
        # Check for obstacles in the front
        front_car_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.4 and abs(self.degrees[i]) < 25]
        if len(front_car_degrees) > 0:
            self.front_car_detected = True
        else:
            self.front_car_detected = False
    
    def find_center(self):
        if self.img is None:
            return
        img = self.preprocess.preprocess(self.img)
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        center_left_x = 240
        center_right_x = 400
        center_high_y = 0
        center_low_y = 240
        
        center = ((nonzerox >= center_left_x) & (nonzerox <= center_right_x) & (nonzeroy >= center_high_y) & (nonzeroy <= center_low_y)).nonzero()[0]
        print("center_num : ", len(center))
        if len(center) > 5000 and self.front_car_detected == False:
            self.center_detected = True
        else:
            self.center_detected = False

    def find_yellow_lane(self):
        print('+++++++++++++++++++++++++++++++++++++++++++++++++++++++')
        img = self.preprocess.find_yellow(self.img)
        if img is None:
            return
        nonzero = img.nonzero()
        # nonzeroy = np.array(nonzero[0])
        # nonzerox = np.array(nonzero[1])
        # center_left_x = 240
        # center_right_x = 400
        # center_high_y = 280
        # center_low_y = 320

        # pts = np.array([[center_left_x,center_high_y],[center_left_x,center_low_y],[center_right_x, center_low_y],[center_right_x,center_high_y]],np.int32)
        # cv2.polylines(img, [pts], False, (0,255,0), 1)
        # cv2.imshow("yellow", img)
        # cv2.waitKey(1)
        # yellow_lane = ((nonzerox >= center_left_x) & (nonzerox <= center_right_x) & (nonzeroy >= center_high_y) & (nonzeroy <= center_low_y)).nonzero()[0]
        # print("yellow_lane_num : ", len(yellow_lane))
        print('nonzero : ', len(nonzero))
        if len(nonzero) > 2:
            self.yellow_lane_detected = True
        else:
            self.yellow_lane_detected = False

    
    def run(self):
        while not rospy.is_shutdown():
            self.speed_pub.publish(self.speed)
            self.steer_pub.publish(self.steer)
            print('state : ', self.state)
            if self.state <=1 and self.obstacle_detected == True:
                print('obstacle_detected')
                self.state = 0
                self.speed = 0
            elif self.state == 0:
                if self.obstacle_detected == False:
                    print('no obstacle_detected & Let\'s go')
                    self.state = 1
                    self.speed = 400
                    self.find_center()

                elif self.obstacle_detected == True:
                    self.speed = 0
                    self.state = 0
            if self.state == 1:
                self.find_center()
                if self.center_detected == True and self.obstacle_detected == False and self.front_car_detected == False:
                    self.speed = 0
                    self.state = 2
                else:
                    self.speed = 400
                    self.state = 1
                    self.steer = 0.5
                    self.find_center()

            if self.state == 2:
                if 0.915 <= abs(self.yaw) <= 0.925:
                    if self.front_car_detected == True:
                        self.speed = 0
                    else:
                        self.speed = 600
                        print('exit_yaw')
                        self.steer = 0.5
                        self.state = 3
                else:
                    self.speed = 600
                    error = round(self.yaw,3) - 0.919
                    if self.front_car_detected == True:
                        self.speed = 0
                    elif abs(error) > 0.005: 
                        if error < 0:
                            self.steer = min(1, self.steer * 1.1)
                            print('1111111111')
                        else:
                            print('22222222222')
                            self.steer = max(0, self.steer * 0.8)
                        print('steer : ', self.steer)
                    # self.find_center()

            if self.state ==3:
                if self.front_car_detected == True:
                        self.speed = 0
                self.find_yellow_lane()
                # if self.yellow_lane_detected == True and self.left_lane_detected == True:
                if self.yellow_lane_detected == True:
                    self.state = 4
                    break
            
        return self.state

