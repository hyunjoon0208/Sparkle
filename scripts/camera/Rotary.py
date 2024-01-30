#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, cv2, sys, os
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
        rospy.init_node('rotary', anonymous=True)

        self.obstacle_detected = True
        self.front_car_detected = True
        self.center_detected = True
        self.yellow_lane_detected = False
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
        self.stop_detector = stop_line()
        self.speed_pub = rospy.Publisher('/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/steer', Float64, queue_size=1)
        self.degrees = []
        self.lane_steer = 0.5
        self.line_flag = 'R'
        self.img = None

    def img_callback(self, data):
        self.img = cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        pid = self.pidcal.pid_cal(self.slidewindow.slidewindow(self.preprocess.preprocess(self.img),'R'))
        if self.line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            steering = abs(pid - 0.5)
            self.lane_steer = steering
    def Imu_callback(self, msg):
        self.yaw = msg.orientation.z
    
    def Lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / np.pi
        degree_max = self.scan.angle_max * 180 / np.pi
        degree_increment = self.scan.angle_increment * 180 / np.pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        # Check for obstacles in the rotary
        obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 0.5 < self.scan.ranges[i] < 1.5 and -10 < self.degrees[i]]
        if len(obstacle_degrees) > 0:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
        
        # Check for obstacles in the front
        front_car_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.5 and abs(self.degrees[i]) < 10]
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
        center_high_y = 200
        center_low_y = 320
        
        center = ((nonzerox >= center_left_x) & (nonzerox <= center_right_x) & (nonzeroy >= center_high_y) & (nonzeroy <= center_low_y)).nonzero()[0]
        print("center_num : ", len(center))
        if len(center) > 5000:
            self.center_detected = True
        else:
            self.center_detected = False

    def find_yellow_lane(self):
        img = self.preprocess.find_yellow(self.img)
        if img is None:
            return
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        center_left_x = 240
        center_right_x = 400
        center_high_y = 280
        center_low_y = 320
        cv2.imshow("yellow", img)
        cv2.waitKey(1)
        yellow_lane = ((nonzerox >= center_left_x) & (nonzerox <= center_right_x) & (nonzeroy >= center_high_y) & (nonzeroy <= center_low_y)).nonzero()[0]
        print("yellow_lane_num : ", len(yellow_lane))
        if len(yellow_lane) > 2000:
            self.yellow_lane_detected = True
        else:
            self.yellow_lane_detected = False

    
    def run(self):
        while not rospy.is_shutdown():
            print('state : ', self.state)
            if self.state <=1 and self.obstacle_detected == True:
                print('obstacle_detected')
                self.state = 0
            if self.state == 0 and self.obstacle_detected == False:
                print('no obstacle_detected & Let\'s go')
                self.state = 1
                self.find_center()
            if self.state == 1 and self.center_detected == False:
                print('center_not_detected')
                self.find_center()
                # self.state = 1
                
            if self.state == 1 and self.center_detected == True:
                print('center_detected & Let\'s go')
                self.state = 2
            
            # if self.state == 2 and 0.85<self.yaw<0.95:
            #     print('exit_yaw')
            #     self.state = 3
            #     # 

            # if self.state == 2 and self.front_car_detected == True:
            #     print('front_car_detected')
            #     # self.state = 2
            #     self.speed = 0
                
            # if self.state == 2 and self.front_car_detected == False:
            #     print('no front_car_detected & Let\'s go')
            #     print('lane detection')
            #     print('yaw : ', self.yaw)
            if self.state == 2 and self.front_car_detected == True:
                print('front_car_detected')
                self.speed = 0
            if self.state == 2 and self.front_car_detected == False:
                if 0.925 <= abs(self.yaw) <= 0.935:
                    print('exit_yaw')
                    self.state = 3
                    self.steer = 0.5
                else:
                    error = round(self.yaw,2) - 0.93
                    if abs(error) > 0.005: 
                        if error > 0:
                            self.steer = 0.25
                        else:
                            self.steer = 0.75
                        print('steer : ', self.steer)
            if self.state == 3:
                self.find_yellow_lane()
                if self.yellow_lane_detected == True:
                    self.state = 4
                    self.speed = 0
                    self.steer = 0.5
                    self.straight_yaw = self.yaw
                    print('straight_yaw : ', self.straight_yaw)
                    
            if self.state == 4 and self.yellow_lane_detected == False:
                #lane change
                # self.steer = 0.3
                self.find_yellow_lane()
                
            elif self.state == 4 and self.yellow_lane_detected == True:
                pass
                
            
            if self.state == 5:
                if self.stop_detector.isStop(self.img):
                    print('stop line detected')
                    self.speed = 0
                    self.state = 6
                    print('DONE ROTARY')
                    
            
            
        # return self.speed, self.steer, self.state


if __name__ == '__main__':
    try:
        print('ROTARY')
        rotary = Rotary()
        rotary.run()
    except rospy.ROSInterruptException:
        pass
