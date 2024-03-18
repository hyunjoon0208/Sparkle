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
from camera.warper import Warper

class StaticObstacle:
    def __init__(self):
        # print("StaticObstacle init")
        self.yaw = 0
        self.straight_yaw = 0
        
        self.lane_steer = 0.5 # camera steer
        self.front_obstacle_distance = 0
        self.front_obstacle_detected = False
        self.front_right_obstacle_detected = False
        self.front_left_obstacle_detected = False
        self.back_right_obstacle_detected = False
        self.back_obstacle_detected = False
        self.narrow_front_obstacle_detected = False
        self.narrow_front2_obstacle_detected = False
        self.narrow_front3_obstacle_detected = False
        self.far_front_obstacle_detected = False
        self.lane_detetced = False
        self.avoid_state = -1
        self.steer = 0.5
        self.speed = 1200
        self.one_line = False

        self.line_flag = "R"
        self.yellow_lane_detected = False

        self.preprocess = Preprocess()
        self.slidewindow = SlideWindow()
        self.pidcal = PidCal()
        self.warper = Warper()

        self.IMU_sub = rospy.Subscriber('/imu', Imu, self.yaw_callback)
        self.Lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)

        # self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)   # FOR TEST!!!!!!!!!!!
        # self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)# FOR TEST!!!!!!!!!!!
    def set_straight_yaw(self, yaw):
        self.straight_yaw = yaw

    def yaw_callback(self, msg):
        self.yaw = msg.orientation.z
        if self.straight_yaw == -0.7: # dynamic -> static
            if self.yaw > 0:
                self.yaw = -self.yaw


    def lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / pi
        degree_increment = self.scan.angle_increment * 180 / pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        far_front_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 6 < self.scan.ranges[i] < 10 and abs(self.degrees[i]) < 2]
        if far_front_obstacle_degrees:
            self.far_front_obstacle_detected = True
        else:
            self.far_front_obstacle_detected = False

        # narrow_front_obstacle
        narrow_front_obstacle_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if 1.3 < self.scan.ranges[i] < 1.4 and abs(self.degrees[i]) < 3]
        narrow_front_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 1.3 < self.scan.ranges[i] < 1.4 and abs(self.degrees[i]) < 3]
        if narrow_front_obstacle_degrees:
            self.narrow_front_obstacle_detected = True
        else:
            self.narrow_front_obstacle_detected = False


        # narrow_front2_obstacle
        narrow_front2_obstacle_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if 1.2 < self.scan.ranges[i] < 1.4  and abs(self.degrees[i]) < 10]
        narrow_front2_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 1.2 < self.scan.ranges[i] < 1.4 and abs(self.degrees[i]) < 10]
        # narrow_front2_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 1.2 < self.scan.ranges[i] < 1.4 and abs(self.degrees[i]) < 20]
        if narrow_front2_obstacle_degrees:
            self.narrow_front2_obstacle_detected = True
        else:
            self.narrow_front2_obstacle_detected = False


        # # narrow_front3_obstacle
        # narrow_front3_obstacle_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if 0 < self.scan.ranges[i] < 1.4  and abs(self.degrees[i]) < 25]
        # narrow_front3_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 0 < self.scan.ranges[i] < 1.4 and abs(self.degrees[i]) < 25]
        # if narrow_front3_obstacle_degrees:
        #     self.narrow_front3_obstacle_detected = True
        # else:
        #     self.narrow_front3_obstacle_detected = False

        # narrow_front3_obstacle
        narrow_front3_obstacle_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if 0 < self.scan.ranges[i] < 1.4  and 0 < self.degrees[i] < 5]
        narrow_front3_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 0 < self.scan.ranges[i] < 1.4 and 0 < self.degrees[i] < 5]
        if narrow_front3_obstacle_degrees:
            self.narrow_front3_obstacle_detected = True
        else:
            self.narrow_front3_obstacle_detected = False

        # front_obstacle
        front_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] <= 1.3 and abs(self.degrees[i]) < 20]
        if front_obstacle_degrees:
            self.front_obstacle_detected = True
        else:
            self.front_obstacle_detected = False
        front_obstacle_distance_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1.25 and abs(self.degrees[i]) < 20]

        if len(front_obstacle_distance_arr) == 0:
            self.front_obstacle_distance = -1
        else:
            self.front_obstacle_distance = min(front_obstacle_distance_arr)
        
        print("front_obstacle_distance", front_obstacle_distance_arr)
        print("narrow_front_obstacle_distance", narrow_front_obstacle_arr)
        print("narrow_front2_obstacle_distance", narrow_front2_obstacle_arr)
        print("narrow_front2_obstacle_degrees", narrow_front2_obstacle_degrees)
        print("narrow_front3_obstacle_distance", narrow_front3_obstacle_arr)
        print("narrow_front3_obstacle_degrees", narrow_front3_obstacle_degrees)
        # print("yaw: ", self.yaw, "straight_yaw - yaw", abs(self.straight_yaw - self.yaw))


        # front_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -70 < self.degrees[i] < -60]
        # front_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 1 and -65 < self.degrees[i] < -55]
        front_right_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.8 and -65 < self.degrees[i] < -55]
        if front_right_obstacle_degrees:
            self.front_right_obstacle_detected = True
        else:
            self.front_right_obstacle_detected = False

        # plus
        front_left_obstacle_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.7 and 15 < self.degrees[i] < 20]
        if front_left_obstacle_degrees:
            self.front_left_obstacle_detected = True
        else:
            self.front_left_obstacle_detected = False




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
 
    def cam_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        warper = self.warper.warp(img_bgr)

        # cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            self.lane_detetced = True
            # cv2.imshow("Slidinw window", slideing_img)
        else:
            self.lane_detetced = False
        
        
        pid = self.pidcal.pid_control(x_location)
        if self.line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            steering = abs(pid - 0.5)
            self.lane_steer = steering
            # find yellow line
            
            img = self.preprocess.find_yellow(warper)
            # cv2.imshow('YELLOW', img)
            if img is None:
                self.yellow_lane_detected = False
            else:
                nonzero = img.nonzero()[0]
                # print("YELLOW NONZERO :",nonzero)
                if len(nonzero) > 2:
                    self.yellow_lane_detected = True
                else:
                    self.yellow_lane_detected = False
        cv2.waitKey(1)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img, self.line_flag)
        slideing_img, x_location, _ = self.slidewindow.slidewindow(img, self.line_flag)
        return slideing_img, x_location
    
    def avoid(self):
        # print("self.avoid_state", self.avoid_state)
        # print("Line flag", self.line_flag)
        # self.steer_pub.publish(self.steer)
        # self.speed_pub.publish(self.speed)
        print("avoid_state: ", self.avoid_state)

        if self.avoid_state == -1:
            print("self.one_line : ", self.one_line)
            print("self.narrow_front_obstacle_detected : ", self.narrow_front_obstacle_detected)
            print("yaw: ", self.yaw, "straight_yaw: ", self.straight_yaw, "abs(self.straight_yaw - self.yaw)", abs(self.straight_yaw - self.yaw))


            #self.one_line == True : 장애물이 내앞에 없음. 그냥 직진하면됨. -> avoid_state = -1로 가야함. 1.3에서 변경됨.
            #self.one_line == False: default, 2차선에 있음.
            #self.one_line == False 
            #self.narrow_front_obstacle_detected == True : 1.3 ~ 1.4 사이에서 장애물이 내 딱 앞에 있을때만 
            #self.narrow_front_obstacle_detected == False : default


            if self.one_line == True:
                if (self.narrow_front2_obstacle_detected == True or self.narrow_front3_obstacle_detected == True) and abs(self.straight_yaw - self.yaw) <= 0.1:
                    self.avoid_state = 0
                    self.steer = 0.3
                    self.one_line = False
                # elif self.narrow_front3_obstacle_detected == True and 0.33 <= abs(self.straight_yaw - self.yaw) < 0.45:
                #     self.avoid_state = 0
                #     self.steer = 0.35
                #     self.one_line = False
        
                else:
                    self.avoid_state = -1
                    self.steer = self.lane_steer
            else:
                if self.narrow_front_obstacle_detected == False and abs(self.straight_yaw - self.yaw) < 0.1: # 2차선에서 주행 중 1차선의 장애물이 탐지되지 않게
                    self.steer = self.lane_steer
                    self.avoid_state = -1
                    self.one_line = True
                
                else:
                    if self.front_obstacle_detected:
                        self.avoid_state = 0
                        self.steer = 0.3
            
            
        elif self.avoid_state == 0: # 정면(-20 ~ 20)에서 장애물 발견, 1차선으로 이동
            # self.speed = 900
            self.speed = 1200
            if 0 < self.front_obstacle_distance < 1.5: # 정면 장애물의 거리를 가지고 steer 계산
                self.steer = min(self.front_obstacle_distance * self.front_obstacle_distance * 0.15, 0.3)
                # print("1: ", self.steer, "distance: ", self.front_obstacle_distance)
            if self.straight_yaw - self.yaw > 0.1:
                self.line_flag = "L"
                if self.yellow_lane_detected and self.lane_detetced:  
                    # print("2, self.yellow_lane_detected and self.lane_detetced")
                    self.steer = self.lane_steer
                elif abs(self.straight_yaw - self.yaw) > 0.13:
                    # print("3", "abs(self.straight_yaw - self.yaw)", abs(self.straight_yaw - self.yaw))
                    # self.steer = 0.35
                    self.steer = 0.4
            if self.front_right_obstacle_detected:
                print("4")
                self.avoid_state = 1
        elif self.avoid_state == 1: # 정면 우측(-60 ~ -50)에서 장애물 발견, 1차선에서 차선 맞추는 중
            # print("5, abs(self.straight_yaw - self.yaw)", abs(self.straight_yaw - self.yaw))
            if abs(self.straight_yaw - self.yaw) > 0.1:
                self.steer = 1
            else:
                self.steer = 0.8 # 0.7
                self.avoid_state = 2
        elif self.avoid_state == 2: # 후면 우측(-110 ~ -100)에서 장애물 발견, 1차선에서 2차선으로 이동
            if self.back_right_obstacle_detected:
                self.steer = 0.9
            if abs(self.straight_yaw - self.yaw) > 0.08: # 너무 확 들어가지 않게 yaw 값을 가지고 조정, 너무 확 들어가면 카메라로 차선 인식을 진행해도 차선에서 벗어남
                self.line_flag = "R"
                self.avoid_state = 3
        elif self.avoid_state == 3: # 2차선에서 차선 맞추는 중
            self.steer = 0.2
            if self.lane_detetced: # 카메라로 차선 인식이 된 경우 PID 제어를 통해 차선 맞춤
                self.speed = 1200
                self.avoid_state = -1

            if self.far_front_obstacle_detected == False and abs(self.straight_yaw - self.yaw) <= 0.05:
                self.avoid_state = 4

        elif self.avoid_state == 4:
            self.speed = 2400
            self.steer = self.lane_steer

            if self.far_front_obstacle_detected == True:
                self.avoid_state = -1
        

if __name__ == '__main__':
    try:
        static_obstacle = StaticObstacle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass