#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys,os
import rospy
import rospkg
import math
import time
import decimal
from morai_msgs.srv import MoraiEventCmdSrv, MoraiEventCmdSrvRequest
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray,Bool
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from math import cos,sin,sqrt,pow,atan2,pi
import time
import numpy as np
from StopDetector import StopDetector
from CurveDetector import CurveDetector
from CarDetector import CarDetector

class main_drive:
    def __init__(self):
        rospy.init_node('main_drive', anonymous=True)
        rospy.wait_for_service("/Service_MoraiEventCmd")
        self.service_client = rospy.ServiceProxy("/Service_MoraiEventCmd",MoraiEventCmdSrv)
        self.request_srv = MoraiEventCmdSrvRequest()
        rate = rospy.Rate(10) # 10hz
        self.flag = 0
        self.stop_line = StopDetector()
        self.curve_detector = CurveDetector(90, 0, 10, 10, 100)
        self.car_detector = CarDetector()
        self.is_corner = False
        self.is_obstacle_static = False
        self.is_obstacle_dynamic = False
        # self.now_velocity = 0
        self.curve_counter = 0
        # subscriber
        rospy.Subscriber("/lane_detection", Float64, self.cam_steer_callback, queue_size=1)
        rospy.Subscriber("/commands/vel", CtrlCmd, self.velocity_callback, queue_size=1)
        rospy.Subscriber("/obstacle_detector", Float64, self.obstacle_callback, queue_size=1)
        # 제어값
        self.cam_steer = 0.0
        self.obstacle_steer = 0.0

        self.steer = 0.0#publish용
        self.velocity = 0

        # class
        #lanedetector
        #obstacledetector
        #trafficlight
    # callback
    def cam_steer_callback(self, msg):
        self.cam_steer = msg.data
    def velocity_callback(self, msg):
        self.velocity = msg.data
    def obstacle_callback(self, msg):
        self.obstacle_steer = msg.data

    def drive(self):
        while not rospy.is_shutdown():
            
            self.steer = self.lane_detector.steer
            if self.flag == 0 and self.stop_line.isStop():# 출발
                self.flag = 1
            elif self.flag == 1: # 장애물 감지 및 회피
                self.steer = self.obstacle_steer
                if CurveDetector.detectCurves():
                    self.curve_counter += 1
                if self.curve_counter == 2:
                    self.flag = 2

            elif self.flag == 2: # 좌회전(교차로, 로터리진입전)
                self.steer = self.cam_steer
                if self.is_corner:
                    self.flag = 3
            
            elif self.flag == 3: # 로터리 정지선 감지
                if self.stop_line.isStop():
                    self.flag = 4
            
            elif self.flag == 4: # 로터리 진입
                if self.car_detector.detect():
                    self.flag = 5
            
            elif self.flag == 5: # 로터리 탈출
                distance_of_front_car = self.car_detector.adcc()
                if distance_of_front_car > 10:
                    self.velocity = 30 # 달려
                elif distance_of_front_car > 5:
                    self.velocity -=1
                else:
                    self.velocity = 0

            elif self.flag == 6: # 차선변경
                # 차선변경
                self.flag = 7
                pass
            elif self.flag == 7: #정지선 검출
                self.flag = 8
                pass

            elif self.flag == 8: #신호받고 좌회전
                self.flag = 9
                pass
            elif self.flag == 9: # 차선변경
                self.flag = 10
                pass
            elif self.flag == 10: # 달려
                if self.stop_line.isStop():
                    self.flag = 11
                    first = time.time()
                pass
            elif self.flag == 11: # 정지선 검출 
                if time.time() - first > 2:
                    self.velocity = 0
                pass