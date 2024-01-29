#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, os, sys, rospy, tf, cv2
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float64
from morai_msgs.msg import CtrlCmd,GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np
from camera.StopDetector import stop_line
from camera.CurveDetector import CurveDetector
from camera.Rotary import Rotary
from camera.slidewindow import SlideWindow
from callback import cam_steer_callback, velocity_callback, obstacle_callback, traffic_light_callback

class main_drive:
    def __init__(self):
        rospy.init_node('main_drive', anonymous=True)
        # rospy.wait_for_service("/Service_MoraiEventCmd")
        # self.service_client = rospy.ServiceProxy("/Service_MoraiEventCmd",MoraiEventCmdSrv)
        # self.request_srv = MoraiEventCmdSrvRequest() # 지울예정

        #class
        self.curve_detector = CurveDetector(90, 0, 10, 10, 100)
        self.Rotary = None
        self.stop_line = stop_line()


        #numeric variable
        self.flag = 0
        self.curve_counter = 0

        #boolean variable
        self.is_corner = False
        self.is_obstacle_static = False
        self.is_obstacle_dynamic = False
        self.is_left_turn = False
        # subscriber
        rospy.Subscriber("/lane_detection", Float64, self.cam_steer_callback, queue_size=1)
        rospy.Subscriber("/commands/vel", CtrlCmd, self.velocity_callback, queue_size=1)
        rospy.Subscriber("/obstacle_detector", Float64, self.obstacle_callback, queue_size=1)
        rospy.Subscriber('GetTrafficLightStatus', GetTrafficLightStatus, self.traffic_light_callback, queue_size=1)
        # publisher
        self.steer_pub = rospy.Publisher('/steer', Float64, queue_size=1)
        self.speed_pub = rospy.Publisher('/speed', Float64, queue_size=1)


        # 제어값
        self.lane_steer = 0.0
        self.obstacle_steer = 0.0

        self.steer = 0.0#publish용
        self.velocity = 0

    # callback
        self.cam_steer_callback = cam_steer_callback()
        self.velocity_callback = velocity_callback()
        self.obstacle_callback = obstacle_callback()
        self.traffic_light_callback = traffic_light_callback()

    # main drive
    def drive(self):
        rotary_state = 0
        while not rospy.is_shutdown():
            if self.flag == 0 and self.stop_line.isStop():# 출발
                self.flag = 1
            elif self.flag == 1: # 장애물 감지 및 회피
                self.steer = self.obstacle_steer
                if CurveDetector.detectCurves():
                    self.curve_counter += 1
                if self.curve_counter == 2:
                    self.flag = 2

            elif self.flag == 2: # 좌회전(교차로, 로터리진입전)
                self.steer = self.lane_steer
                if self.is_corner:
                    self.flag = 3
            
            elif self.flag == 3: # 로터리 정지선 감지
                if self.stop_line.isStop():
                    self.Rotary = Rotary()
                    self.flag = 4
            
            elif self.flag == 4: # 로터리 진입
                if not (rotary_state == 2 and self.Rotary.front_car_detected == False):
                    self.steer, self.velocity, rotary_state = self.Rotary.run()
                else:
                    self.steer = self.lane_steer
                if rotary_state == 6:
                    self.flag = 5
            
            # elif self.flag == 5: # 로터리 탈출
            #     distance_of_front_car = self.car_detector.adcc()
            #     if distance_of_front_car > 10:
            #         self.velocity = 30 # 달려
            #     elif distance_of_front_car > 5:
            #         self.velocity -=1
            #     else:
            #         self.velocity = 0

            # elif self.flag == 6: # 차선변경
            #     # 차선변경
            #     self.flag = 7
            #     pass
            # elif self.flag == 7: #정지선 검출
            #     self.flag = 8
            #     pass

            elif self.flag == 5: #신호받고 좌회전
                if self.is_left_turn:
                    self.Rotary.__del__()
                    # self.flag = 6
                    pass
                self.flag = 6

            elif self.flag == 6: # 차선변경
                self.flag = 7
                pass
            elif self.flag == 8: # 달려
                if self.stop_line.isStop():
                    self.flag = 9
                    first = time.time()
                pass
            elif self.flag == 9: # 정지선 검출 
                if self.stop_line.isStop():
                    self.flag = 10
                    first = time.time()
                if time.time() - first > 2:
                    self.velocity = 0
            
            elif self.flag == 10: # 정지
                return
            
            self.steer_pub.publish(self.steer)
            self.speed_pub.publish(self.velocity)
            

if __name__ == '__main__':
    try:
        main_drive = main_drive()
        main_drive.drive()
    except rospy.ROSInterruptException:
        pass