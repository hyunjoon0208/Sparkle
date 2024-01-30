#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, os, sys, rospy, tf, cv2
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float64, Bool
from morai_msgs.msg import CtrlCmd,GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage, Imu
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np
from camera.StopDetector import stop_line
from camera.CurveDetector import CurveDetector
from camera.Rotary import Rotary
from camera.slidewindow import SlideWindow
from camera.Preprocess import Preprocess
from control.pidcal import PidCal

class main_drive:
    def __init__(self):

        #class
        self.curve_detector = CurveDetector()
        self.Rotary = None
        self.stop_line = stop_line()
        self.slidewindow = SlideWindow()
        self.preprocess = Preprocess()
        self.pidcal = PidCal()

        #numeric variable
        self.flag = 0
        self.curve_counter = 0
        self.line_flag = 'R'
        self.yaw = 0

        #image variable
        self.img = None

        #boolean variable
        self.is_corner = False
        self.is_obstacle_static = False
        self.is_obstacle_dynamic = False
        self.is_left_turn = False
        self.slam_end_flag = True

        # subscriber
        self.img_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_steer_callback, queue_size=1)
        self.obstacle_sub = rospy.Subscriber("/obstacle_detector", Float64, self.obstacle_callback, queue_size=1)
        self.trafiic_light_sub = rospy.Subscriber('GetTrafficLightStatus', GetTrafficLightStatus, self.traffic_light_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.Imu_callback, queue_size=1)
        # rospy.Subscriber('/slam_end_flag', Bool, self.slam_end_callback, queue_size=1)
        # publisher
        # self.steer_pub = rospy.Publisher('/steer', Float64, queue_size=1)
        # self.speed_pub = rospy.Publisher('/speed', Float64, queue_size=1)
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        

        # 제어값
        self.lane_steer = 0.0
        self.obstacle_steer = 0.0

        self.steer = 0.0#publish용
        self.speed = 2400

    def timer_callback(self, _event):
        try :
            self.drive()
        except :
            pass
    # callback
    def cam_steer_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        
        cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            self.lane_detetced = True
            cv2.imshow("Slidinw window", slideing_img)
        else:
            self.lane_detetced = False
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        if self.line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        elif self.line_flag == 'L':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        elif self.line_flag == 'CL':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
            if steering < 0.5:
                steering *= 1.2
            
    def Imu_callback(self, msg):
        self.yaw = abs(msg.orientation.z)
    def obstacle_callback(self, msg):
        self.obstacle_steer = msg.data
    def traffic_light_callback(self, msg):
        try:
            if msg.traffic_light_status == 33:
                self.is_left_turn = True
            else:
                self.is_left_turn = False
        except:
            pass
    
    def lane_detection(self, img):
        self.img = self.preprocess.preprocess(img, self.line_flag)
        slideimg, x_location , self.line_flag = self.slidewindow.slidewindow(self.img,self.line_flag)
        return slideimg, x_location
        
    # main drive
    def drive(self):
        rotary_state = 0
        # while not rospy.is_shutdown():
        # print('img len : ', len(self.img))
        print('flag : ', self.flag)
        print('curve_flag : ', self.curve_detector.curve_flag)
        print('yaw : ', self.yaw)
        # print('slam_end_flag : ', self.slam_end_flag)
        # CURVE-DETECT는 매 iteration마다 실행되어야함
        self.curve_detector.curve_detector(self.yaw)
        # slam 종료신호 받아야함!
        # SLAM-END-FLAG 선언

        if self.flag == 0 and self.slam_end_flag == True:# 출발 
            self.flag = 1





        elif self.flag == 1: # 장애물 감지 및 회피
            # self.steer = self.obstacle_steer
            self.steer = self.lane_steer # for test!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
            if self.curve_detector.curve_flag == 2:
                self.flag = 2





        elif self.flag == 2: # 좌회전(교차로, 로터리진입전)
            self.line_flag = 'CL'
            self.steer = self.lane_steer
            if (0.68<self.yaw<0.77):
                self.line_flag = 'R'
                self.speed = 600
                self.flag = 3

        
        elif self.flag == 3: # 로터리 정지선 감지
            if self.stop_line.isStop():
                self.Rotary = Rotary()
                self.flag = 4
        
        elif self.flag == 4: # 로터리 진입
            if not (rotary_state == 2 and self.Rotary.front_car_detected == False):
                self.steer, self.speed, rotary_state = self.Rotary.run()
            if rotary_state == 4:
                self.line_flag = 'L'
                self.steer = self.lane_steer

            else:
                self.steer = self.lane_steer
            if rotary_state == 6:
                self.flag = 5
        
        # elif self.flag == 5: # 로터리 탈출
        #     distance_of_front_car = self.car_detector.adcc()
        #     if distance_of_front_car > 10:
        #         self.speed = 30 # 달려
        #     elif distance_of_front_car > 5:
        #         self.speed -=1
        #     else:
        #         self.speed = 0

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
                self.speed = 0
        
        elif self.flag == 10: # 정지
            return
        
        self.steer_pub.publish(self.steer)
        self.speed_pub.publish(self.speed)
            

def run():
    rospy.init_node('main_drive', anonymous=True) 
    Main_drive = main_drive()
    rospy.Timer(rospy.Duration(1.0/30.0), Main_drive.timer_callback)
    rospy.spin()
    
if __name__ == '__main__':
    run()