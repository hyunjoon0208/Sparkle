#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, os, sys, rospy, tf, cv2, time
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from std_msgs.msg import Float64, Bool, Int16
from morai_msgs.msg import CtrlCmd,GetTrafficLightStatus
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from math import pi
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import numpy as np
from camera.StopDetector import stop_line
from camera.CurveDetector import CurveDetector
from camera.rotary_test import Rotary
from camera.slidewindow import SlideWindow
from camera.Preprocess import Preprocess
from control.pidcal import PidCal
from control.static_obstacle import StaticObstacle
from control.dynamic_obstacle import DynamicObstacle


class main_drive:
    def __init__(self):

        #class
        self.curve_detector = CurveDetector()
        self.Rotary = Rotary()
        self.stop_line = stop_line()
        self.slidewindow = SlideWindow()
        self.preprocess = Preprocess()
        self.pidcal = PidCal()
        self.static_obstacle = None
        self.dynamic_obstacle = DynamicObstacle()

        #numeric variable
        self.flag = 5
        self.curve_counter = 0
        self.line_flag = 'R'
        self.yaw = 0
        self.obstacle_mode = 0
        self.prev_time = 0
        self.traffic_light_status = 0
        self.state = 0

        #image variable
        self.img = None

        #boolean variable
        self.is_corner = False
        self.is_obstacle_static = False
        self.is_obstacle_dynamic = False
        self.is_left_turn = False
        self.is_obstacle_mode_changed = False
        self.slam_finish = True
        self.front_wall_detected = False
        self.go_curve = False

        # subscriber
        self.img_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_steer_callback, queue_size=1)
        # self.obstacle_mode_sub = rospy.Subscriber('/obstacle_mode', Int16, self.obstacle_mode_callback, queue_size=1)
        self.trafiic_light_sub = rospy.Subscriber('GetTrafficLightStatus', GetTrafficLightStatus, self.traffic_light_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.Imu_callback, queue_size=1)
        self.Lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        # self.slam_finish_sub = rospy.Subscriber(
        #     "/slam_mission_finish", Bool, self.slam_finish_callback
        # )
        # publisher
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)   # FOR TEST!!!!!!!!!!!
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)# FOR TEST!!!!!!!!!!!
        

        # 제어값
        self.lane_steer = 0.0
        self.obstacle_steer = 0.0

        self.steer = 0.0#publish용
        self.speed = 2400

    def slam_finish_callback(self, msg):
        self.slam_finish = msg.data
        print('msg : ', self.slam_finish)

    def lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / pi
        degree_increment = self.scan.angle_increment * 180 / pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        front_wall_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 9.5 < self.scan.ranges[i] < 10 and abs(self.degrees[i]) < 10]
        front_wall_degrees_arr = [self.scan.ranges[i] for i in range(len(self.scan.ranges)) if 9.5 < self.scan.ranges[i] < 10 and abs(self.degrees[i]) < 10]
        if front_wall_degrees:
            self.front_wall_detected = True
        else:
            self.front_wall_detected = False
        print(front_wall_degrees_arr)
    
    def timer_callback(self, _event):
        try :
            self.drive()
        except :
            pass

    # callback
    def cam_steer_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location = self.lane_detection(img_bgr)
        # print("x_location", x_location)
        
        cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            self.lane_detetced = True
            cv2.imshow("Slidinw window", slideing_img)
        else:
            self.lane_detetced = False
        cv2.waitKey(1)
        
        if self.line_flag == 'R':
            pid = self.pidcal.pid_control(x_location)
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        elif self.line_flag == 'L':
            pid = self.pidcal.pid_control(x_location)
            steering = abs(pid - 0.5)
            self.lane_steer = steering

        elif self.line_flag == 'L2':
            pid = self.pidcal.pid_control2(x_location)
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        elif self.line_flag == 'CL':
            pid = self.pidcal.curve_pid_control(x_location)
            steering = abs(pid - 0.5)
            self.lane_steer = steering
            if steering < 0.5:
                steering *= 1.2
        elif self.line_flag == 'CL2':
            if x_location == 374.4:
                print("if x_location == 374.4:")
                return
            pid = self.pidcal.curve_pid_control2(x_location)
            steering = abs(pid - 0.5)
            # print("steering : ", steering)
            self.lane_steer = steering
            if steering < 0.5:
                self.lane_steer *= 1.2
                # print("aft steering : ", self.lane_steer)
        elif self.line_flag == 'CR':
            pid = self.pidcal.curve_pid_control2(x_location)
            steering = abs(pid - 0.5)
            self.lane_steer = steering
            if steering > 0.5:
                steering *= 1.2
            
    def Imu_callback(self, msg):
        self.yaw = abs(msg.orientation.z)

    def traffic_light_callback(self, msg):
        self.traffic_light_status = msg.trafficLightStatus
        # print(self.traffic_light_status)
        if self.traffic_light_status == 33:
            self.is_left_turn = True
        else:
            self.is_left_turn = False

    def obstacle_mode_callback(self, msg):
        if self.slam_finish == False:
            return
        if not self.is_obstacle_mode_changed:
            self.obstacle_mode = msg.data
        if self.obstacle_mode == 2 and self.static_obstacle == None:
            print("Here")
            self.static_obstacle = StaticObstacle()
        if self.is_obstacle_mode_changed == False and self.curve_detector.curve_flag == 1:
            if self.obstacle_mode == 1:
                if abs(self.yaw -  0.7) <= 0.05:
                    self.obstacle_mode = 2
                    self.is_obstacle_mode_changed = True
                    self.static_obstacle = StaticObstacle()
                    self.static_obstacle.set_straight_yaw(-0.7)
            elif self.obstacle_mode == 2:
                self.obstacle_mode = 1
                self.is_obstacle_mode_changed = True
        elif self.curve_detector.curve_flag == 2:
            self.obstacle_mode = 3
        
    
    def lane_detection(self, img):
        self.img = self.preprocess.preprocess(img, self.line_flag)
        slideimg, x_location , self.line_flag = self.slidewindow.slidewindow(self.img,self.line_flag)
        return slideimg, x_location
    
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

    # main drive
    def drive(self):

        print("flag : ", self.flag)

        if self.slam_finish == True:
            self.steer_pub.publish(self.steer)
            self.speed_pub.publish(self.speed)

        rotary_state = 0
        # while not rospy.is_shutdown():
        # print('img len : ', len(self.img))
        # print('flag : ', self.flag)
        # print('curve_flag : ', self.curve_detector.curve_flag)
        # CURVE-DETECT는 매 iteration마다 실행되어야함
        self.curve_detector.curve_detector(self.yaw)
        # slam 종료신호 받아야함!
        # SLAM-END-FLAG 선언


        if self.flag == 0 and self.slam_finish == True:# 출발 
            self.flag = 1


        if self.flag == 1: # 장애물 감지 및 회피
            # if self.obstacle_mode == 0:
            #     self.steer = self.lane_steer
            # elif self.obstacle_mode == 1:
            #     self.steer = self.dynamic_obstacle.steer
            #     self.speed = self.dynamic_obstacle.speed
            # elif self.obstacle_mode == 2:
            #     self.steer = self.static_obstacle.steer
            #     self.speed = self.static_obstacle.speed
            self.steer = self.lane_steer

            if self.curve_detector.curve_flag == 2:
                self.prev_time = time.time()
            #     del(self.static_obstacle)
            #     del(self.dynamic_obstacle)
                self.flag = 2

        if self.flag == 2:
            self.steer = self.lane_steer
            if (time.time() - self.prev_time) >= 1.275: # 좌회전(교차로, 로터리진입전)1.275
                # print('time diff : ', time.time() - self.prev_time)
                self.line_flag = 'CL'
                self.speed = 1200
                self.steer = self.lane_steer
                if (0.70<self.yaw<0.82):
                    self.line_flag = 'R'
                    self.flag = 3
        
        # if self.flag == 3: # 로터리 정지선 감지
        #     self.steer = self.lane_steer
        #     self.speed = 1700
        #     self.stop_line.isStop(self.img)
        #     if self.stop_line.stop_line_detected:
        #         self.flag = 4
        #         self.stop_line.stop_line_detected = False
                
        
        if self.flag == 3: # 노란선 감지
            rotary_speed, rotary_steer = self.Rotary.rotary_lego()
            if self.Rotary.state != 4:
                if self.Rotary.state == 0:
                    self.steer = self.lane_steer
                else:
                    self.speed,self.steer = rotary_speed, rotary_steer
                # self.Rotary.test()
            if self.Rotary.state == 4:
                # self.steer = self.lane_steer
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

        elif self.flag == 5: #신호받고 좌회전 직전
            self.line_flag = 'L'
            self.steer = self.lane_steer
            self.speed = 1400
            if 0.9999 <= self.yaw <= 1:
                self.steer = 0.5
                print('STOP LINE DETECTING')
                if self.stop_line.fast_stop_line_detected == False:
                    self.stop_line.isFastStop(self.img)
            if self.stop_line.fast_stop_line_detected:
                self.speed = 0
                self.steer = self.lane_steer
                print('traffic light status : ', self.traffic_light_status)
                print('is_left_turn : ', self.is_left_turn)
                if self.is_left_turn:
                    self.prev_time = time.time()
                    self.speed = 600
                    self.flag = 6
                    self.stop_line.fast_stop_line_detected = False

        elif self.flag == 6: # 차선변경

            print("yaw :", self.yaw)
            print("steer", self.steer)

            print("abs(self.straight_yaw - self.yaw)", abs(self.straight_yaw - self.yaw))
            
            # if time.time() - self.prev_time < 2:
            #     print('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~직진중~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
            # self.steer = 0.5
            # self.speed = 1000
                    
            # self.straight_yaw = 1
            # if time.time() - self.prev_time < 0.5:
            #     self.steer = 0.5
            #     self.speed = 1000
            # else:
            #     if abs(self.straight_yaw - self.yaw) < 0.1:
            #         self.steer = 0.4
            #     elif abs(self.straight_yaw - self.yaw) < 0.2:
            #         self.steer = 0.2

            if self.front_wall_detected == False:
                self.steer = 0.5
                self.speed = 1000
            else:
                self.go_curve = True

            if self.go_curve == True:
                self.straight_yaw = 0.7
                if 0.15 < abs(self.yaw - self.straight_yaw) < 0.4:
                    self.steer = 0.2
                elif 1.1 < abs(self.yaw - self.straight_yaw) <= 0.15:
                    self.steer = 0.4
                elif abs(self.yaw - self.straight_yaw) <= 1.1:
                    self.steer = 0.5


                # self.straight_yaw = 0.7
                # if 0.15 < abs(self.yaw - self.straight_yaw) < 0.4:
                #     self.steer = 0.2
                # elif 0.1 < abs(self.yaw - self.straight_yaw) <= 0.15:
                #     self.steer = 0.4
                # elif abs(self.yaw - self.straight_yaw) < 0.1:
                #     self.steer = 0.5

                
                    # if 0.9 <= abs(self.yaw - self.straight_yaw) <= 1.1:
                    #     self.steer = 0.4
                    # else:
                    #     self.steer = 0.5 





                # self.line_flag = 'CL2'
                # print("cL2 조향각 : ", self.lane_steer)
                # self.steer = self.lane_steer
                # if 0.69<self.yaw < 0.71:
                #     self.line_flag = 'CR'
                #     self.steer = self.lane_steer
                #     self.flag = 7
                    # self.stop_line.isStop(self.img)
                    # if self.stop_line.stop_line_detected:
                    #     self.flag = 7
                        
        elif self.flag == 7: # 정지선 검출
            self.speed = 1000
            self.line_flag = 'CR'
            self.steer = self.lane_steer
            print('self.line_flag : ', self.line_flag)
            print('self.lane_steer : ', self.lane_steer)
            # if self.find_yellow_lane():
            #     self.flag = 8
            #     self.line_flag = 'CL'
            if 0.999 < self.yaw < 1:
                self.flag = 8
                self.line_flag = 'L'
                self.steer = self.lane_steer
                self.stop_line.fast_stop_line_detected = False
        if self.flag == 8: # 달려
            self.speed = 1800
            print('self.line_flag : ', self.line_flag)
            self.line_flag = 'L2'
            self.steer = self.lane_steer
            if 0.67 < self.yaw < 0.73:
                self.stop_line.isFastStop(self.img)

            if self.stop_line.fast_stop_line_detected:
                self.steer = self.lane_steer
                self.flag = 9
                self.prev_time = time.time()
                self.stop_line.fast_stop_line_detected = False
            else:
                print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!STOP LINE NOT DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        elif self.flag == 9: # 정지선 검출
            if time.time() - self.prev_time < 1.9:
                error = round(self.yaw,3) - 0.7
                if abs(error) > 0.0005: 
                    # if error < 0:
                    #     self.steer = min(0.45, self.steer * 1.001)
                    #     print('1111111111')
                    # else:
                    #     print('22222222222')
                    #     self.steer = max(0.55, self.steer * 0.79)
                    self.steer += error * 0.2225
                    print('steer : ', self.steer)
                else:
                    self.steer = 0.5
            else:
                self.flag = 10

        elif self.flag == 10: # 정지
            self.steer = self.lane_steer
            self.line_flag = 'L2'

            self.stop_line.isStop(self.img)
            if  0 < self.yaw < 0.1 and self.stop_line.stop_line_detected:
                # self.speed = 0
                self.steer = self.lane_steer
                self.flag = 11
                self.prev_time = time.time()
                self.stop_line.stop_line_detected = False
            self.speed = 2000
            
        if self.flag == 11:
            if time.time() - self.prev_time < 1.5:
                self.steer = self.lane_steer
                self.speed = 2400
            else:
                self.speed = 0
                print('FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!FINISH!!!!!!!!!!!')
                return 

def run():
    rospy.init_node('main_drive', anonymous=True) 
    Main_drive = main_drive()
    rospy.Timer(rospy.Duration(1.0/30.0), Main_drive.timer_callback)
    rospy.spin()
    
if __name__ == '__main__':
    run()
