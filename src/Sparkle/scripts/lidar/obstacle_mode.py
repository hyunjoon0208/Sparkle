#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Int16, Bool
from math import *
import os, sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from camera.CurveDetector import CurveDetector


"""
self.obstacle_mode
0 -> normal
1 -> dynamic
2 -> static
"""

class Obstacle_mode:
    def __init__(self):
        rospy.init_node("obstacle_mode")
        self.sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.obstacle_mode_pub = rospy.Publisher("/obstacle_mode", Int16, queue_size=1)
        self.slam_finish_sub = rospy.Subscriber(
            "/slam_mission_finish", Bool, self.slam_finish_callback
        )
        self.IMU_sub = rospy.Subscriber('/imu', Imu, self.yaw_callback)
        self.scan_msg = LaserScan()
        self.obstacle_mode = 0
        self.slam_finish = False
        self.straight_yaw = 0
        self.curve_detector = CurveDetector()
        self.is_curve_flag_changed = False
        self.yaw = 0
        

    def yaw_callback(self, msg):
        self.yaw = msg.orientation.z
        if self.straight_yaw == -0.7: # dynamic -> static
            if self.yaw > 0:
                self.yaw = -self.yaw

    def slam_finish_callback(self, msg):
        self.slam_finish = msg.data

    def lidar_callback(self, msg):
        print("self.curve_detector.curve_flag", self.curve_detector.curve_flag)
        self.curve_detector.curve_detector(abs(self.yaw))
        if self.curve_detector.curve_flag == 1 and self.is_curve_flag_changed == False:
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            self.obstacle_mode = 0
            self.straight_yaw = -0.7
            self.is_curve_flag_changed = True
        self.obstacle_mode_pub.publish(self.obstacle_mode)
        print(self.obstacle_mode)
        print("yaw", self.yaw, "abs(self.straight_yaw - self.yaw)", abs(self.straight_yaw - self.yaw))
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)] 

        for index, value in enumerate(self.scan_msg.ranges):
            # if self.obstacle_mode == 0 and abs(self.straight_yaw - self.yaw) < 0.1:

            #     if (15 <= degrees[index] <= 30 and 2.5 < value <= 3): 
            #         self.obstacle_mode = 1 
            #     # elif abs(degrees[index])< 15 and 0 < value < 1.6:  
            #     elif abs(degrees[index])< 15 and 0 < value < 2.3:            
            #         self.obstacle_mode = 2


            if self.obstacle_mode == 0 and abs(self.straight_yaw - self.yaw) < 0.1:

                if (15 <= degrees[index] <= 30 and 2.5 < value <= 3): 
                    self.obstacle_mode = 1 
                # elif abs(degrees[index])< 15 and 0 < value < 1.6:  
                elif abs(degrees[index])< 15 and 0 < value < 2.3:            
                    self.obstacle_mode = 2

        # if self.slam_finish == False:
        #     self.obstacle_mode = 0
            
def main():
    try:
        obstacle_mode = Obstacle_mode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()