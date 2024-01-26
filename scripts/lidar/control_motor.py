#!/usr/bin/env phyton3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Int16
from math import *
import numpy as np
from CurveDetector import CurveDetector

class Control_motor:
    def __init__(self):
        rospy.init_node("control_motor")
        self.obstacle_detector = rospy.Publisher("/obstacle_detector", Float64, queue_size=1) # TODO : return dict
        rospy.Subscriber("/obstacle_mode", Int16, self.obstacle_mode_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)

        self.scan_msg = LaserScan()
        self.speed_msg = Float64()
        self.steer_msg = Float64()
        self.obstacle_mode = 0

        self.is_mode_changed = False

        self.velocity = 2400
        self.steer = 0.5

    def obstacle_mode_callback(self, msg):
        self.obstacle_mode = msg.data
        if not self.is_mode_changed and CurveDetector.detectCurves():
            if self.obstacle_mode == 1:
                self.obstacle_mode = 2
                self.is_mode_changed = True
            elif self.obstacle_mode == 2:
                self.obstacle_mode = 1
                self.is_mode_changed = True

    def lidar_callback(self, msg):
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi

        degrees = [degree_min + degree_angle_increment *index for index, value in enumerate(self.scan_msg.ranges)] 

        for index, value in enumerate(self.scan_msg.ranges):
            if self.obstacle_mode == 1:
                if abs(degrees[index]) < 50 and 0 < value < 1.6:
                    self.velocity = 0
                else:
                    self.velocity = 2400
            elif self.obstacle_mode == 2:
                if abs(degrees[index])< 3 and 0 < value < 1:
                    # TODO : static obstacle algo 
                    self.velocity = 0
                    self.steer = 0.5

    def return_motor_parm(self):
        return {"velocity" : self.velocity, "steer": self.steer}

def main():
    try:
        control_motor = Control_motor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()