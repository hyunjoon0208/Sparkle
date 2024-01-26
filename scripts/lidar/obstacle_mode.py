#!/usr/bin/env phyton3
#-*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from math import *

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
        self.scan_msg = LaserScan()
        self.obstacle_mode = 0

    def lidar_callback(self, msg):
        self.scan_msg = msg
        degree_min = self.scan_msg.angle_min * 180 / pi
        degree_angle_increment = self.scan_msg.angle_increment * 180 / pi
        degrees = [degree_min + degree_angle_increment * index for index, value in enumerate(self.scan_msg.ranges)] 

        for index, value in enumerate(self.scan_msg.ranges):
            if self.obstacle_mode == 0:
                if ((-30 < degrees[index] < -10 and 0 < value < 1.5)): 
                    self.obstacle_mode = 1
                elif abs(degrees[index])< 3 and 0 < value < 3:
                    self.obstacle_mode = 2
        print("self.obstacle_mode", self.obstacle_mode)
        self.obstacle_mode_pub.publish(self.obstacle_mode)
            
def main():
    try:
        obstacle_mode = Obstacle_mode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()