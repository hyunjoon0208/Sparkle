#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import SetTrafficLight
import time

rospy.init_node('set_traffic_light', anonymous=True)
pub = rospy.Publisher('/SetTrafficLight', SetTrafficLight, queue_size=1)
msg = SetTrafficLight()
msg.trafficLightStatus = 33
msg.trafficLightIndex = 'SN000005'

s_time = time.time()
while (time.time() - s_time < 1):
    pub.publish(msg)