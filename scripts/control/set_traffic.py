#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from morai_msgs.msg import SetTrafficLight

rospy.init_node('set_traffic_light', anonymous=True)
pub = rospy.Publisher('/set_traffic_light', SetTrafficLight, queue_size=1)
msg = SetTrafficLight()
msg.trafficLightStatus = 1
pub.publish(msg)

msg.trafficLightStatus = 33
pub.publish(msg)
