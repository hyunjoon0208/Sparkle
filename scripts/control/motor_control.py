#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64

class MotorContorl:
    def __init__(self):
        rospy.init_node('motor_control', anonymous=True)
        self.steer = 0.5
        self.speed = 1000

        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)

        rospy.Subscriber('/speed', Float64, self.speed_callback, queue_size=1)
        rospy.Subscriber('/steer', Float64, self.steer_callback, queue_size=1)

    def speed_callback(self, msg):
        self.speed = msg.data

    def steer_callback(self, msg):
        self.steer = msg.data

    def run(self):
        while not rospy.is_shutdown():
            self.speed_pub.publish(self.speed)
            self.steer_pub.publish(self.steer)


if __name__ == '__main__':
    try:
        motor_contorl = MotorContorl()
        motor_contorl.run()
    except rospy.ROSInterruptException:
        pass
