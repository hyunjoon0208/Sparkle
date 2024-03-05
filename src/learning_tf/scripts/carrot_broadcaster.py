#! /usr/bin/env python3

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped


class CarrotBroadcaster:
    def __init__(self):
        self.br = tf2_ros.StaticTransformBroadcaster()  # carrot과 고정된 길이만큼
        self.transformStamped = TransformStamped()  # turtle1과 carrot을 이어주는 데이터가 들어간다
        self.transformStamped.header.frame_id = "turtle1"
        self.transformStamped.child_frame_id = "carrot"

    def broadcast(self):
        self.transformStamped.header.stamp = rospy.Time().now()
        # x,z는 다 0이다 default가 0임
        self.transformStamped.transform.translation.y = 1.0
        self.transformStamped.transform.rotation.w = 1.0
        self.br.sendTransform(self.transformStamped)


def main():
    rospy.init_node("CarrotBroadcaster")
    cb = CarrotBroadcaster()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cb.broadcast()
        rate.sleep()


if __name__ == "__main__":
    main()
