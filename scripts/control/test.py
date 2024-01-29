#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage


class test:
    def __init__(self) -> None:
        rospy.init_node('test', anonymous=True)
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        self.cam_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.cam_callback, queue_size=1)
        self.img = None
    def cam_callback(self, data):
        self.img = data
        self.speed_pub.publish(3000)
        self.steer_pub.publish(0.5)


if __name__ == '__main__':
    try:
        test()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass