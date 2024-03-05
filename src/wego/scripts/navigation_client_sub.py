#! usr/bin/env python3

import rospy
from std_msgs.msg import Bool


class navigation_client_sub:
    def __init__(self):
        self.slam_finish_sub = rospy.Subscriber(
            "/slam_mission_finsih", Bool, self.slam_finish_callback
        )
        self.slam_finish = Bool()
        self.slam_finish_flag = 0

    def slam_finish_callback(self, msg):
        self.slam_finish = msg
        print(self.slam_finish)
        self.slam_finish_flag = 1

    def slam_finish_flag(self):
        return self.slam_finish_flag

    def slam_finish(self):
        return self.slam_finish


if __name__ == "__main__":
    rospy.init_node("navigation_client_sub")
    navigation_client_sub = navigation_client_sub()
    rospy.spin()
