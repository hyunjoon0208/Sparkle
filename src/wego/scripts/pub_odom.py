#! /usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry  # 어차피 quaternion으로 들어와서(?)
from geometry_msgs.msg import TransformStamped
import tf2_ros


class PubTF:
    def __init__(self):
        rospy.init_node("odom_tf_broadcast")
        self.br = tf2_ros.TransformBroadcaster()  # 고정된게 x기 때문에
        self.transformStamped = TransformStamped()
        rospy.Subscriber("/odom", Odometry, self.callback)
        self.transformStamped.header.frame_id = "odom"

    def callback(self, msg):
        self.transformStamped.header.stamp = rospy.Time.now()
        self.transformStamped.child_frame_id = msg.child_frame_id
        self.transformStamped.transform.translation.x = msg.pose.pose.position.x
        self.transformStamped.transform.translation.y = msg.pose.pose.position.y
        self.transformStamped.transform.translation.z = msg.pose.pose.position.z
        self.transformStamped.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(self.transformStamped)


def main():
    try:
        pub_tf = PubTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
