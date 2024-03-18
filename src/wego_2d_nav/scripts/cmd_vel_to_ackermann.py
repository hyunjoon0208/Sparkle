# !/usr/bin/env python3

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class Cmd2Ack:
    def __init__(self):
        rospy.init_node("cmd_vel_to_ackermann", anonymous=True)

        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10
        )
        self.ack_pub = rospy.Publisher(
            "/high_level/ackermann_cmd_mux/input/nav_0",
            AckermannDriveStamped,
            queue_size=10,
        )
        self.wheelbase = 0.26  # 뒤와 앞 휠 사이의 거리 / 바꾸지 말 것 webot 기준임
        self.frame_id = "odom"

    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0 or v == 0:  # 뭐라는 걸까 이거 무튼 exception을 잡기 위해서 조건문 씀 알아보기..
            return 0
        radius = v / omega  # v가 0이면 일직선 -> 무한대 값 -> exception으로 던져진다.
        return math.atan(self.wheelbase / radius)

    def cmd_vel_callback(self, data):
        v = data.linear.x
        steering = self.convert_trans_rot_vel_to_steering_angle(
            v, data.angular.z, self.wheelbase
        )

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = v

        self.ack_pub.publish(msg)


def main():
    try:
        cmd_2_ack = Cmd2Ack()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
