#! /usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import actionlib


class NavigationClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        self.goal_list = list()

        self.start = MoveBaseGoal()
        self.start.target_pose.header.frame_id = "map"
        self.start.target_pose.pose.position.x = 0.08081695765820235
        self.start.target_pose.pose.position.y = 0.08893138001633945
        self.start.target_pose.pose.orientation.w = 0.9998993723606314
        self.start.target_pose.pose.orientation.z = -0.014186090117273264

        self.goal_list.append(self.start)

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = 11.259986260618344
        self.goal.target_pose.pose.position.y = 17.139815641177556
        self.goal.target_pose.pose.orientation.w = 0.708821167226234
        self.goal.target_pose.pose.orientation.z = 0.7053882284898432

        self.goal_list.append(self.goal)

        self.sequence = 0
        self.start_time = rospy.Time.now()

    def run(self):
        if self.client.get_state() != GoalStatus.ACTIVE:  # 현재
            self.start_time = rospy.Time.now()
            self.sequence = (self.sequence + 1) % 2
            self.client.send_goal(self.goal_list[self.sequence])
        else:
            if (
                rospy.Time.now().to_sec() - self.start_time.to_sec()
            ) > 30.0:  # 30초가 지나면 stop
                self.stop()

    def stop(self):
        self.client.cancel_all_goals()


def main():
    rospy.init_node("navigation_client")
    navigation_client = NavigationClient()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        navigation_client.run()
        rate.sleep()


if __name__ == "__main__":
    main()
