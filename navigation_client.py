#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib


class NavigationClient:
    def __init__(self):
        self.curr_pose_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.curr_pose_callback
        )
        self.speed_pub = rospy.Publisher("/commands/motor/speed", Float64, queue_size=1)
        self.curr_pose = PoseWithCovarianceStamped()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # when self.end_flag == 1, publish stop
        self.slam_finish_pub = rospy.Publisher(
            "/slam_mission_finsih", Bool, queue_size=10
        )
        self.client.wait_for_server()
        self.goal_list = list()

        # 대회 맵 start pose 값
        # -------------------------------------------------------------
        self.start_x = self.curr_pose.pose.pose.position.x
        self.start_y = self.curr_pose.pose.pose.position.y
        self.start_w = self.curr_pose.pose.pose.orientation.w
        self.start_z = self.curr_pose.pose.pose.orientation.z
        print(self.start_x, self.start_y, self.start_w, self.start_z)

        self.start = MoveBaseGoal()
        self.start.target_pose.header.frame_id = "map"
        self.start.target_pose.pose.position.x = self.start_x
        self.start.target_pose.pose.position.y = self.start_y
        self.start.target_pose.pose.orientation.w = self.start_w
        self.start.target_pose.pose.orientation.z = self.start_z
        # --------------------------------------------------------------
        # test 맵 start pose 값
        # self.start.target_pose.header.frame_id = "map"
        # self.start.target_pose.pose.position.x = -0.00258947235463027
        # self.start.target_pose.pose.position.y = 0.009242369067008465
        # self.start.target_pose.pose.orientation.w = 1.0
        # self.start.target_pose.pose.orientation.z = 0

        # self.goal_list.append(self.start)

        self.goal = MoveBaseGoal()
        # ㄷ대회 맵 goal pose 값
        # --------------------------------------------------------------
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.pose.position.x = 11.984210611857957  # 11.259986260618344
        self.goal.target_pose.pose.position.y = 16.622569660692463
        self.goal.target_pose.pose.orientation.w = 0.7421430974206135
        self.goal.target_pose.pose.orientation.z = 0.6702414661530111
        # --------------------------------------------------------------
        # # test 맵 goal pose 값
        # self.goal.target_pose.header.frame_id = "map"
        # self.goal.target_pose.pose.position.x = 12.53863143616246
        # self.goal.target_pose.pose.position.y = 18.446834705650488
        # self.goal.target_pose.pose.orientation.w = 0.704720102708549
        # self.goal.target_pose.pose.orientation.z = 0.7094854310262136

        self.goal_list.append(self.goal)

        self.sequence = 0
        self.end_flag = 0
        self.start_time = rospy.Time.now()

    def curr_pose_callback(self, msg):
        self.curr_pose = msg

    # def run(self):
    #     if self.client.get_state() != GoalStatus.ACTIVE:  # 현재
    #         print(self.client.get_state(), GoalStatus.ACTIVE)
    #         self.start_time = rospy.Time.now()

    #         # self.sequence = (self.sequence + 1) % 2
    #         # if(self.sequence == 0):
    #         #     self.end_flag = 1
    #         self.client.send_goal(self.goal_list[self.sequence])

    #     else:
    #         range = 0.2
    #         if (
    #             self.curr_pose.pose.pose.position.x
    #             > self.goal_list[self.sequence].target_pose.pose.position.x - range
    #             and self.curr_pose.pose.pose.position.x
    #             < self.goal_list[self.sequence].target_pose.pose.position.x + range
    #         ):
    #             self.end_flag = 1
    #             print(self.end_flag)
    #             # if (
    #             #     rospy.Time.now().to_sec() - self.start_time.to_sec()
    #             # ) > 30.0 or self.end_flag == 1:
    #             self.stop()
    #             print("stop")
    #             self.slam_finish_pub.publish(True)
    #             GoalStatus.SUCCEEDED = True

    def run(self):
        if not self.end_flag and self.client.get_state() != GoalStatus.ACTIVE:
            print(self.client.get_state(), GoalStatus.ACTIVE)
            self.start_time = rospy.Time.now()
            self.client.send_goal(self.goal_list[self.sequence])

        if not self.end_flag:
            range = 0.2
            if (
                self.curr_pose.pose.pose.position.x
                > self.goal_list[self.sequence].target_pose.pose.position.x - range
                and self.curr_pose.pose.pose.position.x
                < self.goal_list[self.sequence].target_pose.pose.position.x + range
            ):
                self.end_flag = 1
                self.stop()
                print("stop")
                self.slam_finish_pub.publish(True)
                GoalStatus.SUCCEEDED = True

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
