import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import actionlib

class NavigationClient:
    def __init__(self):
        self.curr_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.curr_pose_callback)
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.curr_pose = PoseWithCovarianceStamped()
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.slam_finish_pub = rospy.Publisher("/slam_mission_finsih", Bool, queue_size=10)
        self.client.wait_for_server()
        self.goal_list = list()
        self.sequence = 0
        self.end_flag = 0
        self.start_time = rospy.Time.now()

        # Initialize initial pose
        self.initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)

    def curr_pose_callback(self, msg):
        self.curr_pose = msg

    def update_initial_pose(self, x, y, orientation_w, orientation_z):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"
        initial_pose_msg.pose.pose.position = Point(x, y, 0.0)
        initial_pose_msg.pose.pose.orientation.w = orientation_w
        initial_pose_msg.pose.pose.orientation.z = orientation_z
        self.initial_pose_pub.publish(initial_pose_msg)

    def run(self):
        if self.client.get_state() != GoalStatus.ACTIVE:
            print(self.client.get_state(), GoalStatus.ACTIVE)
            self.start_time = rospy.Time.now()
            # Update initial pose
            self.update_initial_pose(0.0, 0.0, 1.0, 0.0)
            self.client.send_goal(self.goal_list[self.sequence])
        else:
            range = 0.2
            if (self.curr_pose.pose.pose.position.x > self.goal_list[self.sequence].target_pose.pose.position.x - range and
                    self.curr_pose.pose.pose.position.x < self.goal_list[self.sequence].target_pose.pose.position.x + range):
                self.end_flag = 1
            if (rospy.Time.now().to_sec() - self.start_time.to_sec()) > 30.0 or self.end_flag == 1:
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