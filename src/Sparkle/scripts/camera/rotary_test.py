import rospy, time, numpy as np, cv2
from .StopDetector import stop_line
from control.pidcal import PidCal
from .slidewindow import SlideWindow
from .Preprocess import Preprocess
from sensor_msgs.msg import Image, CompressedImage, Imu, LaserScan
from std_msgs.msg import Float64


class Rotary:
    def __init__(self):

        # 1. Stop line
        # 2. Front car (While Driving)
        # 3. Wide car(While Stopping)
        # 4. Straight from Stop line
        # 5. Rotary right turn
        # 6. Rotary exit



        self.stop_detector = stop_line()
        self.preprocess = Preprocess()
        self.pid = PidCal()
        self.img = None
        self.lane_steer = None
        self.yaw = 0
        self.speed = 0
        self.steer = 0
        self.state = 0

        self.scan = None
        self.degrees = []

        self.yellow_lane_detected = False
        self.wide_car_detected = False
        self.front_car_detected = False
        self.back_pannel_detected = False
        self.straight_from_stop = 0
        self.rotary_right_turn = 0
        self.straight_escape_rotary = 0
        self.line_flag = 'L'

        self.img_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.img_callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)

        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)


    def img_callback(self, data):
        self.img = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)
        yellow = self.preprocess.find_yellow(self.img)
        yellow_nonzero = yellow.nonzero()
        if len(yellow_nonzero[0]) > 1000:
            self.yellow_lane_detected = True
        else:
            self.yellow_lane_detected = False
        
        self.img = self.preprocess.preprocess(self.img)

        try:
            pid = self.pidcal.pid_control(self.slidewindow.slidewindow(self.img, self.line_flag))
            self.lane_steer = abs(pid - 0.5)
        except:
            pid = None

    def imu_callback(self, msg):
        self.yaw = abs(msg.orientation.z)

    def lidar_callback(self, msg):
        self.scan = msg
        degree_min = self.scan.angle_min * 180 / np.pi
        degree_max = self.scan.angle_max * 180 / np.pi
        degree_increment = self.scan.angle_increment * 180 / np.pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]

        # Check for obstacles in the rotary
        wide_car_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.8 and -5 < self.degrees[i] < 70]
        if len(wide_car_degrees) > 0:
            self.wide_car_detected = True
        else:
            self.wide_car_detected = False
        
        # Check for obstacles in the front
        front_car_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] < 0.7 and abs(self.degrees[i]) < 55]
        if len(front_car_degrees) > 0:
            self.front_car_detected = True
        else:
            self.front_car_detected = False

        back_pannel_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if 4.4 <self.scan.ranges[i] < 5.1 and 170 < abs(self.degrees[i]) < 180]
        if len(back_pannel_degrees) > 0:
            self.back_pannel_detected = True
        else:
            self.back_pannel_detected = False
        

        # if self.back_pannel_detected:
        #     print('11111111111111111111111111111111111111')
        # else:
        #     print('00000000000000000000000000000000000000')
    
    def rotary_lego(self):
        print('Rotary Start')
        print('Rotary state :', self.state)
        print('self.back_pannel_detected :',self.back_pannel_detected)
        # if self.state == 0: # 정지선 인식전.
            
        #     if self.front_car_detected or self.wide_car_detected:
        #         self.speed = 0
        #     else:
        #         print('ELSE')
        #         print('self.stop_detector.isStop(self.img) :',self.stop_detector.isStop(self.img))
        #         if self.stop_detector.isStop(self.img):
        #             self.speed = 0
        #             self.state = 1
        #             return self.speed, self.steer
        #         else:
        #             self.speed = 200
        #             self.steer = 0.5
        #             print('ELSE ELSE')  
        #             return self.speed, self.steer
                
        # if self.state == 1: # 정지선 인식후.
        #     if self.front_car_detected or self.wide_car_detected:
        #         self.speed = 0
        #     else:
        #         if self.back_pannel_detected == False:
        #             self.speed = 400
        #             self.steer = 0.5
        #             # self.speed_pub.publish(self.speed)
        #             # self.steer_pub.publish(self.steer)
                        
        #         else:
        #             self.state = 2
            
        # if self.state == 2: # 회전 시작.
        #     if self.front_car_detected or self.wide_car_detected:
        #         self.speed = 0
        #     else:
        #         if 0.928 < self.yaw < 0.932:
        #             self.state = 3
        #             # self.speed_pub.publish(self.speed)
        #             # self.steer_pub.publish(self.steer
        #         else:
        #             print('self.yaw :',self.yaw)
        #             self.speed = 300
        #             self.steer = 0.9

        # if self.state == 3: # 직진.
        #     if self.front_car_detected or self.wide_car_detected:
        #         self.speed = 0
        #     else:
        #         self.speed = 0
        if self.state == 0:
            if self.front_car_detected or self.wide_car_detected:
                self.speed = 0
            else:
                if self.back_pannel_detected == True:
                    self.state = 1
        
        if self.state == 1:
            if self.front_car_detected or self.wide_car_detected:
                self.speed = 0
            else:
                if self.back_pannel_detected == False:
                    self.state = 2
                else:
                    self.steer = 0.5
                    self.speed = 700
        
        if self.state == 2:
            if self.front_car_detected or self.wide_car_detected:
                self.speed = 0
            else:
                if 0.928 < self.yaw < 0.932:
                    self.state = 3
                else:
                    print('self.yaw :',self.yaw)
                    self.steer = 1
                    self.speed = 300
                    
        
        if self.state == 3:
            if self.yellow_lane_detected == True:
                self.state = 4
            if self.front_car_detected or self.wide_car_detected:
                self.speed = 0
            else:
                self.speed = 1000
                self.steer = 0.5

            
        return self.speed, self.steer
    
    def test(self):
        degree_min = self.scan.angle_min * 180 / np.pi
        degree_max = self.scan.angle_max * 180 / np.pi
        degree_increment = self.scan.angle_increment * 180 / np.pi
        self.degrees = [degree_min + degree_increment * i for i in range(len(self.scan.ranges))]


        back_pannel_degrees = [self.degrees[i] for i in range(len(self.scan.ranges)) if self.scan.ranges[i] > 5.25 and 170 < abs(self.degrees[i]) < 180]
        if len(back_pannel_degrees) > 0:
            self.back_pannel_detected = True
        else:
            self.back_pannel_detected = False
        
        if self.back_pannel_detected:
            print('11111111111111111111111111111111111111')
        else:
            print('00000000000000000000000000000000000000')