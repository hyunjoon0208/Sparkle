#! /usr/bin/env python3

import rospy, os, sys, cv2, numpy as np
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import morai_msgs.msg as CtrlCmd
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
from slidewindow_test2 import SlideWindow
from Preprocess_test import Preprocess
# from control.pidcal import PidCal

class Test_drive:
    def __init__(self):
        rospy.init_node('Test_drive', anonymous=True)
        # self.pub = rospy.Publisher('/ctrl_cmd', CtrlCmd.ControlCommand, queue_size=1)
        # publiser
        self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=1)
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=1)
        # subscriber
        self.sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.callback, queue_size=1)

        self.slidewindow = SlideWindow()
        self.preprocess = Preprocess()
        # self.pidcal = PidCal()
        self.img = None

    
    
    def callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, steering = self.lane_detection(img_bgr)

    def lane_detection(self, img):
        img = self.preprocess.preprocess(img)
        # img, steering= self.slidewindow.slidewindow(img)

        return img
    

if __name__ == '__main__':
    try:
        Test_drive()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass