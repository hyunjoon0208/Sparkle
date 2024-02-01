import cv2, rospy
import numpy as np
# from std_msgs.msg import Bool

class stop_line():
    def __init__(self):
        self.stop_line_detected = False
        self.fast_stop_line_detected = False
        # rospy.init_node('stop_line', anonymous=True)
        # self.stop_pub = rospy.Publisher('/stop_line', Bool, queue_size=1)
    def isStop(self,img):
        print('self.stop_line_detected :',self.stop_line_detected)
        # binary -> find contours -> find stop line
        # out_img = np.dstack((img, img, img))
        if img is not None:
            nonzero = img.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            stop_left_x = 100
            stop_right_x = 380
            stop_high_y = 360
            stop_low_y = 480
            # pts = np.array([[stop_left_x,stop_high_y],[stop_left_x,stop_low_y],[stop_right_x, stop_low_y],[stop_right_x,stop_high_y]],np.int32)
            stop_line = ((nonzerox >= stop_left_x) & (nonzerox <= stop_right_x) & (nonzeroy >= stop_high_y) & (nonzeroy <= stop_low_y)).nonzero()[0]
            # out_img = cv2.polylines(out_img, [pts], True, (0,255,0), 1)
            print("stop_line_num : ", len(stop_line))
            if len(stop_line) > 6000:
                self.stop_line_detected = True
            else:
                self.stop_line_detected = False
            # return self.stop_line_detected, out_img
            return self.stop_line_detected
        return self.stop_line_detected
    
    def isFastStop(self,img):
        print('self.fast_stop_line_detected :',self.fast_stop_line_detected)
        # binary -> find contours -> find stop line
        # out_img = np.dstack((img, img, img))
        if img is not None:
            nonzero = img.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            stop_left_x = 100
            stop_right_x = 380
            stop_high_y = 360
            stop_low_y = 400
            # pts = np.array([[stop_left_x,stop_high_y],[stop_left_x,stop_low_y],[stop_right_x, stop_low_y],[stop_right_x,stop_high_y]],np.int32)
            stop_line = ((nonzerox >= stop_left_x) & (nonzerox <= stop_right_x) & (nonzeroy >= stop_high_y) & (nonzeroy <= stop_low_y)).nonzero()[0]
            # out_img = cv2.polylines(out_img, [pts], True, (0,255,0), 1)
            print("stop_line_num : ", len(stop_line))
            if len(stop_line) > 5000:
                self.fast_stop_line_detected = True
            else:
                self.fast_stop_line_detected = False
            # return self.stop_line_detected, out_img
            return self.fast_stop_line_detected
        return self.fast_stop_line_detected