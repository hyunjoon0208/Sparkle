import cv2, rospy
import numpy as np
# from std_msgs.msg import Bool

class stop_line():
    def __init__(self):
        self.stop_line_detected = False
        # rospy.init_node('stop_line', anonymous=True)
        # self.stop_pub = rospy.Publisher('/stop_line', Bool, queue_size=1)
    def isStop(self,img):
        # binary -> find contours -> find stop line
        # out_img = np.dstack((img, img, img))
        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        stop_left_x = 100
        stop_right_x = 380
        stop_high_y = 240
        stop_low_y = 300
        # pts = np.array([[stop_left_x,stop_high_y],[stop_left_x,stop_low_y],[stop_right_x, stop_low_y],[stop_right_x,stop_high_y]],np.int32)
        stop_line = ((nonzerox >= stop_left_x) & (nonzerox <= stop_right_x) & (nonzeroy >= stop_high_y) & (nonzeroy <= stop_low_y)).nonzero()[0]
        # out_img = cv2.polylines(out_img, [pts], True, (0,255,0), 1)
        print("stop_line_num : ", len(stop_line))
        if len(stop_line) > 5000:
            self.stop_line_detected = True
        else:
            self.stop_line_detected = False
        # return self.stop_line_detected, out_img
        return self.stop_line_detected
            
    # def isYellow(image):
    #     # image -> hsv -> detect yellow -> binary
    #     return True