import cv2
import numpy as np

def cam_steer_callback(self, data):
        img_bgr =cv2.imdecode(np.fromstring(data.data, np.uint8),cv2.IMREAD_COLOR)
        slideing_img, x_location, line_flag = self.lane_detection(img_bgr)
        cv2.imshow("Image window", img_bgr)
        if slideing_img is not None:
            self.lane_detetced = True
            cv2.imshow("Slidinw window", slideing_img)
        else:
            self.lane_detetced = False
        cv2.waitKey(1)
        pid = self.pidcal.pid_control(x_location)
        if line_flag == 'R':
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        else:
            steering = abs(pid - 0.5)
            self.lane_steer = steering
        
def velocity_callback(self, msg):
    self.velocity = msg.data
def obstacle_callback(self, msg):
    self.obstacle_steer = msg.data
def traffic_light_callback(self, msg):
    try:
        if msg.traffic_light_status == 33:
            self.is_left_turn = True
        else:
            self.is_left_turn = False
    except:
        pass