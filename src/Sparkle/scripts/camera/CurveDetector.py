class CurveDetector:
    def __init__(self):
        self.curve_flag = 0
    
    def curve_detector(self,yaw):
        print("yaw", yaw)
        if self.curve_flag == 0:
            # if 0.65 <= yaw <= 0.71:
            # if 0.37 <= yaw <= 0.71:
            if 0.33 <= yaw <= 0.71:
                self.curve_flag = 1
        elif self.curve_flag == 1:
            if 0.985 <= yaw <= 1.0:
                self.curve_flag = 2
        elif self.curve_flag == 2:
            if 0.69 <= yaw <= 0.71:
                self.curve_flag = 3
        else:
            if 0.0 <= yaw <= 0.1:
                self.curve_flag = 0