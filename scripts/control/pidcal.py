import time

class PidCal:
    error_sum = 0
    error_old = 0
    p = [0.000699, 0.000001, 0.00001] # optimized kp,ki,kd
    curve_p = [0.00215, 0.000001, 0.00001]
    # p = [0.001, 0.0000003, 0.003]
    dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd
    curve_dp = [curve_p[0]/10, curve_p[1]/10, curve_p[2]/10]

    def __init__(self):
        # print "init PidCal"
        self.x = 0
        self.clear_time = time.time()
        
    def cal_error(self, setpoint=320):
        return setpoint - self.x

    # twiddle is for optimize the kp,ki,kd
    def twiddle(self, setpoint=320):
        best_err = self.cal_error()
        #threshold = 0.001
        #threshold = 1e-09
        threshold = 0.0000000000000000000000000000001

        # searching by move 1.1x to the target and if go more through the target comeback to -2x
        while sum(self.dp) > threshold:
            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error()

                if err < best_err:  # There was some improvement
                    best_err = err
                    self.dp[i] *= 1.1
                else:  # There was no improvement
                    self.p[i] -= 2*self.dp[i]  # Go into the other direction
                    err = self.cal_error()

                    if err < best_err:  # There was an improvement
                        best_err = err
                        self.dp[i] *= 1.05
                    else:  # There was no improvement
                        self.p[i] += self.dp[i]
                        # As there was no improvement, the step size in either
                        # direction, the step size might simply be too big.
                        self.dp[i] *= 0.95

    def curve_twiddle(self, setpoint=320):
        best_err = self.cal_error()
        threshold = 0.0000000000000000000000000000001

        while sum(self.curve_dp) > threshold:
            for i in range(len(self.curve_p)):
                self.curve_p[i] += self.curve_dp[i]
                err = self.cal_error()

                if err < best_err:
                    best_err = err
                    self.curve_dp[i] *= 1.1
                else:
                    self.curve_p[i] -= 2*self.curve_dp[i]
                    err = self.cal_error()

                    if err < best_err:
                        best_err = err
                        self.curve_dp[i] *= 1.05
                    else:
                        self.curve_p[i] += self.curve_dp[i]
                        self.curve_dp[i] *= 0.95
                        

    # setpoint is the center and the x_current is where the car is
    # width = 640, so 320 is the center but 318 is more accurate in real
    def pid_control(self, x_current, setpoint=320):
        # if time.time() - self.clear_time > 0.1:
        #     self.error_sum = 0
        #     self.clear_time = time.time()
        # print "HHHHHHHHHHHHHHH"
        # print x_current
        self.x = int(x_current)
        self.twiddle()

        error = setpoint - x_current
        p1 = round(self.p[0] * error, 9)
        self.error_sum += error
        i1 = round(self.p[1] * self.error_sum, 9)
        d1 = round(self.p[2] * (error -  self.error_old), 9)
        self.error_old = error
        pid = p1 +i1+ d1
        # pid = p1 + d1
        # pid = p1 + d1
        # print("p : " ,p)
        # print("i : " ,i)
        # print("d : " ,d)
        return pid

    def curve_pid_control(self, x_current, setpoint=320):
        self.x = int(x_current)
        self.twiddle()

        error = setpoint - x_current
        p1 = round(self.curve_p[0] * error, 9)
        self.error_sum += error
        i1 = round(self.curve_p[1] * self.error_sum, 9)
        d1 = round(self.curve_p[2] * (error -  self.error_old), 9)
        self.error_old = error
        pid = p1 +i1+ d1
        # pid = p1 + d1
        # pid = p1 + d1
        # print("p : " ,p)
        # print("i : " ,i)
        # print("d : " ,d)
        return pid