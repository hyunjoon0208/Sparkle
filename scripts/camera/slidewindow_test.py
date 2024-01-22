#! /usr/bin/env python3

import cv2
import numpy as np

class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None

    def slidewindow(self, img):
        # self.lane_tunning_num = 0.001
        x_location = 320
        out_img = np.dstack((img, img, img))
        # out_img = img
        height = img.shape[0]
        width = img.shape[1]
        self.current_line = None
        # num of windows and init the height
        window_height = 10
        nwindows = 30

        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()
        # print("nonzero : ", nonzero)
        #print nonzero
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # print(nonzerox)
        # init data need to sliding windows
        margin = 20
        minpix = 10
        # print("hello")
        left_lane_inds = []
        right_lane_inds = []
        
        win_y_high = 400
        win_y_low = 480
        left_lane_start_x = 30
        left_lane_end_x = 130
        right_lane_start_x = 310
        right_lane_end_x = 440
        
        
        pts_left = np.array([[left_lane_start_x,win_y_low],[left_lane_start_x,win_y_high],[left_lane_end_x, win_y_high],[left_lane_end_x,win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)


        pts_right = np.array([[right_lane_start_x,win_y_low],[right_lane_start_x,win_y_high],[right_lane_end_x,win_y_high],[right_lane_end_x,win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  win_y_high ) &(nonzeroy < win_y_low) & (nonzerox <= left_lane_end_x)).nonzero()[0]
        good_right_inds = ((nonzerox >= right_lane_start_x) & (nonzeroy >= win_y_high) &(nonzeroy < win_y_low) & (nonzerox <= right_lane_end_x)).nonzero()[0]
        line_exist_flag = None
        y_current = height
        x_current = None
        good_center_inds = None
        p_cut = None

        if len(good_left_inds) > len(good_right_inds):
            line_flag = 0
            self.current_line = 'LEFT'
            x_current = np.int(np.median(nonzerox[good_left_inds]))
            y_current = np.int(np.median(nonzeroy[good_left_inds]))

        elif len(good_right_inds) > len(good_left_inds):
            self.current_line = 'RIGHT'
            line_flag = 1
            
        if line_flag != 3:

            cx,cy,lx,rx,ly,ry = [],[],[],[],[],[]
            for window in range(0, nwindows):
                if line_flag == 0:
                    win_y_low = height - (window + 1) * window_height
                    win_y_high = height - (window) * window_height

                    LX_win_x_low = lx_current - margin
                    LX_win_x_high = lx_current + margin
                    
                    RX_win_x_low = rx_current - margin
                    RX_win_x_high = rx_current + margin

                    img = cv2.rectangle(out_img, (LX_win_x_low, win_y_low), (LX_win_x_high, win_y_high), (0, 255, 0), 1)
                    img = cv2.rectangle(out_img, (RX_win_x_low, win_y_low), (RX_win_x_high, win_y_high), (255, 0, 0), 1)
                    
                    good_left_inds = ((nonzerox >= LX_win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= LX_win_x_high)).nonzero()[0]
                    good_right_inds = ((nonzerox >= RX_win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= RX_win_x_high)).nonzero()[0]

                    left_lane_inds.append(good_left_inds)
                    right_lane_inds.append(good_right_inds)
                    # lx.append(lx_current)
                    # rx.append(rx_current)
                    # ly.append(win_y_low)
                    # ry.append(win_y_high)
                    cx.append((lx_current + rx_current) // 2)
                    cy.append(win_y_high)
                    if window < 10:
                        if len(good_left_inds) > minpix and len(good_right_inds) > minpix:
                            lx_current = np.int(np.median(nonzerox[good_left_inds]))
                            rx_current = np.int(np.median(nonzerox[good_right_inds]))
                            # x_location = (lx_current + rx_current) // 2
                            x_location = np.median([lx_current, rx_current])

                            # print('lx_current : ', lx_current)
                            # print('rx_current : ', rx_current)
                        elif len(good_right_inds) > minpix:
                            rx_current = np.int(np.median(nonzerox[good_right_inds]))
                            print('right')
                            print('rx_current : ', rx_current)
                            x_location = rx_current - width*0.125
                        else:
                            print('left')
                            print('lx_current : ', lx_current)
                            lx_current = np.int(np.median(nonzerox[good_left_inds]))
                            x_location = lx_current + width*0.125

                elif line_flag == 1:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = rx_current - margin
                    win_x_high = rx_current + margin

                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    good_right_inds = ((nonzerox >= win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= win_x_high)).nonzero()[0]
                    right_lane_inds.append(good_right_inds)
                    rx.append(rx_current)
                    ry.append(win_y_high)
                    if len(good_right_inds) > minpix:
                        rx_current = np.int(np.median(nonzerox[good_right_inds]))
                        x_location = rx_current - width*0.125
                    
                elif line_flag == 2:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = lx_current - margin
                    win_x_high = lx_current + margin
                    lx.append(lx_current)
                    ly.append(win_y_high)
                    # draw rectangle
                    # 0.33 is for width of the road
                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_left_inds = ((nonzerox >= win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= win_x_high)).nonzero()[0]
                    left_lane_inds.append(good_left_inds)
                    if len(good_left_inds) > minpix:
                        lx_current = np.int(np.median(nonzerox[good_left_inds]))
                    right_lane_inds.append(good_right_inds)
                    x_location = lx_current + width*0.125
                else:
                    break
        
        
        
        print('x_location : ', x_location)
        center = 640 - x_location # -320 ~ 320 -> 0 ~ 1
        steering = center / 640

        return out_img, steering

