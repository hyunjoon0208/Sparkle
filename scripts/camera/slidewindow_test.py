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

        x_location = None
        # init out_img, height, width
        angle = 0
        out_img = np.dstack((img, img, img))
        # out_img = img
        height = img.shape[0]
        width = img.shape[1]

        center = width // 2
        # num of windows and init the height
        window_height = height // 30
        nwindows = 30

        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()
        # print("nonzero : ", nonzero)
        #print nonzero
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # print(nonzerox)
        # init data need to sliding windows
        margin = 45
        minpix = 5
        # print("hello")
        left_lane_inds = []
        right_lane_inds = []
        
        left_lane_start_x = 0
        left_lane_end_x = width*0.4
        right_lane_start_x = width*0.65
        right_lane_end_x = width
        
        
        pts_left = np.array([[left_lane_start_x,height],[left_lane_start_x,height*0.6],[left_lane_end_x, height*0.6],[left_lane_end_x,height]],np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)


        pts_right = np.array([[right_lane_start_x,height],[right_lane_start_x,height*0.6],[right_lane_end_x,height*0.6],[right_lane_end_x,height]],np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  height*0.65 ) & (nonzerox <= left_lane_end_x)).nonzero()[0]
        good_right_inds = ((nonzerox >= right_lane_start_x) & (nonzeroy >= height*0.65) & (nonzerox <= right_lane_end_x)).nonzero()[0]
        line_exist_flag = None
        y_current = height
        x_current = None
        good_center_inds = None
        p_cut = None

        if len(good_right_inds) > minpix and len(good_left_inds) > minpix:
            line_flag = 0
            lx_current = np.int(np.mean(nonzerox[good_left_inds]))
            rx_current = np.int(np.mean(nonzerox[good_right_inds]))
            print('both good')
        elif len(good_right_inds) > minpix:
            line_flag = 1
            x_current = np.int(np.mean(nonzerox[good_right_inds]))
            print('right good')
        elif len(good_left_inds) > minpix:
            line_flag = 2
            x_current = np.int(np.mean(nonzerox[good_left_inds]))
            print('left good')
        else:
            print('----------------lane not found-----------------')
            line_flag = 3
        
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
                    if len(good_left_inds) > minpix and len(good_right_inds) > minpix:
                        lx_current = np.int(np.mean(nonzerox[good_left_inds]))
                        rx_current = np.int(np.mean(nonzerox[good_right_inds]))
                        x_location = (lx_current + rx_current) // 2
                    elif len(good_right_inds) > minpix:
                        rx_current = np.int(np.mean(nonzerox[good_right_inds]))
                        x_location = rx_current - width
                    elif len(good_left_inds) > minpix:
                        lx_current = np.int(np.mean(nonzerox[good_left_inds]))


                elif line_flag == 1:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin

                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    good_right_inds = ((nonzerox >= win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= win_x_high)).nonzero()[0]
                    right_lane_inds.append(good_right_inds)
                    rx.append(x_current)
                    ry.append(win_y_high)
                    if len(good_right_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_right_inds]))
                        x_location = x_current
                    
                elif line_flag == 2:
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    lx.append(x_current)
                    ly.append(win_y_high)
                    # draw rectangle
                    # 0.33 is for width of the road
                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    good_left_inds = ((nonzerox >= win_x_low) & (nonzeroy >=  win_y_low ) &(nonzeroy <= win_y_high) & (nonzerox <= win_x_high)).nonzero()[0]
                    left_lane_inds.append(good_left_inds)
                    if len(good_left_inds) > minpix:
                        x_current = np.int(np.mean(nonzerox[good_left_inds]))
                    right_lane_inds.append(good_right_inds)
                    x_location = x_current
                else:
                    break

            if line_flag == 0:
                # left_fit= np.polyfit(ly, lx, 1)
                # right_fit= np.polyfit(ry, rx, 1)
                # degree = (left_fit[0] + right_fit[0]) / 2.0
                angle = np.arctan2((cy[0] - cy[-1]), (cx[0] - cx[-1])) * 180 / np.pi
                center = (cx[0] + cx[-1]) // 2
            elif line_flag == 1:
                # right_fit= np.polyfit(ry, rx, 1)
                # degree = right_fit[0]
                angle = np.arctan2((ry[0] - ry[-1]), (rx[0] - rx[-1])) * 180 / np.pi
                center = (rx[0] + rx[-1]) // 2 - width*0.45
            elif line_flag == 2:
                # left_fit = np.polyfit(ly, lx, 1)
                # degree = left_fit[0]
                angle = np.arctan2((ly[0] - ly[-1]), (lx[0] - lx[-1])) * 180 / np.pi
                center = (lx[0] + lx[-1]) // 2 + width*0.45
            else:
                pass
                # print('----------------no degree-----------------')
            print('angle : ', angle)
            

        return out_img, angle, center