#! /usr/bin/env python3

import cv2
import numpy as np

class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.leftx = None
        self.rightx = None

    def slidewindow(self, img, line_flag):
        x_location = 320
        out_img = np.dstack((img, img, img))
        # out_img = img
        height = img.shape[0]
        width = img.shape[1]
        self.current_line = None
        # num of windows and init the height
        window_height = 10
        nwindows = 30
        lx_current = 160
        rx_current = 480
        l2x_current = 160
        clx_current = 160
        crx_current = 480
        cl2x_current = 160
        # find nonzero location in img, nonzerox, nonzeroy is the array flatted one dimension by x,y
        nonzero = img.nonzero()
        #print nonzero
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # init data need to sliding windows
        margin = 40
        minpix = 10

        left_lane_inds = []
        right_lane_inds = []
        
        win_y_high = 200
        win_y_low = 480

        left_lane_start_x = 40
        left_lane_end_x = 270
        
        l2_lane_start_x = 40
        l2_lane_end_x = 270
        
        right_lane_start_x = 370
        right_lane_end_x = 600

        cleft_lane_start_x = 0
        cleft_lane_end_x = 170
        cleft_win_y_high = 240
        cleft_win_y_low = 480

        c2left_lane_start_x = 0
        c2left_lane_end_x = 170
        c2left_win_y_high = 240
        c2left_win_y_low = 480


        cright_lane_start_x = 320
        cright_lane_end_x = 600
        cright_win_y_high = 280
        cright_win_y_low = 480
        
        pts_left = np.array([[left_lane_start_x,win_y_low],[left_lane_start_x,win_y_high],[left_lane_end_x, win_y_high],[left_lane_end_x,win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)

        pts_right = np.array([[right_lane_start_x,win_y_low],[right_lane_start_x,win_y_high],[right_lane_end_x,win_y_high],[right_lane_end_x,win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)

        pts_cleft = np.array([[cleft_lane_start_x,cleft_win_y_low],[cleft_lane_start_x,cleft_win_y_high],[cleft_lane_end_x, cleft_win_y_high],[cleft_lane_end_x,cleft_win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_cleft], False, (0,0,255), 1)

        pts_c2left = np.array([[c2left_lane_start_x,c2left_win_y_low],[c2left_lane_start_x,c2left_win_y_high],[c2left_lane_end_x, c2left_win_y_high],[c2left_lane_end_x,c2left_win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_c2left], False, (0,128,255), 1)

        pts_cright = np.array([[cright_lane_start_x,cright_win_y_low],[cright_lane_start_x,cright_win_y_high],[cright_lane_end_x, cright_win_y_high],[cright_lane_end_x,cright_win_y_low]],np.int32)
        cv2.polylines(out_img, [pts_cright], False, (0,255,255), 1) # yellow 

        good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  win_y_high ) &(nonzeroy < win_y_low) & (nonzerox <= left_lane_end_x)).nonzero()[0]
        good_cleft_inds = ((nonzerox >= cleft_lane_start_x) & (nonzeroy >=  win_y_high ) &(nonzeroy < win_y_low) & (nonzerox <= cleft_lane_end_x)).nonzero()[0]
        good_c2left_inds = ((nonzerox >= c2left_lane_start_x) & (nonzeroy >=  win_y_high ) &(nonzeroy < win_y_low) & (nonzerox <= c2left_lane_end_x)).nonzero()[0]
        good_right_inds = ((nonzerox >= right_lane_start_x) & (nonzeroy >= win_y_high) &(nonzeroy < win_y_low) & (nonzerox <= right_lane_end_x)).nonzero()[0]
        good_cright_inds = ((nonzerox >= cright_lane_start_x) & (nonzeroy >= cright_win_y_high) &(nonzeroy < cright_win_y_low) & (nonzerox <= cright_lane_end_x)).nonzero()[0]
        line_exist_flag = None
        y_current = height
        x_current = None
        good_center_inds = None
        p_cut = None
        cv2.circle(out_img, (320, y_current), 5, (0, 255, 255), -1)
        
        if line_flag == 'R':
            self.current_line = 'RIGHT'
            try:
                rx_current = np.int(np.mean(nonzerox[good_right_inds]))
            except:
                pass
        if line_flag == 'L':
            self.current_line = 'LEFT'
            try:
                lx_current = np.int(np.mean(nonzerox[good_left_inds]))
            except:
                pass
        
        if line_flag == 'L2':
            self.current_line = 'LEFT2'
            try:
                l2x_current = np.int(np.mean(nonzerox[good_left_inds]))
            except:
                pass
        if line_flag == 'CL':
            self.current_line = 'CORNER_LEFT'
            try:
                clx_current = np.int(np.mean(nonzerox[good_cleft_inds]))
            except:
                pass
        
        if line_flag == 'CL2':
            self.current_line = 'CORNER_LEFT2'
            try:
                cl2x_current = np.int(np.mean(nonzerox[good_c2left_inds]))
            except:
                pass

        if line_flag == 'CR':
            self.current_line = 'CORNER_RIGHT'
            try:
                crx_current = np.int(np.mean(nonzerox[good_cright_inds]))
            except:
                pass
        R_win_x_l = 370
        R_win_x_h = 600
        L_win_x_l = 40
        L_win_x_h = 270
        for window in range(0, nwindows):
            if line_flag == 'R':
                R_win_x_l = rx_current - margin
                R_win_x_h = rx_current + margin
                R_win_y_l = y_current - (window + 1) * window_height
                R_win_y_h = y_current - (window) * window_height

                img = cv2.rectangle(out_img, (R_win_x_l, R_win_y_l), (R_win_x_h, R_win_y_h), (255, 0, 0), 1)
                good_right_inds = ((nonzerox >= R_win_x_l) & (nonzeroy >= R_win_y_l  ) &(nonzeroy <= R_win_y_h) & (nonzerox <= R_win_x_h)).nonzero()[0]
                if len(good_right_inds) > 0:
                    rx_current = np.int(np.mean(nonzerox[good_right_inds]))
                x_location = rx_current - width*0.225
            
            elif line_flag == 'L':
                L_win_x_l = lx_current - margin
                L_win_x_h = lx_current + margin
                L_win_y_l = y_current - (window + 1) * window_height
                L_win_y_h = y_current - (window) * window_height

                img = cv2.rectangle(out_img, (L_win_x_l, L_win_y_l), (L_win_x_h, L_win_y_h), (0, 255, 0), 1)
                good_left_inds = ((nonzerox >= L_win_x_l) & (nonzeroy >=  L_win_y_l ) &(nonzeroy <= L_win_y_h) & (nonzerox <= L_win_x_h)).nonzero()[0]
                if len(good_left_inds) > 0:
                    lx_current = np.int(np.mean(nonzerox[good_left_inds]))
                x_location = lx_current + width*0.25

            elif line_flag == 'L2':
                L_win_x_l = l2x_current - margin
                L_win_x_h = l2x_current + margin
                L_win_y_l = y_current - (window + 1) * window_height
                L_win_y_h = y_current - (window) * window_height

                img = cv2.rectangle(out_img, (L_win_x_l, L_win_y_l), (L_win_x_h, L_win_y_h), (120, 120, 250), 1)
                good_left_inds = ((nonzerox >= L_win_x_l) & (nonzeroy >=  L_win_y_l ) &(nonzeroy <= L_win_y_h) & (nonzerox <= L_win_x_h)).nonzero()[0]
                if len(good_left_inds) > 0:
                    l2x_current = np.int(np.mean(nonzerox[good_left_inds]))
                x_location = l2x_current + width*0.265

            elif line_flag == 'CL':
                L_win_x_l = clx_current - margin
                L_win_x_h = clx_current + margin
                L_win_y_l = y_current - (window + 1) * (window_height-1)
                L_win_y_h = y_current - (window) * (window_height-1)

                img = cv2.rectangle(out_img, (L_win_x_l, L_win_y_l), (L_win_x_h, L_win_y_h), (0, 0, 255), 1)
                good_cleft_inds = ((nonzerox >= L_win_x_l) & (nonzeroy >=  L_win_y_l ) &(nonzeroy <= L_win_y_h) & (nonzerox <= L_win_x_h)).nonzero()[0]
                if len(good_cleft_inds) > 0:
                    clx_current = np.int(np.mean(nonzerox[good_cleft_inds]))
                
                x_location = clx_current + width*0.225

            elif line_flag == 'CL2':
                L_win_x_l = cl2x_current - margin
                L_win_x_h = cl2x_current + margin
                L_win_y_l = y_current - (window + 1) * (window_height-1)
                L_win_y_h = y_current - (window) * (window_height-1)

                img = cv2.rectangle(out_img, (L_win_x_l, L_win_y_l), (L_win_x_h, L_win_y_h), (0,128,255), 1)
                good_c2left_inds = ((nonzerox >= L_win_x_l) & (nonzeroy >=  L_win_y_l ) &(nonzeroy <= L_win_y_h) & (nonzerox <= L_win_x_h)).nonzero()[0]
                if len(good_c2left_inds) > 0:
                    cl2x_current = np.int(np.mean(nonzerox[good_c2left_inds]))
                
                x_location = cl2x_current + width*0.335

            elif line_flag == 'CR':
                if window >= 21:
                    break
                R_win_x_l = crx_current - margin
                R_win_x_h = crx_current + margin
                R_win_y_l = y_current - (window + 1) * (window_height)
                R_win_y_h = y_current - (window) * (window_height)

                img = cv2.rectangle(out_img, (R_win_x_l, R_win_y_l), (R_win_x_h, R_win_y_h), (0, 255, 255), 1)
                good_cright_inds = ((nonzerox >= R_win_x_l) & (nonzeroy >=  R_win_y_l ) &(nonzeroy <= R_win_y_h) & (nonzerox <= R_win_x_h)).nonzero()[0]
                if len(good_cright_inds) > 0:
                    crx_current = np.int(np.mean(nonzerox[good_cright_inds]))
                x_location = crx_current - width*0.1
        
        
        # print('x_location : ', x_location)
        return out_img, x_location, line_flag