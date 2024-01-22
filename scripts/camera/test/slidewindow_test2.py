#! /usr/bin/env python3

import cv2
import numpy as np

class SlideWindow:
    def __init__(self):
        self.left_fit = None
        self.right_fit = None
        self.now_lane = None
    def slidewindow(self, img):
        x_location = None

        out_img = np.dstack((img, img, img))

        height = img.shape[0] # 480
        width = img.shape[1] # 640

        window_height = 10
        nwindows = 30

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        margin = 20
        minpix = 10

        left_lane_inds = []
        right_lane_inds = []


        #for visualize area which is used to find lane line
        points_left = np.array([[],[],[],[]], np.int32)
        points_right = np.array([[],[],[],[]], np.int32)
        cv2.polylines(out_img, [points_left], False, (0,255,0), 1)
        cv2.polylines(out_img, [points_right], False, (255,0,0), 1)

        #find good indices in left lane and right lane
        good_window_high = 400
        good_window_low = 480
        good_left_lane_start_x = 30
        good_left_lane_end_x = 130
        good_right_lane_start_x = 310
        good_right_lane_end_x = 440

        good_left_inds = ((nonzerox >= good_left_lane_start_x) & (nonzeroy >=  good_window_high ) &(nonzeroy < good_window_low) & (nonzerox <= good_left_lane_end_x)).nonzero()[0]
        good_right_inds = ((nonzerox >= good_right_lane_start_x) & (nonzeroy >= good_window_high) &(nonzeroy < good_window_low) & (nonzerox <= good_right_lane_end_x)).nonzero()[0]
        good_right_inds_len = len(good_right_inds)
        good_left_inds_len = len(good_left_inds)

        if good_right_inds_len > minpix and good_left_inds_len > minpix:
            if good_right_inds_len >= good_left_inds_len:
                self.now_lane = "RIGHT"
                line_flag = 0
                x_current = np.int(np.median(nonzerox[good_right_inds]))
                y_current = np.int(np.median(nonzeroy[good_right_inds]))
            else:
                self.now_lane = "LEFT"
                line_flag = 1
                x_current = np.int(np.median(nonzerox[good_left_inds]))
                y_current = np.int(np.median(nonzeroy[good_left_inds]))
        elif good_right_inds_len > minpix:
            self.now_lane = "RIGHT"
            line_flag = 0
        elif good_left_inds_len > minpix:
            self.now_lane = "LEFT"
            line_flag = 1
        else:
            self.now_lane = None
            line_flag = None

        if line_flag != None:
            for window in range(0, nwindows):
                if line_flag:
                    win_y_low = height - (window + 1) * window_height
                    win_y_high = height - (window) * window_height
                    win_x_low