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

        out_img = np.dstack((img, img, img))
        # out_img = img
        height = img.shape[0]
        width = img.shape[1]

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
        
        
        # first location and segmenation location finder
        # draw line
        # 120 120,60 180,80, 180
        # pts_left = np.array([[width/2 - 120, height], [width/2 - 120, 250], [width/2 - 260, 250], [width/2 - 260, height]], np.int32)
        # cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)
        pts_left = np.array([[left_lane_start_x,height],[left_lane_start_x,height*0.6],[left_lane_end_x, height*0.6],[left_lane_end_x,height]],np.int32)
        cv2.polylines(out_img, [pts_left], False, (0,255,0), 1)


        # pts_right = np.array([[width/2 + 120, height], [width/2 + 120, 300], [width/2 + 260, 300], [width/2 + 260, height]], np.int32)
        # cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        pts_right = np.array([[right_lane_start_x,height],[right_lane_start_x,height*0.6],[right_lane_end_x,height*0.6],[right_lane_end_x,height]],np.int32)
        cv2.polylines(out_img, [pts_right], False, (255,0,0), 1)
        #pts_center = np.array([[width/2 + 90, height], [width/2 + 90, height - 150], [width/2 - 60, height - 231], [width/2 - 60, height]], np.int32)
        #cv2.polylines(out_img, [pts_center], False, (0,0,255), 1)
        # pts_catch = np.array([[0, 340], [width, 340]], np.int32)
        # cv2.polylines(out_img, [pts_catch], False, (0,120,120), 1)

        # indicies before start line(the region of pts_left)
        # nonzerox * 0.33 +
        # 130 337 70
        # good_left_inds = ((nonzerox >= width/2 - 300) & (nonzeroy >=  350 - nonzerox * 0.33 ) & (nonzerox <= width/2 - 110)).nonzero()[0]
        # good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  height*0.65 ) & (nonzerox <= left_lane_end_x)).nonzero()[0]
        good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  height*0.65 ) & (nonzerox <= left_lane_end_x)).nonzero()[0]
        # good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzerox <= left_lane_end_x)).nonzero()[0]

        # good_right_inds = ((nonzerox >= width/2 + 110) & (nonzeroy >= 300) & (nonzerox <= width/2 - 110)).nonzero()[0]
        good_right_inds = ((nonzerox >= right_lane_start_x) & (nonzeroy >= height*0.65) & (nonzerox <= right_lane_end_x)).nonzero()[0]
        # good_right_inds = ((nonzerox >= right_lane_start_x) & (nonzerox <= right_lane_end_x)).nonzero()[0]
        # print(type(good_left_inds))
        # print("good_left_inds", good_right_inds)

        # left line exist, lefty current init
        line_exist_flag = None
        y_current = None
        x_current = None
        good_center_inds = None
        p_cut = None


        # check the minpix before left start line
        # if minpix is enough on left, draw left, then draw right depends on left
        # else draw right, then draw left depends on right
        # if len(good_left_inds) > minpix:
        #     line_flag = 1
        #     x_current = np.int(np.mean(nonzerox[good_left_inds]))
        #     y_current = np.int(np.mean(nonzeroy[good_left_inds]))
        #     max_y = y_current
        # elif len(good_right_inds) > minpix:
        #     line_flag = 2
        #     x_current = nonzerox[good_right_inds[np.argmax(nonzeroy[good_right_inds])]]
        #     y_current = np.int(np.max(nonzeroy[good_right_inds]))
        # else:
        #     line_flag = 3
        #     # indicies before start line(the region of pts_center)
            # good_center_inds = ((nonzeroy >= nonzerox * 0.45 + 132) & (nonzerox >= width/2 - 60) & (nonzerox <= width/2 + 90)).nonzero()[0]
            # p_cut is for the multi-demensional function
            # but curve is mostly always quadratic function so i used polyfit( , ,2)
        #    if nonzeroy[good_center_inds] != [] and nonzerox[good_center_inds] != []:
        #        p_cut = np.polyfit(nonzeroy[good_center_inds], nonzerox[good_center_inds], 2)
        if len(good_right_inds) > minpix and len(good_left_inds) > minpix:
            line_flag = 0
            rx_init = np.int(np.median(nonzerox[good_right_inds]))
            ry_current = np.int(np.median(nonzeroy[good_right_inds]))
            lx_init = np.int(np.median(nonzerox[good_left_inds]))
            ly_current = np.int(np.median(nonzeroy[good_left_inds]))
            x_current = (rx_init + lx_init) // 2 
            y_current = (ry_current + ly_current) // 2
        elif len(good_right_inds) > minpix:
            line_flag = 1
            x_init = np.int(np.mean(nonzerox[good_right_inds]))
            y_current = np.int(np.mean(nonzeroy[good_right_inds]))
        elif len(good_left_inds) > minpix:
            line_flag = 2
            x_init = np.int(np.mean(nonzerox[good_left_inds])) 
            y_current = np.int(np.mean(nonzeroy[good_left_inds]))

        else:
            line_flag = 3
            pass       
        print("x_current : ", x_current)

        if x_current is None:
            cv2.waitKey(10)

        if line_flag != 3:
            # it's just for visualization of the valid inds in the region
            for i in range(len(good_left_inds)):
                    # img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 5, (0,255,0), -1)
                    img = cv2.circle(out_img, (nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]]), 5, (0,255,0), -1)
                    img = cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 5, (0,0,255), -1)
                    # print(nonzerox[good_left_inds[i]], nonzeroy[good_left_inds[i]])
            #
            #
            # for i in range(len(good_right_inds)):
            #         img = cv2.circle(out_img, (nonzerox[good_right_inds[i]], nonzeroy[good_right_inds[i]]), 1, (0,0,255), -1)            # window sliding and draw
            if line_flag == 0:
                lx_current, rx_current = lx_init, rx_init
            for window in range(0, nwindows):
                if line_flag == 0:
                    
                    print('both good')
                    # rectangle x,y range init
                    # good_left_inds = ((nonzerox >= left_lane_start_x) & (nonzeroy >=  height*0.85 ) & (nonzerox <= left_lane_end_x)).nonzero()[0]
                    good_tmp_left_inds = ((nonzerox >= lx_current - margin) & (nonzeroy >=  ly_current - (window + 1) * window_height ) & (nonzerox <= lx_current + margin)).nonzero()[0]
                    good_tmp_right_inds = ((nonzerox >= rx_current - margin) & (nonzeroy >=  ry_current - (window + 1) * window_height ) & (nonzerox <= rx_current + margin)).nonzero()[0]

                    # LX_win_y_low = np.int(np.mean(nonzeroy[good_tmp_left_inds])) - (window + 1) * window_height
                    # LX_win_y_high = np.int(np.mean(nonzeroy[good_tmp_left_inds])) - (window) * window_height
                    LX_win_x_low = np.int(np.median(nonzerox[good_tmp_left_inds])) - margin
                    LX_win_x_high = np.int(np.median(nonzerox[good_tmp_left_inds])) + margin
                    lx_current = (LX_win_x_low + LX_win_x_high) // 2

                    # RX_win_y_low = np.int(np.mean(nonzeroy[good_tmp_right_inds])) - (window + 1) * window_height
                    # RX_win_y_high = np.int(np.mean(nonzeroy[good_tmp_right_inds])) - (window) * window_height
                    RX_win_x_low = np.int(np.mean(nonzerox[good_tmp_right_inds])) - margin
                    RX_win_x_high = np.int(np.mean(nonzerox[good_tmp_right_inds])) + margin
                    rx_current = (RX_win_x_low + RX_win_x_high) // 2

                    LX_win_y_low = ly_current - (window + 1) * window_height
                    LX_win_y_high = ly_current - (window) * window_height
                    # LX_win_x_low = lx_current - margin
                    # LX_win_x_high = lx_current + margin

                    RX_win_y_low = ry_current - (window + 1) * window_height
                    RX_win_y_high = ry_current - (window) * window_height
                    # RX_win_x_low = rx_current - margin
                    # RX_win_x_high = rx_current + margin

                    # draw rectangle
                    # 0.33 is for width of the road
                    img = cv2.rectangle(out_img, (LX_win_x_low, LX_win_y_low), (LX_win_x_high, LX_win_y_high), (0, 255, 0), 1)
                    img = cv2.rectangle(out_img, (RX_win_x_low, RX_win_y_low), (RX_win_x_high, RX_win_y_high), (255, 0, 0), 1)
                    if RX_win_y_low >= height*0.5 and RX_win_y_low < height:
                        x_location = (RX_win_x_low + LX_win_x_low) // 2
                        print('x_location : ', x_location)
                    if x_location is not None:
                        img = cv2.circle(out_img, (x_location,RX_win_y_low), 5, (0,0,255), -1)
                    # img = cv2.circle(out_img, (x_location,(lx_current+rx_current)//2), 5, (0,0,255), -1)
                elif line_flag == 1:
                    print('right good')
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                    if win_y_low >= height*0.7 and win_y_low < height:
                        x_location = x_current
                    img = cv2.circle(out_img, (x_location,y_current), 5, (0,0,255), -1)
                elif line_flag == 2:
                    print('left good')
                    # rectangle x,y range init
                    win_y_low = y_current - (window + 1) * window_height
                    win_y_high = y_current - (window) * window_height
                    win_x_low = x_current - margin
                    win_x_high = x_current + margin
                    # draw rectangle
                    # 0.33 is for width of the road
                    img = cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                    if win_y_low >= height*0.7 and win_y_low < height:
                        x_location = x_current
                    img = cv2.circle(out_img, (x_location,y_current), 5, (0,0,255), -1)
                else:
                    pass
                # if line_flag == 1:
                #     # print("left good")
                #     # rectangle x,y range init
                #     win_y_low = y_current - (window + 1) * window_height
                #     win_y_high = y_current - (window) * window_height
                #     win_x_low = x_current - margin
                #     win_x_high = x_current + margin
                #     # draw rectangle
                #     # 0.33 is for width of the road
                #     cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                #     cv2.rectangle(out_img, (win_x_low + int(width * 0.50), win_y_low), (win_x_high + int(width * 0.50), win_y_high), (255, 0, 0), 1)
                #     # indicies of dots in nonzerox in one square
                #     good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                #     # check num of indicies in square and put next location to current
                #     if len(good_left_inds) > minpix:
                #         x_current = np.int(np.mean(nonzerox[good_left_inds]))

                #     elif nonzeroy[left_lane_inds] != [] and nonzerox[left_lane_inds] != []:
                #         p_left = np.polyfit(nonzeroy[left_lane_inds], nonzerox[left_lane_inds], 2)
                #         x_current = np.int(np.polyval(p_left, win_y_high))
                #     # 338~344 is for recognize line which is yellow line in processed image(you can check in imshow)
                #     if win_y_low >= 108 and win_y_low < 504:
                #     # 0.165 is the half of the road(0.33)
                #         x_location = x_current + int(width * 0.25)
                # else: # change line from left to right above(if)
                #     win_y_low = y_current - (window + 1) * window_height
                #     win_y_high = y_current - (window) * window_height
                #     win_x_low = x_current - margin
                #     win_x_high = x_current + margin
                #     cv2.rectangle(out_img, (win_x_low - int(width * 0.50), win_y_low), (win_x_high - int(width * 0.50), win_y_high), (0, 255, 0), 1)
                #     cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
                #     good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                #     if len(good_right_inds) > minpix:
                #         x_current = np.int(np.mean(nonzerox[good_right_inds]))
                #     elif nonzeroy[right_lane_inds] != [] and nonzerox[right_lane_inds] != []:
                #         p_right = np.polyfit(nonzeroy[right_lane_inds], nonzerox[right_lane_inds], 2)
                #         x_current = np.int(np.polyval(p_right, win_y_high))
                #     if win_y_low >= 338 and win_y_low < 344:
                #     # 0.165 is the half of the road(0.33)
                #         x_location = x_current - int(width * 0.25)

                # cv2.circle(out_img, (x_current,y_current), 1, (0,0,255), -1)
                # left_lane_inds.extend(good_left_inds)
        #        right_lane_inds.extend(good_right_inds)

            #left_lane_inds = np.concatenate(left_lane_inds)
            #right_lane_inds = np.concatenate(right_lane_inds)

        #else:
            """
            # it's just for visualization of the valid inds in the region
            # for i in range(len(good_center_inds)):
            #     img = cv2.circle(out_img, (nonzerox[good_center_inds[i]], nonzeroy[good_center_inds[i]]), 1, (0,0,255), -1)
            # try:
            #    for window in range(0, nwindows):
            #        x_current = int(np.polyval(p_cut, max_y - window * window_height))
            #        if x_current - margin >= 0:
            #            win_x_low = x_current - margin
            #            win_x_high = x_current + margin
            #            win_y_low = max_y - (window + 1) * window_height
            #            win_y_high = max_y - (window) * window_height

            #            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)
            #            cv2.rectangle(out_img, (win_x_low - int(width * 0.23), win_y_low), (win_x_high - int(width * 0.23), win_y_high), (0, 255, 0), 1)
            #            if win_y_low >= 338 and win_y_low < 344:
            #                x_location = x_current - int(width * 0.115)
            #except:
            #    pass
            """
        # pts_center = np.array([[width//2,0],[width//2,height]])
        # cv2.polylines(out_img,[pts_center],False,(0,120,120),1)

        # if x_location is not None :
        #     cv2.circle(out_img,(x_location,340),5,(255,255,255))


        return out_img, x_location