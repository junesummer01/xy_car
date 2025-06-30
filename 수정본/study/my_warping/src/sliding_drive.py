#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image, Imu
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
from math import *
import signal
import sys
import os

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None

CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250  # ROI row 
ROI_HEIGHT = HEIGHT - ROI_ROW
L_ROW = ROI_HEIGHT - 120  # Row for position detection
LANE_WIDTH = WIDTH * 0.8

GRAY_COLOR = (150, 150, 150)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 255, 0)
BLUE_COLOR = (255, 0, 0)
YELLOW_COLOR = (0, 255, 255)
MAGENTA_COLOR = (255, 0, 255)
CYAN_COLOR = (255, 255, 0)

IMG_BORDER = 300
N_WINDOWS = 15
WINDOW_HEIGHT = int(HEIGHT / N_WINDOWS)

CAR_PIXEL = 73
METER_PER_PIXEL = 0.055

TARGET_X_IDX = 1

first_drive_flag = True
left_found_flag, right_found_flag = [], []
prev_left_found_flag, prev_right_found_flag = False, False
nz = []

x_left_list = np.full(N_WINDOWS, 0)
x_right_list = np.full(N_WINDOWS, WIDTH)
prev_x_left_list = np.full(N_WINDOWS, 0)
prev_x_right_list = np.full(N_WINDOWS, WIDTH)

imu_callback_time = None
vel_x, vel_y, cur_vel, prev_vel = 0, 0, 0, 0


class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])

    def clear(self):
        self.data = []


MV_AVG_SIZE = 20
WINDOW_MARGIN = 60

cur_left_x_mv = MovingAverage(MV_AVG_SIZE)
cur_right_x_mv = MovingAverage(MV_AVG_SIZE)
cur_left_x_mv.add_sample(WIDTH * 0.1)
cur_right_x_mv.add_sample(WIDTH * 0.9)


# Camera Image Topic Callback
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# IMU Topic Callback for Checking Velocity of Car
def imu_callback(data):
    global imu_callback_time, vel_x, vel_y, cur_vel, prev_vel, first_drive_flag

    if imu_callback_time is None:
        dt = 0
        imu_callback_time = time.time()

        return
    else:
        dt = time.time() - imu_callback_time

    # Get Velocity of X, Y
    vel_x += (data.linear_acceleration.x * dt)
    vel_y += (data.linear_acceleration.y * dt)

    cur_vel = math.sqrt(math.pow(vel_x, 2) + math.pow(vel_y, 2))

    # Velocity == 0 if Same Velocity
    if cur_vel == prev_vel:
        vel_x, vel_y = 0, 0
        cur_vel = 0
        first_drive_flag = True

    prev_vel = cur_vel
    imu_callback_time = time.time()


# Filter Only While Lane
def binarize_image(src_img):
    blur = cv2.GaussianBlur(src_img, (5, 5), 0)
    hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
    hls_yellow_binary = cv2.inRange(hls, (20, 145, 100), (70, 255, 255))
    hls_all_binary = cv2.inRange(hls, (0, 145, 0), (255, 255, 255))
    bin_lane_img = cv2.bitwise_xor(hls_yellow_binary, hls_all_binary)

    return bin_lane_img


# Warp Image for Sliding Window
def warp_image(img):
    # Set Points of Source Image 
    top_y_offset = HEIGHT * 0.65
    below_y_offset = HEIGHT * 0.8
    tl_offset = WIDTH * 0.28
    tr_offset = WIDTH - tl_offset
    bl_offset = WIDTH * 0
    br_offset = WIDTH - bl_offset

    src_tl = [tl_offset, top_y_offset]
    src_tr = [tr_offset, top_y_offset]
    src_bl = [bl_offset, below_y_offset]
    src_br = [br_offset, below_y_offset]

    src_pt = np.float32([src_tl, src_tr, src_bl, src_br])

    # Set Points of Destination Image
    dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])

    # Get Perspective Transform Matrix
    warp_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
    warp_inverse_mat = cv2.getPerspectiveTransform(dst_pt, src_pt)
    
    # Get Translated Image
    dst_img = cv2.warpPerspective(img, warp_mat, (WIDTH, HEIGHT))
    cv2.imshow('dst_img', dst_img)

    # Border Points
    border_top_y_offset = HEIGHT * 0.65
    border_below_y_offset = HEIGHT * 0.8
    border_tl_offset = ((tl_offset * IMG_BORDER) / (WIDTH/2)) + tl_offset
    border_tr_offset = (WIDTH+2*IMG_BORDER) - border_tl_offset
    border_bl_offset = ((bl_offset * IMG_BORDER) / (WIDTH/2)) + bl_offset
    border_br_offset = (WIDTH+2*IMG_BORDER) - border_bl_offset

    border_src_tl = [border_tl_offset, border_top_y_offset]
    border_src_tr = [border_tr_offset, border_top_y_offset]
    border_src_bl = [border_bl_offset, border_below_y_offset]
    border_src_br = [border_br_offset, border_below_y_offset]
    
    border_src_pt = np.float32([border_src_tl, border_src_tr, border_src_bl, border_src_br])
    border_dst_pt = np.float32([[0, 0], [WIDTH+2*IMG_BORDER, 0], [0, HEIGHT], [WIDTH+2*IMG_BORDER, HEIGHT]])

    # Get Perspective Transform Matrix
    border_warp_mat = cv2.getPerspectiveTransform(border_src_pt, border_dst_pt)
    border_warp_inverse_mat = cv2.getPerspectiveTransform(border_dst_pt, border_src_pt)
    
    # Get Translated Image
    border_dst_img = cv2.warpPerspective(img, border_warp_mat, (WIDTH+2*IMG_BORDER, HEIGHT))

    return dst_img, border_warp_inverse_mat, src_pt


# Calc PolyFit List
def warp_process_image(src_img):
    bin_lane_img = binarize_image(src_img)
    cv2.imshow('bin_lane_img', bin_lane_img)

    x_left_list, y_left_list, x_right_list, y_right_list, sliding_img = sliding_window(bin_lane_img)

    left_fit = np.polyfit(np.array(y_left_list), np.array(x_left_list), 2)
    right_fit = np.polyfit(np.array(y_right_list), np.array(x_right_list), 2)

    return left_fit, right_fit, bin_lane_img, sliding_img


# Check is Same Lane
def check_same_lane():
    global x_left_list, x_right_list, prev_x_left_list, prev_x_right_list

    y1_point = int(N_WINDOWS * 0.8 / N_WINDOWS)
    y2_point = int(N_WINDOWS * 0.5 / N_WINDOWS)
    y3_point = int(N_WINDOWS * 0.2 / N_WINDOWS)

    left_dx1 = abs(x_left_list[y1_point] - prev_x_left_list[y1_point])
    left_dx2 = abs(x_left_list[y2_point] - prev_x_left_list[y2_point])
    left_dx3 = abs(x_left_list[y3_point] - prev_x_left_list[y3_point])
    
    right_dx1 = abs(x_right_list[y1_point] - prev_x_right_list[y1_point])
    right_dx2 = abs(x_right_list[y2_point] - prev_x_right_list[y2_point])
    right_dx3 = abs(x_right_list[y3_point] - prev_x_right_list[y3_point])

    left_diff = (left_dx1 + left_dx2 + left_dx3) / 3
    right_diff = (right_dx1 + right_dx2 + right_dx3) / 3
    MAX_AVG_GAP = 30

    if (left_diff > MAX_AVG_GAP) and (right_diff > MAX_AVG_GAP):
        print('Not All Same')
        x_left_list = prev_x_left_list[:]
        x_right_list = prev_x_right_list[:]
    
    else:
        if left_diff > MAX_AVG_GAP:
            print('Left Not Same')
            x_left_list = x_right_list[:] - np.full(len(x_right_list), LANE_WIDTH)

        if right_diff > MAX_AVG_GAP:
            print('Right Not Same')
            x_right_list = x_left_list[:] + np.full(len(x_left_list), LANE_WIDTH)


# Find Lane with Sliding Window
def sliding_window(bin_img):
    global x_left_list, x_right_list, prev_x_left_list, prev_x_right_list, cur_left_x_mv, cur_right_x_mv, nz
    global first_drive_flag, left_found_flag, right_found_flag, prev_left_found_flag, prev_right_found_flag

    HIST_FIND_HEIGHT_RATE = 0.6
    histogram = np.sum(bin_img[int(HEIGHT * HIST_FIND_HEIGHT_RATE):, :], axis=0)
    midpoint = int(WIDTH / 2)

    cur_left_x = cur_left_x_mv.get_wmm()
    cur_right_x = cur_right_x_mv.get_wmm()

    # Check is Same Start Point
    MAX_DIFF = 10

    hist_left_found = (abs(np.argmax(histogram[:midpoint]) - cur_left_x) < MAX_DIFF)
    hist_right_found = (abs(np.argmax(histogram[midpoint:]) + midpoint - cur_right_x) < MAX_DIFF)

    if hist_left_found:
        cur_left_x_mv.add_sample(np.argmax(histogram[:midpoint]))
    if hist_right_found:
        cur_right_x_mv.add_sample(np.argmax(histogram[midpoint:]) + midpoint)

    # Reset Start Point
    if first_drive_flag:
        cur_left_x = np.argmax(histogram[:midpoint])
        cur_right_x = np.argmax(histogram[midpoint:]) + midpoint

        cur_left_x_mv.clear()
        cur_right_x_mv.clear()

        print('Not All Found')

    nz = bin_img.nonzero()
    y_left_list, y_right_list = [], []

    sliding_img = np.dstack((bin_img, bin_img, bin_img)) * 255
    sliding_img = cv2.copyMakeBorder(sliding_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

    prev_left_x = cur_left_x
    prev_left_diff = 0
    left_found_flag = []

    prev_right_x = cur_right_x
    prev_right_diff = 0
    right_found_flag = []

    total_left_found_flag = False
    total_right_found_flag = False

    x_left_list, x_right_list = [], []
    WINDOW_PART_RATE = 0.2
    MIN_PIX = WINDOW_HEIGHT * WINDOW_MARGIN * WINDOW_PART_RATE * 0.25

    # Find Next Window
    for window_idx in range(N_WINDOWS):
        win_yl = HEIGHT - (window_idx + 1) * WINDOW_HEIGHT
        win_yl_part = win_yl + WINDOW_PART_RATE * WINDOW_HEIGHT
        win_yh = HEIGHT - window_idx * WINDOW_HEIGHT

        win_xll = cur_left_x - WINDOW_MARGIN
        win_xlh = cur_left_x + WINDOW_MARGIN
        win_xrl = cur_right_x - WINDOW_MARGIN
        win_xrh = cur_right_x + WINDOW_MARGIN

        left_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        right_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]

        # Both Left, Right Window Not Found
        if (len(left_found_upper_part) < MIN_PIX) and (len(right_found_upper_part) < MIN_PIX):
            cur_left_x = prev_left_x + prev_left_diff
            cur_right_x = prev_right_x + prev_right_diff
            left_found_flag.append(False)
            right_found_flag.append(False)
        
        else:
            # Left Window Found
            if len(left_found_upper_part) > MIN_PIX:
                cur_left_x = int(np.mean(nz[1][left_found_upper_part]))
                prev_left_diff = cur_left_x - prev_left_x
                left_found_flag.append(True)
                total_left_found_flag = True

            # Left Window Not Found
            else:
                cur_left_x = prev_left_x + prev_right_diff
                prev_left_diff = prev_right_diff
                if window_idx == 0:
                    cur_left_x = cur_right_x - LANE_WIDTH
                left_found_flag.append(False)

            # Right Window Found
            if len(right_found_upper_part) > MIN_PIX:
                cur_right_x = int(np.mean(nz[1][right_found_upper_part]))
                prev_right_diff = cur_right_x - prev_right_x
                right_found_flag.append(True)
                total_right_found_flag = True

            # Right Window Not Found
            else:
                cur_right_x = prev_right_x + prev_left_diff
                prev_right_diff = prev_left_diff
                if window_idx == 0:
                    cur_right_x = cur_left_x + LANE_WIDTH
                right_found_flag.append(False)

        x_left_list.append(cur_left_x)
        x_right_list.append(cur_right_x)

        y_left_list.append((win_yl + win_yh) / 2)
        y_right_list.append((win_yl + win_yh) / 2)

        prev_left_x = cur_left_x
        prev_right_x = cur_right_x

    # Check is Lane Found
    if not total_left_found_flag:
        prev_left_found_flag = False
    else:
        prev_left_found_flag = True

    if not total_right_found_flag:
        prev_right_found_flag = False
    else:
        prev_right_found_flag = True

    if not first_drive_flag:
       check_same_lane()
    else:
        prev_left_found_flag = False
        prev_right_found_flag = False

    first_drive_flag = False

    # Update Position List
    cur_left_x_mv.add_sample(x_left_list[0])
    cur_right_x_mv.add_sample(x_right_list[0])
    prev_x_left_list = x_left_list[:]
    prev_x_right_list = x_right_list[:]

    return x_left_list, y_left_list, x_right_list, y_right_list, sliding_img


# Publish Motor Topic
def motor_drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)


# Calculate Drive Angle
def get_drive_angle(dist):
    global x_left_list, x_right_list

    move_y_dist = dist * METER_PER_PIXEL
    target_y = TARGET_X_IDX * WINDOW_HEIGHT
    target_y -= move_y_dist
    idx_y = int((float(target_y) / HEIGHT) * N_WINDOWS)
    idx_y = min(max(0, idx_y), N_WINDOWS - 1)

    diff_x = ((x_left_list[idx_y] + x_right_list[idx_y]) / 2) - (WIDTH / 2)
    print('diff_x', diff_x)
        
    control_grad = diff_x / float(target_y + CAR_PIXEL)
    control_angle = math.degrees(math.atan(control_grad))
    print('control_angle', control_angle)

    return control_angle, target_y


# Fill Front Lane in Green
def fill_warped_lane(image, warp_img, border_warp_inverse_mat, left_fit, right_fit):
    warp_img = cv2.copyMakeBorder(warp_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)
    plot_y = np.linspace(0, HEIGHT - 1, HEIGHT)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)

    left_fit_x = left_fit[0] * (plot_y**2) + left_fit[1]*plot_y + left_fit[2] + IMG_BORDER
    right_fit_x = right_fit[0] * (plot_y**2) + right_fit[1]*plot_y + right_fit[2] + IMG_BORDER

    left_points = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
    right_points = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))])
    points = np.hstack((left_points, right_points))

    color_warp = cv2.fillPoly(color_warp, np.int_([points]), GREEN_COLOR)
    cv2.imshow('color_warp', color_warp)

    new_warp = cv2.warpPerspective(color_warp, border_warp_inverse_mat, (WIDTH+2*IMG_BORDER, HEIGHT))
    new_warp = new_warp[:, IMG_BORDER:IMG_BORDER+WIDTH]
    cv2.imshow('new_warp', new_warp)
    new_lane_img = cv2.addWeighted(image, 1, new_warp, 0.3, 0)

    cv2.imshow('new_lane_img', new_lane_img)


# CV Draw Function
def draw_img(src_img, src_pt, drive_angle, target_y, bin_lane_img, sliding_img):
    global x_left_list, x_right_list, left_found_flag, right_found_flag, nz

    # Draw Lines of Perspective Area
    point_img = src_img.copy()

    tup_src_tl = tuple(map(int, src_pt[0]))
    tup_src_tr = tuple(map(int, src_pt[1]))
    tup_src_bl = tuple(map(int, src_pt[2]))
    tup_src_br = tuple(map(int, src_pt[3]))

    cv2.line(point_img, tup_src_tl, tup_src_tr, RED_COLOR, 2)
    cv2.line(point_img, tup_src_tr, tup_src_br, RED_COLOR, 2)
    cv2.line(point_img, tup_src_br, tup_src_bl, RED_COLOR, 2)
    cv2.line(point_img, tup_src_bl, tup_src_tl, RED_COLOR, 2)
    cv2.imshow('perspective_point_img', point_img)

    for window_idx in range(N_WINDOWS):
        win_xll = x_left_list[window_idx] - WINDOW_MARGIN
        win_xlh = x_left_list[window_idx] + WINDOW_MARGIN
        win_xrl = x_right_list[window_idx] - WINDOW_MARGIN
        win_xrh = x_right_list[window_idx] + WINDOW_MARGIN

        win_yl = HEIGHT - (window_idx + 1) * WINDOW_HEIGHT
        win_yh = HEIGHT - window_idx * WINDOW_HEIGHT

        found_color = GREEN_COLOR
        predict_color = YELLOW_COLOR

        left_window_color = found_color if left_found_flag[window_idx] else predict_color
        right_window_color = found_color if right_found_flag[window_idx] else predict_color

        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xll), int(win_yl)), 
                        (int(IMG_BORDER + win_xlh), int(win_yh)), left_window_color, 2)
        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xrl), int(win_yl)), 
                        (int(IMG_BORDER + win_xrh), int(win_yh)), right_window_color, 2)

        if window_idx > 0:
            prev_center_x = (x_left_list[window_idx - 1] + x_right_list[window_idx - 1]) / 2
            cur_center_x = (x_left_list[window_idx] + x_right_list[window_idx]) / 2

            cv2.line(sliding_img, (int(IMG_BORDER+prev_center_x), int(win_yh+WINDOW_HEIGHT)),
                    (int(IMG_BORDER+cur_center_x), int(win_yl+WINDOW_HEIGHT)), MAGENTA_COLOR, 2)

    lane_img = cv2.cvtColor(bin_lane_img, cv2.COLOR_GRAY2BGR)
    lane_img = cv2.copyMakeBorder(lane_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

    sliding_img = cv2.bitwise_or(sliding_img, lane_img)

    cv2.line(sliding_img, (0, int(HEIGHT - target_y)), (int(IMG_BORDER*2 + WIDTH), int(HEIGHT - target_y)), GRAY_COLOR, 2)
    cv2.putText(sliding_img, 'angle: ' + str(drive_angle)[:5], (int(IMG_BORDER*2 + WIDTH * 0.7), 
                int(HEIGHT * 0.1)), 1, 1, GRAY_COLOR, 1)
    cv2.imshow('new_sliding_window', sliding_img)
    cv2.waitKey(1)


##################################################################################################################
#################################                                                #################################
#################################                 Previous Code                  #################################
#################################                                                #################################
##################################################################################################################

prev_warp_img_w = 320
prev_warp_img_h = 240

prev_warpx_WINDOW_MARGIN = 20
prev_warpy_WINDOW_MARGIN = 3

prev_nwindows = 9
prev_WINDOW_MARGIN = 12
prev_minpix = 5

######################
'''
prev_nwindows = 15
prev_WINDOW_MARGIN = 60
prev_minpix = 5
'''
######################

prev_lane_bin_th = 145

def prev_code_warp_process_image(src_img):
    global prev_nwindows, prev_WINDOW_MARGIN, prev_minpix, prev_lane_bin_th

    blur = cv2.GaussianBlur(src_img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, bin_lane_img = cv2.threshold(L, prev_lane_bin_th, 255, cv2.THRESH_BINARY)

    histogram = np.sum(bin_lane_img[bin_lane_img.shape[0]//2:,:], axis=0)      
    midpoint = int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    WINDOW_HEIGHT = int(bin_lane_img.shape[0]/prev_nwindows)
    nz = bin_lane_img.nonzero()

    left_lane_inds = []
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], []

    out_img = np.dstack((bin_lane_img, bin_lane_img, bin_lane_img))*255
    out_img = cv2.copyMakeBorder(out_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT,
                                     cv2.BORDER_CONSTANT)

    prev_left_x = leftx_current
    prev_right_x = rightx_current

    for window in range(prev_nwindows):
        win_yl = bin_lane_img.shape[0] - (window+1)*WINDOW_HEIGHT
        win_yh = bin_lane_img.shape[0] - window*WINDOW_HEIGHT

        win_xll = leftx_current - prev_WINDOW_MARGIN
        win_xlh = leftx_current + prev_WINDOW_MARGIN
        win_xrl = rightx_current - prev_WINDOW_MARGIN
        win_xrh = rightx_current + prev_WINDOW_MARGIN

        cv2.rectangle(out_img, (int(IMG_BORDER + win_xll), int(win_yl)), (int(IMG_BORDER + win_xlh), int(win_yh)),
                      (0,255,0), 2) 
        cv2.rectangle(out_img, (int(IMG_BORDER + win_xrl), int(win_yl)), (int(IMG_BORDER + win_xrh), int(win_yh)),
                      (0,255,0), 2) 

        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > prev_minpix:
            leftx_current = int(np.mean(nz[1][good_left_inds]))
        if len(good_right_inds) > prev_minpix:        
            rightx_current = int(np.mean(nz[1][good_right_inds]))

        lx.append(leftx_current)
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

        prev_center_x = (prev_left_x + prev_right_x) / 2
        cur_center_x = (leftx_current + rightx_current) / 2

        cv2.line(out_img, (int(IMG_BORDER + prev_center_x), int(win_yh)),
                 (int(IMG_BORDER + cur_center_x), int(win_yl)), MAGENTA_COLOR, 2)

        prev_left_x = leftx_current
        prev_right_x = rightx_current

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
    #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
    
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    lane_img = cv2.cvtColor(bin_lane_img, cv2.COLOR_GRAY2BGR)
    lane_img = cv2.copyMakeBorder(lane_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

    out_img = cv2.bitwise_or(out_img, lane_img)

    out_img[nz[0][left_lane_inds], IMG_BORDER + nz[1][left_lane_inds]] = BLUE_COLOR
    out_img[nz[0][right_lane_inds], IMG_BORDER + nz[1][right_lane_inds]] = RED_COLOR

    cv2.imshow("prev_sliding_img", out_img)
    
    return lfit, rfit, out_img, bin_lane_img

def prev_code_fill_warped_lane(image, warp_img, Minv, left_fit, right_fit):
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    left_points = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    right_points = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((left_points, right_points))
    
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (WIDTH, HEIGHT))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)



##################################################################################################################
##################################################################################################################
##################################################################################################################
##################################################################################################################
##################################################################################################################

def start():
    global image, motor, cur_vel

    rospy.init_node('h_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    motor_msg = xycar_motor()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    imu_sub = rospy.Subscriber('imu', Imu, imu_callback)

    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    prev_time = time.time()

    while not rospy.is_shutdown():
        warp_img, border_warp_inverse_mat, src_pt = warp_image(image)
        left_fit, right_fit, bin_lane_img, sliding_img = warp_process_image(warp_img)
        fill_warped_lane(image, warp_img, border_warp_inverse_mat, left_fit, right_fit)

        move_dist = cur_vel * (time.time() - prev_time)
        drive_angle, target_y = get_drive_angle(move_dist)
        draw_img(image, src_pt, drive_angle, target_y, bin_lane_img, sliding_img)

        # #########################################################
        # #################    Previous Code    ###################
        # #########################################################

        # prev_code_src_img = image.copy()
        # prev_code_warp_img = warp_img.copy()
        # prev_code_warp_inverse_mat = warp_inverse_mat.copy()
        # prev_code_move_dist = move_dist
        # prev_code_src_pt = src_pt
        # prev_code_left_fit, prev_code_right_fit, prev_code_sliding_img, prev_code_bin_lane_img = prev_code_warp_process_image(prev_code_warp_img)
        # prev_code_lane_img = prev_code_fill_warped_lane(prev_code_src_img, warp_img, prev_code_warp_inverse_mat, prev_code_left_fit, prev_code_right_fit)
        # prev_code_drive_angle, prev_code_target_y = get_drive_angle(prev_code_move_dist)

        # #########################################################

        motor_drive(int(drive_angle), 10)

        prev_time = time.time()


if __name__ == '__main__':
    start()
