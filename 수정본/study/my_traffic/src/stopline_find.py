#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

image = cv2.imread('stopline.png', cv2.IMREAD_UNCHANGED)

cv2.imshow("Original Image",image)

# image(원본 이미지)의 특정영역(ROI Area)을 잘라내기
roi_img = image[300:480, 0:640]
cv2.imshow("ROI Image",roi_img)

# HSV 포맷으로 변환하고 V 채널에 대해 범위를 정해서 흑백이진화 이미지로 변환
hsv_image = cv2. cvtColor(roi_img, cv2.COLOR_BGR2HSV)
upper_white = np.array([255, 255, 255])
lower_white = np.array([0, 0, 120])
binary_img = cv2.inRange(hsv_image, lower_white, upper_white)
cv2.imshow("Black&White Binary Image",binary_img)

# 흑백 이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
stopline_check_img = binary_img[100:120, 200:440]
cv2.imshow("Stopline Check Image",stopline_check_img)

# 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
cv2.rectangle(img, (200,100),(440,120),(0,255,0),3)
cv2.imshow('Stopline Check Area', img)
    
# 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
stopline_count = cv2.countNonZero(stopline_check_img)

# 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
if stopline_count > 1500:
    print("Stopline Found...! White pixel count is", stopline_count)

cv2.waitKey()
