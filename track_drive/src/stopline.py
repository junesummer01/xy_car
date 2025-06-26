#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np
import cv2

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
Blue =  (255,0,0) # 파란색
Green = (0,255,0) # 녹색
Red =   (0,0,255) # 빨간색
Yellow = (0,255,255) # 노란색
stopline_num = 1 # 정지선 발견때마다 1씩 증가

#=============================================
# 정지선이 있는지 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_stopline(image):
    global stopline_num
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    roi_img = image[300:480, 0:640]
    #cv2.imshow("ROI Image", roi_img)
    # HSV 포맷으로 변환하고 특정 범위를 정해서 흑백 이진화 이미지로 변환
    hsv_image = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV) 
    upper_white = np.array([255, 255, 255])
    lower_white = np.array([0, 0, 180])
    binary_img = cv2.inRange(hsv_image, lower_white, upper_white)

    # 흑백이진화 이미지에서 특정영역을 잘라내서 정지선 체크용 이미지로 만들기
    stopline_check_img = binary_img[100:120, 200:440] 
    
    # 흑백이진화 이미지를 칼라이미지로 바꾸고 정지선 체크용 이미지 영역을 녹색사각형으로 표시
    img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(img, (200,100),(440,120),Green,3)
    cv2.imshow('Stopline Check', img)
    cv2.waitKey(1)
    
    # 정지선 체크용 이미지에서 흰색 점의 개수 카운트하기
    stopline_count = cv2.countNonZero(stopline_check_img)
    
    # 사각형 안의 흰색 점이 기준치 이상이면 정지선을 발견한 것으로 한다
    if stopline_count > 2500:
        print("Stopline Found...! -", stopline_num)
        stopline_num = stopline_num + 1
        return True
    
    else:
        return False
