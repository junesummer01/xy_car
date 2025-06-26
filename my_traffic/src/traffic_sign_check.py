#!/usr/bin/env python
# -*- coding: utf-8 -*- 1
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, os, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
image = np.empty(shape=[0])
Green = (0,255,0)
Red = (0,0,255)
Yellow = (0,255,255)

#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 카메라 Exposure값을 변경하는 함수
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
	
#=============================================
# 신호등의 파란불을 체크해서 True/False 값을 반환하는 함수
#=============================================
def check_traffic_sign(image):
    MIN_RADIUS, MAX_RADIUS = 15, 25
    
    # 원본이미지를 복제한 후에 특정영역(ROI Area)을 잘라내기
    cimg = image.copy()
    Center_X, Center_Y = 320, 100  # ROI 영역의 중심위치 좌표 
    XX, YY = 220, 80  # 위 중심 좌표에서 좌우로 XX, 상하로 YY만큼씩 벌려서 ROI 영역을 잘라냄   

    # ROI 영역에 녹색 사각형으로 테두리를 쳐서 표시함 
    cv2.rectangle(cimg, (Center_X-XX, Center_Y-YY), (Center_X+XX, Center_Y+YY), Green, 2)
	
    # 원본 이미지에서 ROI 영역만큼 잘라서 roi_img에 담음 
    roi_img = cimg[Center_Y-YY:Center_Y+YY, Center_X-XX:Center_X+XX]

    # roi_img 칼라 이미지를 회색 이미지로 바꾸고 노이즈 제거를 위해 블러링 처리를 함  
    img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(img, (5, 5), 0)

    # Hough Circle 함수를 이용해서 이미지에서 원을 (여러개) 찾음 
    circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                  param1=40, param2=20, 
                  minRadius=MIN_RADIUS, maxRadius=MAX_RADIUS)

    # 디버깅을 위해서 Canny 처리를 했을때의 모습을 화면에 표시함
    # 위 HoughCircles에서 param1, param2에 사용했던 값을 아래 canny에서 똑같이 적용해야 함. 순서 조심.
    canny = cv2.Canny(blur, 20, 40)  # 주의: param1, param2 순서가 바뀜
    cv2.imshow('Canny image used by HoughCircles', canny)
    cv2.waitKey(1)

    if circles is not None:
    
        # 정수값으로 바꾸고 발견된 원의 개수를 출력
        circles = np.round(circles[0, :]).astype("int")
        print("\nFound",len(circles),"circles")
        
        # 중심의 Y좌표값 순서대로 소팅해서 따로 저장
        y_circles = sorted(circles, key=lambda circle: circle[1])
 
        # 중심의 X좌표값 순서대로 소팅해서 circles에 다시 저장
        circles = sorted(circles, key=lambda circle: circle[0])
         
        # 발견된 원들에 대해서 루프를 돌면서 하나씩 녹색으로 그리기 
        for i, (x, y, r) in enumerate(circles):
            cv2.circle(cimg, (x+Center_X-XX, y+Center_Y-YY), r, Green, 2)
 
    # 이미지에서 정확하게 3개의 원이 발견됐다면 신호등 찾는 작업을 진행  
    if (circles is not None) and (len(circles)==3):
         
        # 가장 밝은 원을 찾을 때 사용할 리스트 선언
        brightness_values = []       

        # 발견된 원들에 대해서 루프를 돌면서 하나씩 처리 
 	    # 원의 중심좌표, 반지름. 내부밝기 정보를 구해서 화면에 출력
        for i, (x,y,r) in enumerate(circles):
            roi = img[y-(r//2):y+(r//2),x-(r//2):x+(r//2)]
            # 밝기 값은 반올림해서 10의 자리수로 만들어 사용
            mean_value = round(np.mean(roi),-1)
            brightness_values.append(mean_value)
            print(f"Circle {i} at ({x},{y}), radius={r}: brightness={mean_value}")
			                
            # 원의 밝기를 계산했던 사각형 영역을 빨간색으로 그리기 
            cv2.rectangle(cimg, ((x-(r//2))+Center_X-XX, (y-(r//2))+Center_Y-YY),
                ((x+(r//2))+Center_X-XX, (y+(r//2))+Center_Y-YY), Red, 2)

        # 가장 밝은 원의 인덱스 찾기
        brightest_idx = np.argmax(brightness_values)
        brightest_circle = circles[brightest_idx]
        x, y, r = brightest_circle

        print(f" --- Circle {brightest_idx} is the brightest.")
        cv2.circle(cimg,(x+Center_X-XX,y+Center_Y-YY),r,Yellow,2)
            
        # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
        cv2.imshow('Circles Detected', cimg)
        cv2.waitKey(1)

        # 아래와 같은 에러처리 코드가 보강되면 좋음
        #  (1)제일 위와 제일 아래에 있는 2개 원의 Y좌표값 차이가 크면 안됨 
        #  (2)제일 왼쪽과 제일 오른쪽에 있는 2개 원의 X좌표값 차이가 크면 안됨  
        #  (3)원들이 좌우로 너무 붙어 있으면 안됨 
		
		# 제일 위와 제일 아래에 있는 2개 원의 Y좌표값 차이가 크면 안됨 
        vertical_diff = MAX_RADIUS * 2
        if (y_circles[-1][1] - y_circles[0][1]) > vertical_diff:
            print("Circles are scattered vertically!")
            return False
        
        # 제일 왼쪽과 제일 오른쪽에 있는 2개 원의 X좌표값 차이가 크면 안됨 
        horizontal_diff = MAX_RADIUS * 8
        if (circles[-1][0] - circles[0][0]) > horizontal_diff:
            print("Circles are scattered horizontally!")
            return False      
            
        # 원들이 좌우로 너무 붙어 있으면 안됨 
        min_distance = MIN_RADIUS * 3
        for i in range(len(circles) - 1):
            if (circles[i+1][0] - circles[i][0]) < min_distance:
                print("Circles are too close horizontally!")
                return False 
            
        # 3개 중에서 세번째 원이 가장 밝으면 (파란색 신호등) True 리턴 
        if (max_mean_value_index == 2):
            print("Traffic Sign is Blue...!")
            return True
        
        # 첫번째나 두번째 원이 가장 밝으면 (파란색 신호등이 아니면) False 반환 
        else:
            print("Traffic Sign is NOT Blue...!")
            return False
			
        colors = ['Red', 'Yellow', 'Blue']
        return True, colors[brightest_idx]
      
    # 신호등 찾기 결과가 표시된 이미지를 화면에 출력
    cv2.imshow('Circles Detected', cimg)
    
    # 원본 이미지에서 원이 발견되지 않았다면 False 리턴   
    #print("Can't find Traffic Sign...!")
    return False, 'Black'

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Driver')
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")

    # 카메라 Exposure값을 적절히 설정
    cam_exposure(100)
	
    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # 앞에 있는 신호등에 어떤 불이 켜졌는지 체크합니다.  
        time.sleep(0.8)
        flag, color = check_traffic_sign(image)
            
        # 신호등 신호를 판별했으면 그 색상을 표시
        if (flag == True):
            print ("--- Found", color)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()

