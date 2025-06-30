#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import apriltag

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
bridge = CvBridge()
cv_image = None
Blue = (0,255,0) 
Green = (0,255,0)
Red = (0,0,255) 
Yellow = (0,255,255) 

#=============================================
# 콜백함수 - USB 카메라 토픽을 처리하는 콜백함수
#=============================================
def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('ar_detect')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)

    #=========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw", Image)
    print("Camera Ready")

    # AprilTag Detector를 생성
    detector = apriltag.Detector()
    camera_matrix = np.array([[371.42821, 0., 310.49805],
                              [0., 372.60371, 235.74201],
                              [0., 0., 1.]])

    # 인쇄한 AprilTag의 크기는 9.5cm
    tag_size = 9.5  # AprilTag size

    while not rospy.is_shutdown():

        if cv_image is not None:
            
            # 사진 이미지를 흑백으로 바꾸어 AprilTag를 찾습니다
            image = cv_image.copy()
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            detections = detector.detect(gray_image)

            if len(detections) != 0:
                print(f"Detected {len(detections)} tags")
            else:
                print('.', end='', flush=True)

            # 발견된 모든 AprilTag에 대해서 거리정보를 수집합니다.
            for detection in detections:
                for corner in detection.corners:
                    corner = tuple(map(int, corner))
                    cv2.circle(image, corner, 5, Green, 2)

                center = tuple(map(int, detection.center))
                #print(f"Center: {center}")
                cv2.circle(image, center, 5, Red, -1)
 
                # 양쪽 세로 모서리 길이의 평균값을 구합니다
                left_pixels = abs(detection.corners[0][1] - detection.corners[3][1])
                right_pixels = abs(detection.corners[1][1] - detection.corners[2][1])
                tag_size_pixels = (left_pixels + right_pixels) // 2

                # 보정작업을 통해 거리값을 구합니다
                distance = ((camera_matrix[0, 0] * tag_size) / tag_size_pixels) * 100/126
                distance_to_tag_cm = distance * 1.1494 - 14.94

                # 센터의 X좌표를 이용해서 보정작업을 통해 X변위값을 구합니다
                x_offset_pixels = center[0] - 320
                x_offset_cm = ((x_offset_pixels * tag_size) / tag_size_pixels) * 1

                print(f"Tag #{detection.tag_id}: Z: {distance_to_tag_cm:.1f} cm, X: {x_offset_cm:.1f} cm")
                cv2.putText(image, f"{distance_to_tag_cm:.1f} cm", (center[0]+30,center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Yellow, 1)
 
            cv2.imshow('Detected AprilTags', image)
            cv2.waitKey(1)

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()

