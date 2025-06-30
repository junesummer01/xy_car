#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import apriltag
import numpy as np
import cv2

detector = apriltag.Detector()
tag_size = 9.5  # AprilTag size
camera_matrix = np.array([[371.42821, 0., 310.49805],
                          [0., 372.60371, 235.74201],
                          [0., 0., 1.]])
Green = (0,255,0)
Red = (0,0,255)
Yellow = (0,255,255)
         
#=============================================
# 이미지에서 AR태그를 찾아 정보를 수집하는 함수
#=============================================   
def ar_detect(image):

    # AR태그의 ID값, X위치값, Z위치값을 담을 빈 리스트 준비
    ar_msg = {"ID":[],"DX":[],"DZ":[]}
    ar_msg["ID"].clear()
    ar_msg["DX"].clear()
    ar_msg["DZ"].clear()

    if image is not None:
        
        # 이미지를 흑백으로 바꾸어 AprilTag를 찾습니다
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray_image)

        if len(detections) != 0:
            print(f"Detected {len(detections)} tags")
        else:
            print('.', end='', flush=True)
            
        # 발견된 모든 AR태그에 대해서 정보 수집 
        for detection in detections:
            for corner in detection.corners:
                corner = tuple(map(int, corner))
                cv2.circle(image, corner, 5, Green, 2)

            center = tuple(map(int, detection.center))
            cv2.circle(image, center, 5, Red, -1)
 
            # 양쪽 세로 모서리 길이의 평균값을 구합니다
            tag_size_pixels = np.mean([
                abs(detection.corners[0][1] - detection.corners[3][1]),
                abs(detection.corners[1][1] - detection.corners[2][1])
            ])

            # 보정작업을 통해 거리값을 구합니다
            distance = ((camera_matrix[0, 0] * tag_size) / tag_size_pixels) * 100/126
            distance_to_tag_cm = distance * 1.1494 - 14.94

            # 센터의 X좌표를 이용해서 보정작업을 통해 X변위값을 구합니다
            x_offset_cm = ((center[0] - 320) * tag_size) / tag_size_pixels

            # 발견된 모든 AR태그의 정보를 수집하여 리스트에 담습니다.
            ar_msg["ID"].append(detection.tag_id)
            ar_msg["DZ"].append(distance_to_tag_cm)
            ar_msg["DX"].append(x_offset_cm)
            
    cv2.imshow('Detected AprilTags', image)
    cv2.waitKey(1)

    return ar_msg

#=============================================
# AprilTag Detector를 구동하여 정보를 수집한 후  
# 제일 가까이에 있는 AprilTag의 정보를 반환하는 함수
# ID값과 함께 거리값과 좌우치우침값을 같이 반환
#=============================================
def check_AR(cam_image):

    image = cam_image.copy()

    # AprilTag Detector를 구동하여 정보를 수집
    ar_data = ar_detect(image)
        
    if not ar_data["ID"]:
        # 발견된 AR태그가 없으면 
        # ID값 99, Z위치값 500cm, X위치값 500cm로 리턴
        return 99, 500, 500
		
    # 가장 가까운 AR태그 선택
    closest_index = ar_data["DZ"].index(min(ar_data["DZ"]))  
    tag_id = ar_data["ID"][closest_index]
    z_pos = ar_data["DZ"][closest_index]
    x_pos = ar_data["DX"][closest_index]

    # ID번호, 거리값, 좌우치우침값 리턴
    return tag_id, round(z_pos,1), round(x_pos,1)




