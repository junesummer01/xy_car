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
import cv2, rospy, time, math, os
from xycar_msgs.msg import xycar_motor
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
motor = None  # 모터 노드 변수
Fix_Speed = 12  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
motor_msg = xycar_motor()  # 모터 토픽 메시지
    
detector = apriltag.Detector()
tag_size = 9.5  # AprilTag size
camera_matrix = np.array([[371.42821, 0., 310.49805],
                          [0., 372.60371, 235.74201],
                          [0., 0., 1.]])

#=============================================
# 콜백함수 - USB 카메라 토픽을 처리하는 콜백함수
#=============================================
def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수  
# 입력으로 지속시간을 받아 그 시간동안 속도=0 토픽을 모터로 보냄.
# 지속시간은 1초 단위임. 
#=============================================
def stop_car(duration):
    for i in range(int(duration*10)): 
        drive(angle=0, speed=0)
        time.sleep(0.1)

#=============================================
# 카메라 Exposure값을 변경하는 함수
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
    
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
            left_pixels = abs(detection.corners[0][1] - detection.corners[3][1])
            right_pixels = abs(detection.corners[1][1] - detection.corners[2][1])
            tag_size_pixels = (left_pixels + right_pixels) // 2

            # 보정작업을 통해 거리값을 구합니다
            distance = ((camera_matrix[0, 0] * tag_size) / tag_size_pixels) * 100/126
            distance_to_tag_cm = distance * 1.1494 - 14.94

            # 센터의 X좌표를 이용해서 보정작업을 통해 X변위값을 구합니다
            x_offset_pixels = center[0] - 320
            x_offset_pixels = center[0] - 320
            x_offset_cm = ((x_offset_pixels * tag_size) / tag_size_pixels) * 1

            # 발견된 모든 AR태그의 정보를 수집하여 리스트에 담습니다.
            ar_msg["ID"].append(detection.tag_id)  # ID값
            ar_msg["DZ"].append(distance_to_tag_cm) # Z거리값
            ar_msg["DX"].append(x_offset_cm) # X거리값
            
    cv2.imshow('Detected AprilTags', image)
    cv2.waitKey(1)

    return ar_msg       
        
#=============================================
# AprilTag Detector를 구동하여 정보를 수집한 후  
# 제일 가까이에 있는 AprilTag의 정보를 반환하는 함수
# ID값과 함께 거리값과 좌우치우침값을 같이 반환
#=============================================
def check_AR(cv_image):

    # AprilTag Detector를 구동하여 정보를 수집
    ar_data = ar_detect(cv_image)
        
    if not ar_data["ID"]:
        # 발견된 AR태그가 없으면 
        # ID값 99, Z위치값 500cm, X위치값 500cm로 리턴
        return 99, 500, 500  # 66 

    # 가장 가까운 AR태그 선택
    closest_index = ar_data["DZ"].index(min(ar_data["DZ"]))  
    tag_id = ar_data["ID"][closest_index]
    z_pos = ar_data["DZ"][closest_index]
    x_pos = ar_data["DX"][closest_index]

    # ID번호, 거리값, 좌우치우침값 리턴
    return tag_id, round(z_pos,1), round(x_pos,1)

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, new_angle, new_speed
     
    AR_DRIVE = 4
    FINISH = 9
    drive_mode = AR_DRIVE
    cam_exposure(100) # 카메라 Exposure값을 적절히 설정
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('AR_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, img_callback)

    #=========================================
    # 첫번째 토픽이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message('/usb_cam/image_raw', Image)
    print("Camera Ready ----------")
    stop_car(10)

    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # ======================================
        # AR태그를 발견하면 AR 주행을 시작합니다.
        # ======================================
        while drive_mode == AR_DRIVE:
        
            # 전방에 AR태그가 보이는지 체크합니다.   
            ar_ID, z_pos, x_pos = check_AR(cv_image)
            
            if (ar_ID == 99):
                # AR태그가 안 보이면 AR태그를 계속 찾습니다.   
                continue
    
            else:
                # AR태그가 있는 곳으로 주행합니다. 
                print("ID=", ar_ID," Z_pos=",z_pos," X_pos=",x_pos) 
                if (z_pos > 30):
                    # Z값이 ?센치 이상이면 핸들 각도를 조종하면서 주행합니다.  
                    new_angle = x_pos * 1
                    print (f"Angle: {new_angle:.1f}")
		
                else:
                    # Z값이 ?센치 이하이면 차량을 세우고 FINISH 모드로 넘어갑니다.     
                    new_angle = 0
                    new_speed = 0                      
                    drive_mode = FINISH  
                    print ("----- Parking completed... -----")

            drive(new_angle, new_speed)
            time.sleep(0.5)  # 0.5초 쉬어갑니다.

        # ======================================
        # 주행을 끝냅니다. 
        # ======================================
        if drive_mode == FINISH:
           
            # 차량을 정지시키고 모든 작업을 끝냅니다.
            stop_car(10) # 1초간 정차 
            print ("----- Bye~! -----")
            return            

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
