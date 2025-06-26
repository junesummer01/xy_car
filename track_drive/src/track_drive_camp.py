#!/usr/bin/env python
# -*- coding: utf-8 -*- 2
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
from ar_track_alvar_msgs.msg import AlvarMarkers

#=========================================
# 외부에 있는 아래 파일들을 사용하기 위한 import 입니다.
# artag.py / traffic.py / ultrasonic.py /
# hough.py / 
#=========================================
import artag
import traffic
import ultrasonic
import hough

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 12  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
bridge = CvBridge()  # OpenCV 함수를 사용하기 위한 브릿지 
ultra_msg = None  # 초음파 데이터를 담을 변수
image = np.empty(shape=[0])  # 카메라 이미지를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지
WIDTH, HEIGHT = 640, 480  # 카메라 이미지 가로x세로 크기
View_Center = WIDTH//2  # 화면의 중앙값 = 카메라 위치
ar_msg = {"ID":[],"DX":[],"DZ":[]}  # AR태그 토픽을 담을 변수

#=============================================
# 콜백함수 - USB 카메라 토픽을 처리하는 콜백함수
#=============================================
def usbcam_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 콜백함수 - 초음파 토픽을 받아서 처리하는 콜백함수
#=============================================
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

#=============================================
# 콜백함수 - AR 토픽을 받아서 처리하는 콜백함수
#=============================================
def ar_callback(data):
    global ar_msg

    # AR태그의 ID값, X값, Z값을 담을 빈 리스트 준비
    ar_msg["ID"] = []
    ar_msg["DX"] = []
    ar_msg["DZ"] = []

    # 발견된 모든 AR태그에 대해서 정보 수집하여 ar_msg 리스트에 담음
    for i in data.markers:
        ar_msg["ID"].append(i.id) # ID값을 리스트에 추가
        ar_msg["DX"].append(int(i.pose.pose.position.x*100)) # cm 단위로 변경
        ar_msg["DZ"].append(int(i.pose.pose.position.z*100)) # cm 단위로 변경
    
#=============================================
# 모터로 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    motor.publish(motor_msg)
    
#=============================================
# 차량을 정차시키는 함수  
# 인자로 받은 시간동안 속도=0 토픽을 모터로 보냄.
#=============================================
def stop_car(duration):
    for i in range(int(duration*10)): 
        drive(angle=0, speed=0)
        time.sleep(0.1)
        
#=============================================
# 차량을 구동시키는 함수  
# 인자로 받은 시간동안 조향각과 속도값을 담아 토픽을 모터로 보냄.
#=============================================
def move_car(duration, angle, speed):
    for i in range(int(duration*10)): 
        drive(angle, speed)
        time.sleep(0.1)
        
#=============================================
# 카메라의 Exposure 값을 변경하는 함수 
# 입력으로 0~255 값을 받을 수 있음.
# 32의 배수가 되는 순간 갑자기 어두워진다. 주의~!
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
    
#=============================================
# 실질적인 메인 함수 
#=============================================
def start():

    global motor, ultra_msg, image, angle_avg
    global new_angle, new_speed
   
    # 동작모드를 고유의 숫자값으로 구분하여 지정합니다. 
    # 각 동작모드마다 while 블럭이 하나씩 존재합니다. 
    TRAFFIC_SIGN = 1
    SENSOR_DRIVE = 2
    AR_DRIVE = 3
    LANE_DRIVE = 4 
    
    # 아래 while 블럭중에 처음에 어디로 들어갈지 결정합니다. 
    drive_mode = TRAFFIC_SIGN
    
    # 아래에서 사용할 로컬변수의 초기값을 여기서 세팅합니다.
    go_next = False
    count = 0

    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, ar_callback, queue_size=1 )
    
    #=========================================
    # 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")
    rospy.wait_for_message("ar_pose_marker", AlvarMarkers)
    print("AR detector Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    #============================================
    # 바퀴 구동 테스트를 위해 
    # 1초 동안 정차, 1초간 회전을 두번 반복합니다.
    #============================================  
    stop_car(1.0)  # 1초 동안 정차
    move_car(1.0, 0, 50)  # 1초 주행        
    stop_car(1.0)  # 1초 동안 정차
    move_car(1.0, 0, 50)  # 1초 주행     
    stop_car(1.0)  # 1초 동안 정차

    #============================================
    # 카메라 노출값을 세팅합니다. 
    # 사진을 어둡게도, 환하게도 만들 수 있습니다.
    #============================================
    cam_exposure(120)  # 카메라의 Exposure 값을 변경

    #=========================================
    # 메인 루프 
    #=========================================
    while not rospy.is_shutdown():

        # ======================================
        # 전방에서 신호등을 찾고 파란색 불이 켜지면 출발합니다.
        # 다음 순서인 SENSOR_DRIVE 모드로 넘어갑니다.  
        # ======================================
        while drive_mode == TRAFFIC_SIGN:

            # 잠깐씩 쉬었다가 다음으로 넘어갑니다.  
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            time.sleep(0.3)

            # 신호등이 있는지, 있다면 어떤 불이 켜졌는지 체크합니다.  
            flag, color = traffic.check_traffic_sign(image)
            
            # 신호등을 찾았고, 파란불이 켜졌으면 다음 모드로 넘어갑니다.
            if (flag == True) and (color == 'Blue'):
                print("#======================================#") 
                print("#  Traffic sign is Blue! -- Go Next!   #") 
                print("#======================================#") 
                go_next = True
            
            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다. 시간은 변경할 수 있습니다.
                cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = SENSOR_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(120)  # 카메라의 노출값을 적절하게 변경합니다.
                print ("----- Sensor driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # 거리센서를 이용하여 라바콘 사이를 주행합니다.
        # 주행 중에 전방에 AR태그가 있는지 계속 체크합니다.
        # AR태그가 가까이 보이면 
        # 다음 순서인 AR_DRIVE 모드로 넘어갑니다. 
        # ======================================
        while drive_mode == SENSOR_DRIVE:

            # 카메라 영상을 화면에 표시합니다.
            cv2.imshow('Sensor Driving', image)
            cv2.waitKey(1)

            # 잠깐씩 쉬었다가 다음으로 넘어갑니다.  
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            time.sleep(0.3)
            
            # 초음파센서 데이터를 이용해서 주행을 위한 핸들값을 찾아냅니다. 
            new_angle = ultrasonic.sonic_drive(ultra_msg)  
            
            # 주행 속도값을 세팅합니다. 
            new_speed = Fix_Speed

            # 위에서 결정된 핸들값과 속도값을 토픽에 담아 모터로 보냅니다.
            drive(new_angle, new_speed)

            # 전방에 AR태그가 있는지 체크합니다. 
            ar_ID, z_pos, x_pos = artag.check_AR(ar_msg)            

            # AR태그가 안보이면 발견하지 못했다는 메시지를 출력합니다.  
            if (ar_ID == 99):
                print("......Cannot find AR nearby...",count) 
                count = count+1

            # AR태그가 발견되면 여기로 들어갑니다. 
            else:

                # 발견된 AR태그의 ID값과 Z거리값, X거리값을 출력합니다.  
                print("ID=",ar_ID," Z=",z_pos," X=",x_pos,sep='') 

                # Z값이 100센치보다 멀면 메시지 출력하고 라바콘 주행을 계속합니다. 
                if (z_pos > 100):
                    print ("....AR found, but it's far away",count)
                    count = count+1
                    
                # Z값이 100센치보다 가까우면 다음 모드로 넘어갑니다.
                else:
                    print("#=================================#") 
                    print("#  Nearby AR found! -- Go Next!   #")
                    print("#=================================#") 
                    go_next = True
                  
            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다. 시간은 변경할 수 있습니다.
                cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = AR_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(120)  # 카메라의 노출값을 적절하게 변경합니다.
                print ("----- AR driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # AR박스를 발견하면 가까이 다가가서 정차합니다.
        # 다음 순서인 LANE_DRIVE 모드로 넘어갑니다.          
        # ======================================
        while drive_mode == AR_DRIVE:
        
            # 카메라 영상을 화면에 표시합니다.
            cv2.imshow('AR driving', image)
            cv2.waitKey(1)
                    
            # 잠깐씩 쉬었다가 다음으로 넘어갑니다.  
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            time.sleep(0.3)
                  
            # 전방에 AR태그가 보이는지 체크합니다.   
            ar_ID, z_pos, x_pos = artag.check_AR(ar_msg)            
                     
            if (ar_ID == 99):
                # AR태그가 안 보이면 AR태그를 계속 찾습니다.   
                continue
    
            else:
                # AR태그가 있는 곳으로 주행합니다. 
                print("ID=", ar_ID," Z_pos=",z_pos," X_pos=",x_pos) 
                if (z_pos > 40):
                    # Z값이 40센치 이상이면 핸들 각도를 조종하면서 주행합니다.  
                    new_angle = x_pos * 3
                    print ('Angle:', new_angle)
                    new_speed = Fix_Speed
                    drive(new_angle, new_speed)
					
                else:
                    # Z값이 40센치 이하이면 차량을 세우고 다음 모드로 넘어갑니다.     
                    print("#=================================#") 
                    print("#  AR close enough! -- Go Next!   #")
                    print("#=================================#") 
                    go_next = True

            # 아래 if 블럭에는 go_next가 True로 변경된 경우에만 들어갈 수 있습니다.
            if (go_next == True):                
                stop_car(2.0)  # 차량을 멈춥니다. 시간은 변경할 수 있습니다.
                cv2.destroyAllWindows()  # OpenCV 창을 모두 닫습니다. 
                drive_mode = LANE_DRIVE  # 다음 모드로 넘어갑니다.
                cam_exposure(120)  # 카메라의 노출값을 적절하게 변경합니다.
                print ("----- Lane driving starts... -----")
                go_next = False  # 다음을 위해 원래대로 False로 바꿉니다.
                count = 0

        # ======================================
        # 차선을 인식해서 차선을 따라 주행합니다. 
        # 중간에 장애물이 있으면 회피하여 주행합니다. 
        # 최대한 빠르게 주행합니다.
        # ======================================
        while drive_mode == LANE_DRIVE:

            # 잠깐씩 쉬었다가 다음으로 넘어갑니다.  
            # 개발단계에서의 편의를 위해 삽입된 코드입니다. 
            # 코드 개발이 끝난후 꼭 필요하지 않으면 삭제하세요.             
            time.sleep(0.3)
        
            # 카메라 영상에서 차선의 위치를 알아냅니다. 
            found, x_left, x_right = hough.lane_detect(image)

            # 차선인식이 됐으면 핸들값과 속도값을 결정합니다.
            if found:
                
                #====================================================
                # 차선과 차량의 위치차이로 핸들값을 계산합니다.
                #  * 위치차이를 핸들값으로 변환할때 가중치 조정이 필요합니다.
                #  * 코너주행에서는 계산법이 달라져야 합니다.
                #====================================================
                x_midpoint = (x_left+x_right)//2  # 차선의 중점 구하기
                new_angle = (x_midpoint-View_Center)*1.0  # 핸들값 계산

                #====================================================
                # 속도값을 적절하게 세팅합니다. 
                #  * 최대한 빨리 주행하게 만들어 봅니다.
                #  * 직진구간에서는 코너보다 빠르게 주행하게 하면 좋습니다.
                #====================================================
                new_speed = Fix_Speed
                
                # 위에서 결정된 핸들값과 속도값을 토픽에 담아 모터로 보냅니다.
                drive(new_angle, new_speed)  

            # 차선인식이 안됐으면 가던대로 갑니다.
            else:            
                print("Lane finding fails... Keep going!", count)
                count = count+1
                            
                # 기존의 핸들값과 속도값을 그대로 다시 한번 토픽에 담아 모터로 보냅니다.
                drive(new_angle, new_speed)
                
        # ======================================
        # 주행을 끝냅니다. 
        # ======================================
        if drive_mode == FINISH:
           
            # 차량을 정지시키고 모든 작업을 끝냅니다.
            stop_car(2.0) # 차량을 멈춥니다.
            print ("----- Bye~! -----")
            return            

    # ROS 실행이 종료되면 여기로 와서 아래 코드가 실행됩니다.  
    stop_car(2.0)  # 차량을 멈춥니다.  

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
