#!/usr/bin/env python
# -*- coding: utf-8 -*-
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
import numpy as np
import cv2, rospy, time, os, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge

import aprilartag
import filter
import hough
import pid
import stopline
import trafficlight
import ultradrive

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None
Fix_Speed = 60
new_angle = 0
new_speed = Fix_Speed
bridge = CvBridge()
ultra_msg = None
image = np.empty(shape=[0])
motor_msg = xycar_motor()
WIDTH, HEIGHT = 640, 480
View_Center = WIDTH//2
ar_msg = {"ID":[],"DX":[],"DZ":[]}

avg_count = 25
ultra_data = [filter.MovingAverage(avg_count) for i in range(8)]

angle_avg_count = 10
angle_avg = filter.MovingAverage(angle_avg_count)

# PID 게인 조정: 빠른 응답을 위해 Kp를 높이고, Kd를 약간 높임
pid = pid.PID(kp=1.2, ki=0.001, kd=0.6)

def usbcam_callback(msg):
    global image
    image = bridge.imgmsg_to_cv2(msg, "bgr8")

def ultra_callback(msg):
    global ultra_msg
    ultra_msg = msg.data

def ultra_filtering(ultra_msg):
    global ultra_data
    for i in range(8):
        ultra_data[i].add_sample(float(ultra_msg[i]))
    ultra_list = [int(ultra_data[i].get_min()) for i in range(8)]
    return tuple(ultra_list)

def drive(angle, speed):
    motor_msg.angle = float(angle)
    motor_msg.speed = float(speed)
    if motor:
        motor.publish(motor_msg)

def stop_car(duration):
    for _ in range(int(duration*10)):
        drive(angle=0, speed=0)
        time.sleep(0.1)

def move_car(duration, angle, speed):
    for i in range(int(duration*10)): 
        drive(angle, speed)
        time.sleep(0.1)

def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)

def start():
    global motor, ultra_msg, image, angle_avg
    global new_angle, new_speed
    retry_count = 0
    View_Center = 320

    rospy.init_node('Track_Driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,usbcam_callback, queue_size=1)

    rospy.wait_for_message("/usb_cam/image_raw/", Image)
    print("Camera Ready --------------")
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")

    print("======================================")
    print(" S T A R T    D R I V I N G ...")
    print("======================================")

    STOP_LINE = 1
    TRAFFIC_LIGHT = 2
    SENSOR_DRIVE = 3
    AR_DRIVE = 4
    LANE_DRIVE = 5
    PARKING = 6
    FINISH = 7

    # stop_car(2.0)
    drive_mode = STOP_LINE
    cam_exposure(110)
    print("----- Finding Stopline starts... -----")
    go_next = False
    count = 0

    while not rospy.is_shutdown():
        while drive_mode == STOP_LINE:
            time.sleep(0.1)
            new_angle = 0
            new_speed = 30
            drive(new_angle, new_speed)
            flag = stopline.check_stopline(image)
            if flag:
                print("#======================================#")
                print("#  Found Stopline! -- Go Next!         #")
                print("#======================================#")
                go_next = True
            if go_next:
                # stop_car(2.0)
                drive_mode = TRAFFIC_LIGHT
                cam_exposure(110)
                print("----- Traffic Light Checking starts... -----")
                go_next = False
                count = 0

        while drive_mode == TRAFFIC_LIGHT:
            flag, color = trafficlight.check_traffic_light(image)
            if flag and color == 'Blue':
                print("#======================================#")
                print("#  Traffic light is Blue! -- Go Next!  #")
                print("#======================================#")
                go_next = True
            if go_next:
                # stop_car(2.0)
                cv2.destroyAllWindows()
                drive_mode = SENSOR_DRIVE
                cam_exposure(110)
                print("----- Sensor driving starts... -----")
                go_next = False
                count = 0

        while drive_mode == SENSOR_DRIVE:
            new_angle = ultradrive.sonic_drive(ultra_msg, new_angle)
            new_speed = 30
            drive(new_angle, new_speed)
            ar_ID, z_pos, x_pos = aprilartag.check_AR(image)
            if ar_ID == 99:
                count = count+1
            else:
                if z_pos > 80:
                    count = count+1
                else:
                    print(f"ID={ar_ID} Z={z_pos} X={x_pos}")
                    print("#=================================#")
                    print("#  Nearby AR found! -- Go Next!   #")
                    print("#=================================#")
                    go_next = True
            if go_next:
                # stop_car(2.0)
                drive_mode = AR_DRIVE
                cam_exposure(110)
                print("----- AR driving starts... -----")
                go_next = False
                count = 0

        while drive_mode == AR_DRIVE:
            ar_ID, z_pos, x_pos = aprilartag.check_AR(image)
            if ar_ID != 99:
                print(f"ID={ar_ID} Z={z_pos} X={x_pos}")
                retry_count = 0
                distance = math.sqrt(z_pos**2 + x_pos**2)
                if distance > 100:
                    x_pos = x_pos + 0
                    new_angle = x_pos * 1
                elif distance > 50:
                    x_pos = x_pos + 20
                    new_angle = x_pos * 2
                else:
                    x_pos = x_pos + 30
                    new_angle = x_pos * 3
                new_speed = 30
                drive(new_angle, new_speed)
            else:
                retry_count = retry_count + 1
                if retry_count == 10:
                    print("#============================#")
                    print("#  No more AR! -- Go Next!   #")
                    print("#============================#")
                    go_next = True
                else:
                    print(f"AR not found. Searching... {retry_count}")
                    if retry_count < 5:
                        new_speed = 30
                    else:
                        new_speed = 0
                    drive(new_angle, new_speed)
                    time.sleep(0.2)
            if go_next:
                # stop_car(2.0)
                cv2.destroyAllWindows()
                drive_mode = LANE_DRIVE
                cam_exposure(110)
                print("----- Lane driving starts... -----")
                go_next = False
                count = 0

        while drive_mode == LANE_DRIVE:
            found, x_left, x_right = hough.lane_detect(image)
            if found:
                count = 0
                x_midpoint = (x_left + x_right) // 2
                corner_shift = new_angle * 0.5
                new_angle = (x_midpoint - (View_Center+corner_shift)) * 0.7
                new_angle = pid.pid_control(new_angle)
                # 직진 구간 속도 40 추가 및 커브 구간 속도 조정
                if abs(new_angle) < 10:    # 매우 직진 구간
                    new_speed = 60          # 최고속도
                elif abs(new_angle) < 20:   # 직진 구간
                    new_speed = 65
                elif abs(new_angle) < 40:   # 완만한 커브
                    new_speed = 30
                else:                       # 급커브
                    new_speed = 20          # 안정성 강화를 위해 하향 조정
                drive(new_angle, new_speed)
            else:
                count = count+1
                if count < 10:
                    drive(new_angle, new_speed)
                else:
                    new_speed = 0
                    drive(new_angle, new_speed)
            print(new_speed)
            print(new_angle)

if __name__ == '__main__':
    start()
