#!/usr/bin/env python
# -*- coding: utf-8 -*- 15
#=============================================
# 본 프로그램은 자이트론에서 제작한 것입니다.
# 상업라이센스에 의해 제공되므로 무단배포 및 상업적 이용을 금합니다.
# 교육과 실습 용도로만 사용가능하며 외부유출은 금지됩니다.
#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import numpy as np
import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None  # 모터 노드 변수
Fix_Speed = 15  # 모터 속도 고정 상수값 
new_angle = 0  # 모터 조향각 초기값
new_speed = Fix_Speed  # 모터 속도 초기값
ultra_msg = None  # 초음파 데이터를 담을 변수
motor_msg = xycar_motor()  # 모터 토픽 메시지

#=============================================
# 콜백함수 - 초음파 토픽을 받아서 처리하는 콜백함수
#=============================================
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

#=============================================
# 모터 토픽을 발행하는 함수 
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)
    
#=============================================
# 초음파 센서를 이용해서 벽까지의 거리를 알아내서
# 벽과 충돌하지 않으며 주행하도록 모터로 토픽을 보내는 함수
#=============================================
def sonic_drive():
    
    # 왼쪽 벽이 멀리 있으면, 왼쪽으로 주행
    if (ultra_msg[1] > ultra_msg[3]):
        angle = -50

    # 오른쪽 벽이 멀리 있으면, 오른쪽으로 주행
    elif (ultra_msg[1] < ultra_msg[3]):
        angle = 50

    # 위 조건들에 해당하지 않는 경우라면 직진 주행
    else:
        angle = 0

    # angle 값을 반환
    return angle

#=============================================
# 실질적인 메인 함수 
#=============================================
def start():
    global motor
    global Fix_Speed
    
    #=========================================
    # 노드를 생성하고, 구독/발행할 토픽들을 선언합니다.
    #=========================================
    rospy.init_node('ultra_driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size=1)
    #Fix_Speed = rospy.get_param('~speed', 15)

    #=========================================
    # 발행자 노드들로부터 첫번째 토픽들이 도착할 때까지 기다립니다.
    #=========================================
    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("UltraSonic Ready ----------")
    drive(0,0)
    time.sleep(2)
    
    # 메인루프
    while not rospy.is_shutdown():

        # 초음파 센서로 주행합니다.
        new_angle = sonic_drive()
        new_speed = Fix_Speed
        drive(new_angle, new_speed)        

#=============================================
# 메인함수를 호출합니다.
# start() 함수가 실질적인 메인함수입니다.
#=============================================
if __name__ == '__main__':
    start()
