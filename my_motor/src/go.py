#!/usr/bin/env python

import rospy, time
from xycar_msgs.msg import xycar_motor

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
motor = None
motor_msg = xycar_motor()

#=============================================
# 모터 토픽을 발행하는 함수.
#=============================================
def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

#=============================================
# 실질적인 메인 함수
#=============================================
def start():
    global motor

    #=========================================
    # ROS 노드를 생성하고 초기화
    # 토픽의 구독과 발행을 선언
    #=========================================
    rospy.init_node('driver')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    print ("----- Xycar self driving -----")

    #=========================================
    # 메인루프 - ROS가 종료될 때까지 반복
    #=========================================
    while not rospy.is_shutdown():

        for i in range(20):
            angle = 0
            speed = 0
            drive(angle, speed)
            time.sleep(0.1)

        for i in range(30):
            angle = 0
            speed = 12
            drive(angle, speed)
            time.sleep(0.1)

#=============================================
# 메인 함수 - start() 함수를 호출.
#=============================================
if __name__ == '__main__':
    start()
