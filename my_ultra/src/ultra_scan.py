#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int32MultiArray

ultra_msg = None

def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

#=============================================
# 실질적인 메인 함수
#=============================================
def start():

    rospy.init_node("ultra_node")
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

    rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)
    print("Ultrasonic Ready ----------") 

    while not rospy.is_shutdown():

        print(ultra_msg)
        rospy.wait_for_message("xycar_ultrasonic", Int32MultiArray)

#=============================================
# 메인 함수 - start() 함수를 호출.
#=============================================
if __name__ == '__main__':
    start()
