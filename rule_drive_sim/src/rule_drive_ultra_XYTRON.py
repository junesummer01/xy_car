#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

motor_msg = xycar_motor()
ultrasonicData = None

def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

rospy.init_node('driver')
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, callback)
rospy.wait_for_message('xycar_ultrasonic', Int32MultiArray)

while not rospy.is_shutdown():

    L = ultrasonicData[1]  
    R = ultrasonicData[3] 
    print(R,L)
    
    angle = 0
	Q = R - L
    if Q > 0 and abs(Q) > 0.1:           
        angle = Q
    elif Q < 0 and abs(Q) > 0.1:
        angle = Q

    motor_msg.angle = int(angle)
    motor_msg.speed = 10
    motor_pub.publish(motor_msg)
	
