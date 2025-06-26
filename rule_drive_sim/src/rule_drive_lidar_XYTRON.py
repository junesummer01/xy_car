#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

motor_msg = xycar_motor()
ranges = None

def callback(msg): 
    global ranges
    ranges = msg.ranges

rospy.init_node('driver')
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber('scan', LaserScan, callback)

while not rospy.is_shutdown():

    while ranges is None:
        continue
        
    # The simulator sends 180 data from right to left 
    R = ranges[45]  
    L = ranges[135] 

    # The Xycar sends 505 data 
    # R = ranges[440]  
    # L = ranges[64] 
    
    print(R,L)
    Q = R - L
    
    angle = 0
    if Q > 0 and abs(Q) > 0.1:          
        angle = Q
    elif Q < 0 and abs(Q) > 0.1:     
        angle = Q

    motor_msg.angle = int(angle)
    motor_msg.speed = 10
    
    motor_pub.publish(motor_msg)

