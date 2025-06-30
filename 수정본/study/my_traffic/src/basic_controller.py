#!/usr/bin/env python

import rospy
from xycar_msgs.msg import xycar_motor
class BasicController:
    def __init__(self):
        self.motor_pub=rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)
        self.motor_msg = xycar_motor()  # 모터 토픽 메시지
    def drive(self, angle, speed):
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)