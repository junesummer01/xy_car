#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import time

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

Imu_msg = None

def imu_callback(data):
    global Imu_msg
    Imu_msg = [data.orientation.x, data.orientation.y, data.orientation.z,
               data.orientation.w] 

rospy.init_node("Imu_Print")
rospy.Subscriber("/imu/data", Imu, imu_callback)

rospy.wait_for_message("/imu/data", Imu)
print("IMU Ready ----------") 

while not rospy.is_shutdown():
    if Imu_msg == None:
        continue
    
    (roll, pitch, yaw) = euler_from_quaternion(Imu_msg)
 
    print('Roll:%.4f, Pitch:%.4f, Yaw:%.4f' % (roll, pitch, yaw))

    time.sleep(1.0)
