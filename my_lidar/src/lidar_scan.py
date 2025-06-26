#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import time
from sensor_msgs.msg import LaserScan

ranges = None

def lidar_callback(data):
    global ranges    
    ranges = data.ranges[1:505]

rospy.init_node('Lidar', anonymous=True)
rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

rospy.wait_for_message("/scan", LaserScan)
print("Lidar Ready ----------")

while not rospy.is_shutdown():
        
    ranges_cm = [int(distance * 100) for distance in ranges]
    step = len(ranges_cm) // 36
    print("Distances(cm):")
    print(ranges_cm[::step])
    time.sleep(0.5)
    
