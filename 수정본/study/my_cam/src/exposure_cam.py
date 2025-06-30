#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

#=============================================
# 콜백함수 - USB 카메라 토픽을 받아서 처리하는 콜백함수
#=============================================
def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

#=============================================
# 카메라 Exposure값을 변경하는 함수
#=============================================
def cam_exposure(value):
    command = 'v4l2-ctl -d /dev/videoCAM -c exposure_absolute=' + str(value)
    os.system(command)
	
rospy.init_node('cam_control', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

rospy.wait_for_message("/usb_cam/image_raw/", Image)
print("Camera Ready --------------")

exposure_value = 0

while not rospy.is_shutdown():

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray,(5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70)

    cv2.imshow("original", cv_image)
    cv2.imshow("gray", gray)
    cv2.imshow("gaussian blur", blur_gray)
    cv2.imshow("edge", edge_img)
    cv2.waitKey(200)

    print("Cam Exposure value = ", exposure_value)
    cam_exposure(exposure_value)
    exposure_value = (exposure_value+1) % 256
    

