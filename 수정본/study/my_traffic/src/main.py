#!/usr/bin/env python
# -*- coding: utf-8 -*-


# import rospy
# ## 객체 불러오기
# from basic_controller import BasicController
# from traffic_light_detector import TrafficLightDetector
# # from sonic_driver import SonicDriver
# from ultra_drive_cam import UltraDriveCamera
# from sensor_msgs.msg import Image

# if __name__ == '__main__':
#     rospy.init_node('main_controller')
    
#     ## 각 객체 정의하기
#     controller = BasicController()
#     detector = TrafficLightDetector()
#     # sonic = SonicDriver()
#     ultra = UltraDriveCamera()

#     rospy.wait_for_message("/usb_cam/image_raw/", Image)
#     rospy.loginfo("카메라 준비됨")

#     rate = rospy.Rate(10)  # 10Hz
#     executed = False

#     ## 실제 실행되는 코드 : 불러온 class객체에서 필요한 def문 사용
#     while not rospy.is_shutdown():
#         ## green light detecting
#         if detector.is_green_light_detected() and not executed:
#             rospy.loginfo("🟢 초록불 감지 - 주행 시작")
#             controller.drive(0, 60)
#             rospy.sleep(1.0)
#             controller.drive(0, 0)
#             executed = True  # 한 번만 실행되도록 플래그 설정
#         ## sonic drive (라바콘 회피)
#         if executed==True:
#             angle, speed = ultra.run()
#             controller.drive(angle, speed)
#             print(speed)
#             rate.sleep()

#         rate.sleep()

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 클래스 객체 import
from basic_controller import BasicController
from traffic_light_detector import TrafficLightDetector
from ultra_drive_cam import UltraDriveCamera

class MainController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('main_controller')

        # 객체 생성
        self.controller = BasicController()
        self.detector = TrafficLightDetector()
        self.ultra = UltraDriveCamera()

        # 카메라 초기화
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.image_callback)
        self.image = None

        # 실행 여부 체크
        self.executed = False
        self.rate = rospy.Rate(10)  # 10Hz

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Image conversion error: %s", e)

    def run(self):
        rospy.loginfo("카메라 준비 중...")
        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        rospy.loginfo("카메라 준비 완료 ✅")

        while not rospy.is_shutdown():
            if self.detector.is_green_light_detected() and not self.executed:
                rospy.loginfo("🟢 초록불 감지 - 주행 시작")
                self.controller.drive(0, 60)
                rospy.sleep(1.0)
                self.controller.drive(0, 0)
                self.executed = True

            # 초록불 이후에만 주행 시작
            if self.executed and self.image is not None:
                angle, speed = self.ultra.vision_drive(self.image)
                self.controller.drive(angle, speed)

            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = MainController()
        node.run()
    except rospy.ROSInterruptException:
        pass
