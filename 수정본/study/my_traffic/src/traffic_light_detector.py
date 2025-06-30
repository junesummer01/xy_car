#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

class TrafficLightDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.image_callback)
        self.latest_image = None
        self.green_detected = False
        self.state_pub = rospy.Publisher("state", Int32, queue_size=1)

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"cv_bridge 변환 실패: {e}")

    def is_green_light_detected(self):
        if self.latest_image is None:
            return False

        # HSV 변환 및 마스크
        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        result = cv2.bitwise_and(self.latest_image, self.latest_image, mask=mask)
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (9, 9), 2)

        # 원 검출
        circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, 20,
                                param1=50, param2=30,
                                minRadius=10, maxRadius=100)

        output_image = self.latest_image.copy()

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv2.circle(output_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv2.circle(output_image, (i[0], i[1]), 2, (0, 0, 255), 3)

            if not self.green_detected:
                self.green_detected = True
                self.state_pub.publish(Int32(1))
                rospy.loginfo("초록불 감지됨: state=1 퍼블리시")

            # 디버깅 시각화 창
            cv2.imshow("Raw Image", self.latest_image)
            cv2.imshow("Green Mask", result)
            cv2.imshow("Detected Circles", output_image)
            cv2.waitKey(1)

            return True

        # 원 없을 때도 디버깅 창 띄우기
        cv2.imshow("Raw Image", self.latest_image)
        cv2.imshow("Green Mask", result)
        cv2.imshow("Detected Circles", output_image)
        cv2.waitKey(1)

        return False
