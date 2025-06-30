#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 카메라 데이터를 ROS에서 받아와 빨간색 라바콘(종이컵)을 인식하고
# 중심 위치를 기반으로 조향각과 속도를 결정하여 xycar_motor로 제어
#=============================================

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor

class UltraDriveCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        self.last_cx = 320  # 화면 중심 좌표

        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.usbcam_callback)
        rospy.init_node('red_cone_vision_driver', anonymous=True)

        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        rospy.loginfo("Camera Ready --------------")

    def usbcam_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_red_cone_center(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 300:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2
                self.last_cx = cx
                return cx

        return self.last_cx

    def vision_drive(self, frame):
        cx = self.get_red_cone_center(frame)
        frame_center = frame.shape[1] // 2
        error = cx - frame_center

        rospy.loginfo(f"[Vision] Red center: {cx} (error: {error})")

        k = 0.2
        angle = -k * error

        speed = 5 if abs(error) < 40 else 30
        angle = max(min(angle, 100.0), -100.0)

        return angle, speed

    def drive(self, angle, speed):
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed
        self.motor_pub.publish(motor_msg)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image.size != 0:
                angle, speed = self.vision_drive(self.image)
                self.drive(angle, speed)

                # 시각화 (선택 사항)
                vis = self.image.copy()
                cv2.line(vis, (vis.shape[1] // 2, 0), (vis.shape[1] // 2, vis.shape[0]), (255, 255, 255), 2)
                cv2.circle(vis, (int(self.last_cx), vis.shape[0] // 2), 10, (0, 255, 0), -1)
                cv2.imshow("Red Cone Detection", vis)
                cv2.waitKey(1)
            rate.sleep()

#=============================================
# 프로그램 진입점
#=============================================
if __name__ == "__main__":
    node = UltraDriveCamera()
    node.run()
