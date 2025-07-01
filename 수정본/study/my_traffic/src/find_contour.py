#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =================================================================================
#
# FILE: hull_cluster_driver.py
#
# AUTHOR: Gemini AI
#
# DESCRIPTION:
#   [Graceful Model Degradation]
#   검출된 라바콘의 개수에 따라 차선 모델을 유연하게 변경합니다.
#   - 3개 이상: 2차 곡선 피팅 (가장 이상적)
#   - 2개: 1차 직선 피팅 (안정성 확보)
#   - 1개 이하: 유령 차선 로직 의존
#   이를 통해 매우 넓은 간격의 라바콘 코스에서도 안정성을 유지합니다.
#
# =================================================================================

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from collections import defaultdict

class HullClusterDriver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        self.last_cx = 320
        self.is_paused = False
        self.angle = 0
        self.last_error = 0
        self.LANE_WIDTH = 400

        self.LEFT_LANE_COLOR = (0, 255, 255)
        self.RIGHT_LANE_COLOR = (255, 0, 255)
        self.CENTER_LINE_COLOR = (255, 0, 0)

        self.last_successful_coeffs = None

        rospy.init_node('hull_cluster_driver', anonymous=True)
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.usbcam_callback)

        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        rospy.loginfo("HullClusterDriver (Graceful Degradation) Started.")

    def usbcam_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.last_successful_coeffs is None:
                h, w = self.image.shape[:2]
                self.last_successful_coeffs = np.array([0, 0, w / 2])
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def get_lane_center(self, frame):
        h, w = frame.shape[:2]
        roi_top_y = int(h * 0.6)

        last_drive_path = np.poly1d(self.last_successful_coeffs)

        mask = np.zeros((h, w), dtype=np.uint8)
        roi_vertices = np.array([[(0, roi_top_y), (w, roi_top_y), (w, h), (0, h)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 60, 40]); upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([158, 36, 28]); upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        final_mask = cv2.bitwise_and(red_mask, red_mask, mask=mask)

        contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        all_centroids = []
        if contours:
            for contour in contours:
                if cv2.contourArea(contour) > 50:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]); cy = int(M["m01"] / M["m00"])
                        all_centroids.append((cx, cy))
                        cv2.drawContours(self.image, [contour], -1, (0,255,0), 2)

        if len(all_centroids) < 2:
             is_path_detected = False
        else:
            all_centroids = np.array(all_centroids)
            dividing_line_x = np.polyval(last_drive_path, all_centroids[:, 1])
            left_centroids = all_centroids[all_centroids[:, 0] < dividing_line_x]
            right_centroids = all_centroids[all_centroids[:, 0] >= dividing_line_x]

            left_lane_coeffs, right_lane_coeffs = None, None

            # ✨ [핵심] 왼쪽 차선 모델링 (점진적 저하)
            if len(left_centroids) >= 3: # 점 3개 이상 -> 2차 곡선
                y_vals = left_centroids[:, 1]; x_vals = left_centroids[:, 0]
                left_lane_coeffs = np.polyfit(y_vals, x_vals, 2)
            elif len(left_centroids) == 2: # 점 2개 -> 1차 직선
                y_vals = left_centroids[:, 1]; x_vals = left_centroids[:, 0]
                line_coeffs = np.polyfit(y_vals, x_vals, 1)
                # 2차항 계수가 0인 2차 다항식으로 변환하여 데이터 형식 통일
                left_lane_coeffs = np.insert(line_coeffs, 0, 0)

            # ✨ [핵심] 오른쪽 차선 모델링 (점진적 저하)
            if len(right_centroids) >= 3: # 점 3개 이상 -> 2차 곡선
                y_vals = right_centroids[:, 1]; x_vals = right_centroids[:, 0]
                right_lane_coeffs = np.polyfit(y_vals, x_vals, 2)
            elif len(right_centroids) == 2: # 점 2개 -> 1차 직선
                y_vals = right_centroids[:, 1]; x_vals = right_centroids[:, 0]
                line_coeffs = np.polyfit(y_vals, x_vals, 1)
                right_lane_coeffs = np.insert(line_coeffs, 0, 0)

            center_path_coeffs = None
            y_range = np.arange(roi_top_y, h)

            if right_lane_coeffs is not None and left_lane_coeffs is not None:
                left_fit = np.poly1d(left_lane_coeffs); right_fit = np.poly1d(right_lane_coeffs)
                center_x_vals = (left_fit(y_range) + right_fit(y_range)) / 2
                center_path_coeffs = np.polyfit(y_range, center_x_vals, 2)
            elif left_lane_coeffs is not None:
                left_fit = np.poly1d(left_lane_coeffs)
                center_x_vals = left_fit(y_range) + self.LANE_WIDTH / 2
                center_path_coeffs = np.polyfit(y_range, center_x_vals, 2)
            elif right_lane_coeffs is not None:
                right_fit = np.poly1d(right_lane_coeffs)
                center_x_vals = right_fit(y_range) - self.LANE_WIDTH / 2
                center_path_coeffs = np.polyfit(y_range, center_x_vals, 2)

            if center_path_coeffs is not None:
                self.last_successful_coeffs = center_path_coeffs
            
        drive_path_func = np.poly1d(self.last_successful_coeffs)
        path_points = []
        for y in range(roi_top_y, h):
            x = int(drive_path_func(y))
            if 0 < x < w: path_points.append((x, y))
        if len(path_points) > 1:
            cv2.polylines(self.image, [np.array(path_points, dtype=np.int32)], False, self.CENTER_LINE_COLOR, 3)

        y_horizon = int(h * 0.65)
        final_target_cx = drive_path_func(y_horizon)
        self.last_cx = final_target_cx
        return final_target_cx

    def vision_drive(self, frame):
        target_cx = self.get_lane_center(frame)
        frame_center = frame.shape[1] // 2
        error = target_cx - frame_center
        k_p = 0.4; k_d = 0.6
        d_error = error - self.last_error
        self.angle = (-k_p * error) + (-k_d * d_error)
        self.last_error = error
        if abs(self.angle) < 10: speed = 35
        elif abs(self.angle) < 30: speed = 20
        else: speed = 15
        self.angle = max(min(self.angle, 100.0), -100.0)
        speed = max(min(speed, 50), 0)
        return self.angle, speed

    def drive(self, angle, speed):
        motor_msg = xycar_motor()
        motor_msg.angle = int(angle)
        motor_msg.speed = int(speed)
        self.motor_pub.publish(motor_msg)

    def run(self):
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                if self.image.size == 0:
                    rate.sleep()
                    continue
                if not self.is_paused:
                    angle, speed = self.vision_drive(self.image)
                    self.drive(angle, speed)
                cv2.circle(self.image, (int(self.last_cx), int(self.image.shape[0] * 0.65)), 15, (0, 255, 0), 3)
                cv2.putText(self.image, f"Angle: {self.angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                if self.is_paused:
                    cv2.putText(self.image, "PAUSED", (self.image.shape[1]//2 - 70, self.image.shape[0]//2),
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                cv2.imshow("HullClusterDriver", self.image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): break
                elif key == 32:
                    self.is_paused = not self.is_paused
                    if self.is_paused: rospy.logwarn("Motor PAUSED!"); self.drive(0, 0)
                    else: rospy.loginfo("Motor Resumed.")
                rate.sleep()
        finally:
            rospy.loginfo("Shutting down. Stopping motor."); self.drive(0, 0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        driver_node = HullClusterDriver()
        driver_node.run()
    except rospy.ROSInterruptException: pass