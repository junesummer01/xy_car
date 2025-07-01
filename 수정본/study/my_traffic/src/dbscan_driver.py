#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =================================================================================
#
# FILE: hull_cluster_driver.py
#
# AUTHOR: Gemini AI
#
# DESCRIPTION:
#   [원거리 조향] 최종 목표점을 이미지의 더 높은 지점(먼 곳)으로 설정하여
#   미리 경로를 예측하고 부드럽게 주행하는 성능을 구현합니다.
#
# CHANGELOG (User Request):
#   - 최종 목표점의 y좌표를 h(이미지 맨 아래)에서 h * 0.75로 상향 조정
# =================================================================================

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from collections import defaultdict

class HullClusterDriver:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        self.last_cx = 320
        self.is_paused = False

        self.CLUSTER_COLORS = [
            (0, 255, 0), (0, 255, 255), (255, 0, 255), (255, 255, 0),
            (0, 128, 255), (255, 128, 0)
        ]
        self.INLIER_COLOR = (0, 255, 0)
        self.OUTLIER_COLOR = (0, 0, 255)
        self.CENTER_LINE_COLOR = (255, 0, 0)

        self.last_successful_slope = 0.0
        self.last_successful_intercept = 0.0
        self.is_path_detected_recently = False

        rospy.init_node('hull_cluster_driver', anonymous=True)
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.usbcam_callback)

        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        rospy.loginfo("RANSAC Hull Cluster Driver (Far-sight) Started.")

    def usbcam_callback(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.last_successful_intercept == 0.0:
                self.last_successful_intercept = self.image.shape[1] / 2
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def get_lane_center(self, frame):
        h, w = frame.shape[:2]
        roi_top_y = int(h * 0.5)
        
        # ... (이전 코드와 동일한 부분은 생략) ...
        mask = np.zeros((h, w), dtype=np.uint8)
        roi_vertices = np.array([[(0, roi_top_y), (w, roi_top_y), (w, h), (0, h)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_vertices, 255)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 70, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([134, 50, 0])
        upper_red2 = np.array([180, 255, 255])
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
        final_mask = cv2.bitwise_and(red_mask, red_mask, mask=mask)

        pixel_coords = np.argwhere(final_mask > 0)
        if len(pixel_coords) > 500:
            indices = np.random.choice(len(pixel_coords), 500, replace=False)
            sampled_coords = pixel_coords[indices]
        else:
            sampled_coords = pixel_coords
            
        if len(sampled_coords) < 10:
            self.is_path_detected_recently = False
        else:
            X = sampled_coords[:, ::-1]
            db = DBSCAN(eps=70, min_samples=10).fit(X)
            labels = db.labels_
            unique_labels = set(labels)
            n_clusters_ = len(unique_labels) - (1 if -1 in labels else 0)

            for label in unique_labels:
                if label != -1:
                    points_in_cluster = X[labels == label]
                    if len(points_in_cluster) >= 3:
                        hull = cv2.convexHull(points_in_cluster)
                        color = self.CLUSTER_COLORS[label % len(self.CLUSTER_COLORS)]
                        cv2.polylines(self.image, [hull], isClosed=True, color=color, thickness=2)

            if n_clusters_ >= 2:
                cluster_centers = [(np.mean(X[labels == l], axis=0)[0], l) for l in unique_labels if l != -1]
                cluster_centers.sort(key=lambda c: c[0])
                left_lane_label = cluster_centers[0][1]
                right_lane_label = cluster_centers[-1][1]
                lane_data = defaultdict(list)
                for label in [left_lane_label, right_lane_label]:
                    points = X[labels == label]
                    y_to_x = defaultdict(list)
                    for x, y in points: y_to_x[y].append(x)
                    for y, x_coords in sorted(y_to_x.items()): lane_data[label].append((y, min(x_coords), max(x_coords)))
                
                center_path_points = []
                left_lane_map = {y: (xmin, xmax) for y, xmin, xmax in lane_data[left_lane_label]}
                right_lane_map = {y: (xmin, xmax) for y, xmin, xmax in lane_data[right_lane_label]}
                for y in range(roi_top_y, h):
                    if y in left_lane_map and y in right_lane_map:
                        _, left_x_max = left_lane_map[y]
                        right_x_min, _ = right_lane_map[y]
                        center_path_points.append((int((left_x_max + right_x_min) / 2), y))
                
                if len(center_path_points) > 10:
                    y_vals = np.array([p[1] for p in center_path_points]).reshape(-1, 1)
                    x_vals = np.array([p[0] for p in center_path_points])
                    ransac = RANSACRegressor()
                    try:
                        ransac.fit(y_vals, x_vals)
                        alpha = 0.6
                        current_slope = ransac.estimator_.coef_[0]
                        current_intercept = ransac.estimator_.intercept_
                        self.last_successful_slope = alpha * current_slope + (1 - alpha) * self.last_successful_slope
                        self.last_successful_intercept = alpha * current_intercept + (1 - alpha) * self.last_successful_intercept
                        self.is_path_detected_recently = True
                    except ValueError:
                        self.is_path_detected_recently = False
                else:
                    self.is_path_detected_recently = False
            else:
                self.is_path_detected_recently = False

        y1 = roi_top_y
        x1 = int(self.last_successful_slope * y1 + self.last_successful_intercept)
        y2 = h - 1
        x2 = int(self.last_successful_slope * y2 + self.last_successful_intercept)
        cv2.line(self.image, (x1, y1), (x2, y2), self.CENTER_LINE_COLOR, 3)

        # ✨ [핵심] 제어를 위한 최종 목표점 계산 지점 변경
        # y=h (화면 맨 아래) 대신, 조금 더 먼 곳(y=h*0.75)을 바라보도록 설정
        y_horizon = int(h * 0.45)
        final_target_cx = self.last_successful_slope * y_horizon + self.last_successful_intercept
        self.last_cx = final_target_cx
        
        return final_target_cx, self.last_successful_slope

    def vision_drive(self, frame):
        target_cx, slope = self.get_lane_center(frame)
        frame_center = frame.shape[1] // 2
        error = target_cx - frame_center
        k_p = 3
        angle = -k_p * error
        if abs(slope) < 0.2: speed = 30
        elif abs(slope) < 0.8: speed = 20
        else: speed = 15
        angle = max(min(angle, 100.0), -100.0)
        speed = max(min(speed, 35), 0)
        return angle, speed

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
                    
                # 시각화용 원은 여전히 y=h*0.75 지점에 그림
                cv2.circle(self.image, (int(self.last_cx), int(self.image.shape[0] * 0.45)), 15, (0, 255, 0), 3)

                if self.is_paused:
                    cv2.putText(self.image, "PAUSED", (self.image.shape[1]//2 - 70, self.image.shape[0]//2), 
                                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                cv2.imshow("RANSAC Cluster Driver (Far-sight)", self.image)
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
