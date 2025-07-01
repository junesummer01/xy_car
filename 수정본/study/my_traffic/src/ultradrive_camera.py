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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#=============================================
# 카메라 데이터를 ROS에서 받아와 양쪽의 빨간색 라바콘(종이컵)을
# 모두 인식하고, 그 중심 경로를 따라 주행하도록 제어
#=============================================

import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor

class UltraDriveCamera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = np.empty(shape=[0])
        # 초기 last_cx는 화면의 중앙으로 설정
        self.last_cx = 320  

        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        rospy.Subscriber("/usb_cam/image_raw/", Image, self.usbcam_callback)
        rospy.init_node('red_cone_vision_driver', anonymous=True)

        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        rospy.loginfo("Camera Ready --------------")

    def usbcam_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def get_red_cone_center(self, frame):
        # BGR 이미지를 HSV 컬러 스페이스로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 빨간색을 위한 HSV 범위 정의 (두 부분으로 나뉨)
        # 이 값들은 환경에 따라 튜닝이 필요할 수 있습니다.
        lower_red1 = np.array([0, 100, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 50])
        upper_red2 = np.array([180, 255, 255])

        # 각 범위에 대한 마스크 생성
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        # 두 마스크를 하나로 합침
        mask = cv2.bitwise_or(mask1, mask2)

        # 마스크에서 contour(윤곽선) 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 유효한 라바콘들의 중심 x좌표를 저장할 리스트
        valid_cones_cx = []

        for c in contours:
            area = cv2.contourArea(c)
            # 너무 작거나 큰 노이즈를 걸러내기 위한 면적 필터링
            # (300, 50000 값은 실제 환경에 맞춰 조정 필요)
            if 300 < area < 50000:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                valid_cones_cx.append(cx)
                # (디버깅용) 인식된 라바콘에 초록색 사각형 그리기
                # cv2.rectangle(self.image, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # 유효한 라바콘의 개수에 따라 주행 로직 결정
        if len(valid_cones_cx) >= 2:
            # 2개 이상 인식되면 가장 왼쪽과 오른쪽 라바콘의 중간을 목표로 설정
            leftmost_cx = min(valid_cones_cx)
            rightmost_cx = max(valid_cones_cx)
            target_cx = (leftmost_cx + rightmost_cx) // 2
            rospy.loginfo(f"Two+ cones detected. Left: {leftmost_cx}, Right: {rightmost_cx}, Target: {target_cx}")
            self.last_cx = target_cx
            return target_cx

        elif len(valid_cones_cx) == 1:
            # 1개만 인식되면, 해당 라바콘을 피해서 주행
            single_cx = valid_cones_cx[0]
            frame_center = frame.shape[1] // 2
            # 회피를 위한 오프셋 (차량 폭과 안전거리를 고려하여 튜닝)
            offset = 150 
            
            if single_cx < frame_center:
                # 라바콘이 왼쪽에 있으므로 오른쪽으로 회피
                target_cx = single_cx + offset
                rospy.loginfo(f"Single cone on left ({single_cx}). Evading right, Target: {target_cx}")
            else:
                # 라바콘이 오른쪽에 있으므로 왼쪽으로 회피
                target_cx = single_cx - offset
                rospy.loginfo(f"Single cone on right ({single_cx}). Evading left, Target: {target_cx}")
            
            self.last_cx = target_cx
            return target_cx
        
        # 유효한 라바콘이 하나도 없을 경우
        rospy.logwarn("No valid cones detected. Using last known center.")
        # 마지막으로 유효했던 중심점을 그대로 반환하여 현재 주행 방향을 유지
        return self.last_cx

    def vision_drive(self, frame):
        cx = self.get_red_cone_center(frame)
        frame_center = frame.shape[1] // 2
        error = cx - frame_center

        rospy.loginfo(f"[Vision] Target Center: {cx} (Error: {error})")

        # P제어: 에러 값에 비례하여 조향각 결정
        k = 0.4 # P 제어 게인 (클수록 더 민감하게 반응, 튜닝 필요)
        angle = -k * error

        # 에러 값에 따라 속도 조절
        speed = 10 if abs(error) < 50 else 5
        
        # 조향각과 속도의 최대/최소값 제한
        angle = max(min(angle, 50.0), -50.0)
        speed = max(min(speed, 20), 0)

        return angle, speed

    def drive(self, angle, speed):
        motor_msg = xycar_motor()
        motor_msg.angle = angle
        motor_msg.speed = speed
        self.motor_pub.publish(motor_msg)

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.image.size != 0:
                angle, speed = self.vision_drive(self.image)
                self.drive(angle, speed)

                # 시각화 (디버깅에 유용)
                vis_img = self.image.copy()
                # 화면 중앙선
                cv2.line(vis_img, (vis_img.shape[1] // 2, 0), (vis_img.shape[1] // 2, vis_img.shape[0]), (255, 0, 0), 2)
                # 현재 목표 지점
                cv2.circle(vis_img, (int(self.last_cx), vis_img.shape[0] // 2), 10, (0, 255, 0), -1)
                cv2.imshow("Red Cone Detection", vis_img)
                cv2.waitKey(1)
            
            rate.sleep()

#=============================================
# 프로그램 진입점
#=============================================
if __name__ == "__main__":
    try:
        node = UltraDriveCamera()
        node.run()
    except rospy.ROSInterruptException:
        pass

    # 수정전(가장 큰 라바콘 1개만 인식)
    # def get_red_cone_center(self, frame):
    #     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #     lower_red1 = np.array([0, 100, 100])
    #     upper_red1 = np.array([10, 255, 255])
    #     lower_red2 = np.array([160, 100, 100])
    #     upper_red2 = np.array([180, 255, 255])

    #     mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    #     mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    #     mask = cv2.bitwise_or(mask1, mask2)

    #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #     if contours:
    #         largest = max(contours, key=cv2.contourArea)
    #         area = cv2.contourArea(largest)
    #         if area > 300:
    #             x, y, w, h = cv2.boundingRect(largest)
    #             cx = x + w // 2
    #             self.last_cx = cx
    #             return cx

    #     return self.last_cx

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
