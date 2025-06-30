#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TrafficLightDetector:
    def __init__(self):
        # ROS 노드 초기화! 🤖 'Driver'라는 이름으로 시작할게!
        rospy.init_node('Driver', anonymous=True)
        
        # 이미지 메시지를 OpenCV 이미지로 바꿔주는 다리 역할! 🌉
        self.bridge = CvBridge()
        
        # USB 카메라에서 이미지 받아올 준비! '/usb_cam/image_raw/' 토픽을 구독할 거야! 📸
        # 새 이미지가 오면 'usbcam_callback' 함수를 호출해줘!
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, self.usbcam_callback, queue_size=1)
        
        # 초록색 원이 감지됐는지 알려주는 플래그! 처음엔 '아직 안 보임'으로 시작! 🚦
        self.is_green_circle_detected = False
        
        # 카메라가 준비될 때까지 잠깐 기다려줘! ⏳
        rospy.wait_for_message("/usb_cam/image_raw/", Image)
        print("Camera Ready --------------") # 준비 완료!
        
        # 이제 신호등 감지 시작! ROS 노드가 계속 돌아가게 해주는 마법! ✨
        rospy.spin()

    def usbcam_callback(self, data):
        # 카메라에서 이미지가 오면 이 함수가 호출돼! 🖼️
        try:
            # ROS 이미지 메시지를 OpenCV가 알아볼 수 있는 'bgr8' 형식으로 변환!
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(f"이미지 변환 에러 발생! ㅠㅠ: {e}")
            return

        # 이미지 크기를 줄여서 처리 속도를 높일 수도 있어! (필요하면 주석 해제해봐!) 🤏
        # cv_image = cv2.resize(cv_image, (640, 480)) 

        # 색상 공간을 HSV로 바꿔서 초록색을 더 쉽게 찾을 수 있게 해! 🌈
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 초록색 범위 정의! (이 값은 카메라나 환경에 따라 조절해야 해! 진짜 중요! ✨)
        # H(색상): 초록색은 보통 35~85 정도야.
        # S(채도): 색의 선명도! 너무 낮으면 흐릿한 색도 잡혀.
        # V(명도): 색의 밝기! 너무 낮으면 어두운 색은 안 잡혀.
        lower_green = np.array([35, 100, 100]) # H, S, V 최솟값
        upper_green = np.array([85, 255, 255]) # H, S, V 최댓값

        # 초록색 범위 안에 있는 픽셀들만 골라내는 마스크 만들기! 🎭
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # 원본 이미지에 마스크를 적용해서 초록색 부분만 남기기! 💚
        res = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # 원 검출을 위해 초록색 마스크 이미지를 그레이스케일로 변환하고 블러 처리! 🌫️
        # 블러 처리는 노이즈를 줄여서 원을 더 잘 찾게 도와줘!
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2) 

        # 허프 변환으로 원 찾기! 🎯
        # cv2.HoughCircles(이미지, 검출 방법, 해상도 비율, 원들 사이의 최소 거리, ...)
        # param1: Canny 엣지 검출기의 높은 임계값.
        # param2: 원의 중심을 찾기 위한 누적 배열의 임계값. 이 값이 작을수록 더 많은 원을 찾지만, 가짜 원도 많이 나와!
        # minRadius, maxRadius: 찾을 원의 최소/최대 반지름! (이것도 조절 필요!)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1=50, param2=30, minRadius=10, maxRadius=100)

        self.is_green_circle_detected = False # 일단 이번 프레임에서는 '안 보임'으로 초기화!
        if circles is not None:
            # 찾은 원들의 좌표를 정수로 바꿔줘!
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # 찾은 원이 너무 작거나 크지 않은지 확인하고, 초록색 원으로 판단!
                # 이미 초록색 마스크를 적용했으니, 여기서 원이 검출되면 초록색 원이라고 봐도 돼!
                
                # 원 그리기 (디버깅용! 화면에 초록색 원을 그려줄 거야! 🟢)
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
                
                self.is_green_circle_detected = True # 초록색 원 감지!
                print(f"초록색 원 감지! 출발 신호! 🚀 (중심: {x}, {y}, 반지름: {r})")
                break # 하나만 찾아도 출발 신호니까 바로 멈춰!

        # 감지 상태 출력! 📢
        if not self.is_green_circle_detected:
            print("초록색 원 미감지... 대기 중... 🚦")

        # 화면에 이미지 보여주기 (디버깅용! 카메라 화면이랑 초록색 마스크 결과 보여줄게! 🖥️)
        cv2.imshow("Traffic Light Detector", cv_image)
        cv2.imshow("Green Mask", res) 
        cv2.waitKey(1) # 1ms 대기해서 화면 업데이트!

# 이 파일이 직접 실행될 때만 아래 코드를 실행해줘!
if __name__ == '__main__':
    try:
        TrafficLightDetector() # 클래스 객체 만들면 바로 시작!
    except rospy.ROSInterruptException:
        print("노드 종료! 안녕! 👋") # ROS 노드가 꺼지면 이 메시지 뜰 거야!
    finally:
        cv2.destroyAllWindows() # 열려있던 모든 OpenCV 창 닫아주기!
