import cv2
import numpy as np

# 트랙바가 변경될 때마다 호출될 더미 함수
def nothing(x):
    pass

# 이미지 불러오기
# 'image.png' 파일이 이 스크립트와 같은 폴더에 있어야 합니다.
image = cv2.imread('image.png')
if image is None:
    print("Error: 'image.png' not found. Make sure the image file is in the same directory.")
    exit()

# HSV로 변환
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 튜닝을 위한 창과 트랙바 생성
cv2.namedWindow('HSV Tuner')
cv2.createTrackbar('L-H', 'HSV Tuner', 0, 179, nothing)
cv2.createTrackbar('L-S', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('L-V', 'HSV Tuner', 0, 255, nothing)
cv2.createTrackbar('U-H', 'HSV Tuner', 179, 179, nothing)
cv2.createTrackbar('U-S', 'HSV Tuner', 255, 255, nothing)
cv2.createTrackbar('U-V', 'HSV Tuner', 255, 255, nothing)

# 초기값 설정 (2단계에서 제안한 값으로 시작)
cv2.setTrackbarPos('L-S', 'HSV Tuner', 70)
cv2.setTrackbarPos('L-V', 'HSV Tuner', 40)


print("Adjust the sliders to find the best HSV range.")
print("Press 's' to save the current values and print them.")
print("Press 'q' to quit.")


while True:
    # 트랙바에서 현재 값들을 가져옴
    l_h = cv2.getTrackbarPos('L-H', 'HSV Tuner')
    l_s = cv2.getTrackbarPos('L-S', 'HSV Tuner')
    l_v = cv2.getTrackbarPos('L-V', 'HSV Tuner')
    u_h = cv2.getTrackbarPos('U-H', 'HSV Tuner')
    u_s = cv2.getTrackbarPos('U-S', 'HSV Tuner')
    u_v = cv2.getTrackbarPos('U-V', 'HSV Tuner')

    # lower/upper 경계값 생성
    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    # 마스크 생성 및 표시
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    result = cv2.bitwise_and(image, image, mask=mask)

    cv2.imshow('Original Image', image)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord('s'):
        print("\n--- Current HSV Values ---")
        print(f"lower_bound = np.array([{l_h}, {l_s}, {l_v}])")
        print(f"upper_bound = np.array([{u_h}, {u_s}, {u_v}])")
        print("--------------------------\n")

cv2.destroyAllWindows()