import cv2
import numpy as np

Green = (0,255,0)
Red = (0,0,255)

img = cv2.imread('traffic_sign.jpg', cv2.IMREAD_GRAYSCALE)
blur = cv2.GaussianBlur(img, (3, 3), 0)
cimg = cv2.cvtColor(blur,cv2.COLOR_GRAY2BGR)

circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 
                   1, 
                   10,
                   param1=30,
                   param2=15,
                   minRadius=10, 
                   maxRadius=30)

circles = np.round(circles[0, :]).astype("int")

# Sort the circles based on the x position of the center
circles = sorted(circles, key=lambda circle: circle[0])

for i, (x,y,r) in enumerate(circles):
    cv2.circle(cimg,(x,y),r,Green,2)
    cv2.circle(cimg,(x,y),4,Red,2)
    print(f"Circle {i} at ({x},{y}), radius={r}")

cv2.imshow('img', cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
