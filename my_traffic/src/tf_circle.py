import cv2
import numpy as np

Green = (0,255,0)
Red = (0,0,255)

img = cv2.imread('baduk1.jpg', cv2.IMREAD_GRAYSCALE)
blur = cv2.GaussianBlur(img, (3, 3), 0)
cimg = cv2.cvtColor(blur,cv2.COLOR_GRAY2BGR)

circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 
                   1, 
                   20,
                   param1=50,
                   param2=25,
                   minRadius=0, 
                   maxRadius=50)

circles = np.uint16(np.around(circles))

for i in circles[0,:]:
    cv2.circle(cimg,(i[0],i[1]),i[2],Green,2)
    cv2.circle(cimg,(i[0],i[1]),2,Red,3)

cv2.imshow('img', cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
