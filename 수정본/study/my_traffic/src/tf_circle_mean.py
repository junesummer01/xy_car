import cv2
import numpy as np

Green = (0,255,0)
Red = (0,0,255)

# Load the image
img = cv2.imread("traffic_sign.jpg",cv2.IMREAD_GRAYSCALE)

# Apply Gaussian blur to reduce noise
blur = cv2.GaussianBlur(img,(5,5),0)

# Make color image for check
cimg = cv2.cvtColor(blur,cv2.COLOR_GRAY2BGR)

# Apply Hough Circle Transform to detect circles
circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 
          1, 20, param1=50, param2=30, minRadius=5, maxRadius=30)

if circles is not None:
    # Sort the circles based on the x position of the center
    circles = np.round(circles[0,:]).astype("int")
    circles = sorted(circles, key=lambda circle: circle[0])

    # Extract small rectangle areas around the center of each circle,
    # calculate the mean value of each rectangle area
    for i, (x, y, r) in enumerate(circles):
        x1 = x-(r//2)
        y1 = y-(r//2)
        x2 = x+(r//2)
        y2 = y+(r//2)
        roi = img[y1:y2,x1:x2]
        mean_value = np.mean(roi)
        print(f"Circle {i} at ({x},{y}), radius={r}: mean value={mean_value}")

        # Draw the detected circles and rectangle areas on the original image
        cv2.circle(cimg,(x,y),r,Green,2)
        cv2.rectangle(cimg,(x1,y1),(x2,y2),Red,2)

# Display the result
cv2.imshow('Circles Detected', cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
