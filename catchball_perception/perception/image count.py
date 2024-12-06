import cv2
import os
import numpy as np

def process_image_and_save():
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    # cv2.imshow('dd',frame)
    img = frame

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # convert to hsv colorspace
    hsv1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv2 = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # lower bound and upper bound for Yellow color
    lower_bound = np.array([20, 80, 80])
    upper_bound = np.array([30, 255, 255])

    # find the colors within the boundaries
    mask = cv2.inRange(hsv2, lower_bound, upper_bound)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)

    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    gray = cv2.medianBlur(mask, 9)

    rows = gray.shape[0]
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 3,
                            param1=100, param2=10,
                            minRadius=8, maxRadius=70)
    o = 0
    if circles is not None:
        circles = np.uint16(np.around(circles))
        circles2 = sorted(circles[0], key=lambda x: x[2], reverse=True)
        i = circles2[0]
        if (i[0] >= gray.shape[1]) or (i[1] >= gray.shape[0]):
            return mask
        # cv2.imshow('d', hsv1)
        
        while True:
            
            image_path = os.path.join('/home/airlab/ws_gfair/src/gfair/ball', f'image_{o}.png')
            cv2.imwrite(image_path, hsv1)
            o += 1
            # cv2.circle(hsv1, center, 1, (255, 0, 0), 3) #if you want to see the circle.

    return mask

# 함수 호출
result = process_image_and_save()