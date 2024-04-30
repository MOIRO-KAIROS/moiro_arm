import numpy as np
import cv2
color = "blue"

while True:
    ret, frame = VideoCapture()
    # 색상 범위 지정
    if color == 'blue':
        low = np.array([90, 50, 50])
        up = np.array([110, 255, 255])
    elif color == 'red':
        low = np.array([-10, 50, 50])
        up = np.array([20, 255, 255])
    elif color == 'yellow':
        low = np.array([20, 50, 50])
        up = np.array([70, 255, 255])
    else:
        low = np.array([100, 50, 50])
        up = np.array([140, 255, 255])

    # 프레임을 HSV 형식으로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 색상에 따라 마스크 생성
    mask = cv2.inRange(hsv, low, up)
    
    # 마스크를 사용하여 큐브 영역 추출
    cube_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if cube_contours:
        idx = 0
        for cube_contour in cube_contours:
            area = cv2.contourArea(cube_contour)
            epsilon = 0.1 * cv2.arcLength(cube_contour, True)
            cube_corners = cv2.approxPolyDP(cube_contour, epsilon, True)
            if area >= 1000 and len(cube_corners) == 4:
                idx,frame = detect_cube(frame,cube_corners,idx,color)