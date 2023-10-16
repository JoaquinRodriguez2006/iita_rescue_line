import cv2
from camthreader import *
import numpy as np
import math
import requests
import serial

debugOriginal = True
debugBlack = True
debugHori = True
debugGreen = True  # Dejar esta línea para debug de máscara verde
record = False

noise_blob_threshold = 20
requests.packages.urllib3.disable_warnings()
vs = WebcamVideoStream(src=0).start()
ser = serial.Serial('/dev/serial0', 115200)

lower_black = np.array([0, 0, 0])  # BGR
upper_black = np.array([95, 95, 95])
lower_green = np.array([20, 64, 64])  # HSV
upper_green = np.array([75, 255, 255])

test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)

cam_x = width / 2 - 1   # 79 ~ middle column
cam_y = height - 1      # 119 ~ bottom row

# GET X AND Y ARRAYS
# scaled by frame dimensions, use this for filtering pixels
x_com = np.zeros(shape=(height, width))
y_com = np.zeros(shape=(height, width))
for i in range(height):
    for j in range(width):
        x_com[i][j] = (j - cam_x) / (width / 2)   # [-1, 1]
        y_com[i][j] = (cam_y - i) / height        # [0, 1]

while True:
    frame = vs.read()
    # frame = cv2.flip(frame, -1)     # comment out if testing on robot
    frame[:30, :, :] = 255          # ignore top 1/8 of the frame
    kernel = np.ones((3, 3), np.uint8)

    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # FILTER BLACK PIXELS
    black_mask = cv2.inRange(frame, lower_black, upper_black)
    x_black = cv2.bitwise_and(x_com, x_com, mask=black_mask)
    # scale such that bottom pixels have more weight
    x_black *= (1 - y_com)
    y_black = cv2.bitwise_and(y_com, y_com, mask=black_mask)

    # DETECT HORIZONTAL INTERSECTION LINES
    blurred = cv2.GaussianBlur(black_mask, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 200)
    lines = cv2.HoughLines(edges, rho=1, theta=np.pi / 180, threshold=30)

    minY = 1000  # arbitrarily high value

    bottom_mask = np.zeros(shape=(height, width), dtype=np.uint8)

    if lines is not None:
        lines = [[line[0][0], line[0][1]] for line in lines]
        # theta closer to 90deg will be placed first
        lines = sorted(lines, key=lambda x: abs((x[1]/np.pi*180) - 90))
        # print(lines)
        rho, theta = lines[0]
        degrees = theta / np.pi * 180
        if degrees > 45 and degrees < 135:   # detect a horizontal line
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho

            # 200 = sqrt(height ** 2 + width ** 2)
            x1 = int(x0 + 200 * (-b))
            y1 = int(y0 + 200 * a)
            x2 = int(x0 - 200 * (-b))
            y2 = int(y0 - 200 * a)

            # cv2.line(frame, (x1, y1), (x2, y2),(0, 0, 255), 1)  # draw a line
            # get the lowest y-value of the line in vector form

            if height - max(y1, y2) < minY:
                minY = min(minY, height - y1)
                minY = min(minY, height - y2)

            pts = np.array([[x1, y1], [x2, y2], [160, 120], [0, 120]], np.int32)
            bottom_mask = cv2.fillPoly(bottom_mask, [pts], 255)  # draw a polygon

    if minY == 1000:    # no line detected
        minY = 0

    # FILTER GREEN PIXELS
    hsv_frame = cv2.bitwise_and(hsv_frame, hsv_frame, mask=bottom_mask)
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    green_mask_middle = green_mask.copy()
    # version of green mask with only the middle area shown, used for double green square (maybe)
    green_mask_bottom = green_mask.copy()
    # version of green mask with only the bottom area shown, used for a single green square
    green_mask_bottom[:85, :] = 0

    # DETECT GREEN SQUARES
    # extract outermost contours (ignore inner holes)
    green_contours, hierarchy = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours_bottom, hierarchy = cv2.findContours(
        green_mask_bottom, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # remove noise by limiting blob size
    green_contours = [c for c in green_contours if cv2.contourArea(
        c) > noise_blob_threshold]
    green_contours_bottom = [
        c for c in green_contours_bottom if cv2.contourArea(c) > noise_blob_threshold]
    green_mask = cv2.drawContours(green_mask, green_contours, -1, 255, -1)
    green_mask_bottom = cv2.drawContours(
        green_mask_bottom, green_contours_bottom, -1, 255, -1)
    x_green = cv2.bitwise_and(x_com, x_com, mask=green_mask_bottom)

    # CALCULATE RESULTANT
    green_state = 0

    x_resultant = np.mean(x_black)
    y_resultant = np.mean(y_black)
    angle = (math.atan2(y_resultant, x_resultant) / math.pi * 180) - 90
    speed = 20

    if len(green_contours_bottom) == 1:
        if np.mean(x_green) < 0:  # turn left
            green_state = 1
            angle = 45
        else:
            green_state = 2  # turn right
            angle = -45
    if len(green_contours_bottom) == 2:
        green_state = 3

    # stop the robot from turning if it sees nothing
    if y_resultant == 0:
        angle = 0

    output = [255, speed,
              254, round(angle) + 90,
              253, green_state]
    ser.write(output)

    # DEBUGS
    print("Angle: ", angle, "Green Squares: ", green_state)
    if debugOriginal:
        cv2.imshow('Original', frame)
    if debugBlack:
        cv2.imshow('Black Mask', black_mask)
    if debugHori:
        cv2.imshow('Hori Mask', bottom_mask)
    if debugGreen:
        cv2.imshow('Green Mask', green_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
vs.stop()
