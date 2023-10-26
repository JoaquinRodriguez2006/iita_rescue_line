import cv2
from camthreader import *
import numpy as np
import math
import serial
import time

### DEBUG FLAGS ###
debugOriginal = False       # original frame
debugBlack = False          # black mask
debugGreen = False          # green mask
debugBlue = False           # blue mask
debugSliced = False         # sliced black mask to re-acquire line after obstacle
debugLinegapMask = False    # linegap mask (triangle)
debugSilver = False         # sliced frame for evac entrance (should be a silver strip)
debugSlicedGreen = False

#as size increases, %white decreases

### THRESHOLDS ###
lower_black_silver = np.array([0, 0, 0])     # BGR
upper_black_silver = np.array([80, 80, 80])

min_square_size = 40    # for filtering green squares # old: 361
min_cube_size = 100     # for filtering blue cube

### GET FRAME DIMENSIONS ###
vs = WebcamVideoStream(src = 0).start()
ser = serial.Serial('/dev/serial0', 115200, timeout = 0)

test_frame = vs.read()
width, height = test_frame.shape[1], test_frame.shape[0]
print(width, height)

### CONSTANTS ###
x_com = np.zeros(shape = (height, width))
y_com = np.zeros(shape = (height, width))

cam_x = width / 2 - 1   # 79 ~ middle coloumn
cam_y = height - 1      # 119 ~ bottom row

for i in range(height):
    for j in range(width):
        x_com[i][j] = (j - cam_x) / (width / 2)   # [-1, 1]
        y_com[i][j] = (cam_y - i) / height        # [0, 1]

kernel = np.ones((5, 5), np.uint8)  # for denoising line mask
ball_kernel = np.ones((3, 3), np.uint8) # for denoising ball mask

x_scaling_factor = ((1 - y_com) ** 0.1) # for scaling x-values so lower pixels have higher weightage

linegap_mask = np.zeros(shape = (height, width), dtype = np.uint8)
pts = np.array([[-80, 0], [239, 0], [cam_x, cam_y]], np.int32)  # TUNE NEXT TIME
pts = pts.reshape((-1, 1, 2))
cv2.fillPoly(linegap_mask, [pts], 255)  # triangle where stuff at the side are masked out

### VARIABLES ###
counter = 0             # for ignoring double green squares
prev_angle = 0          # for storing forced turn angle

greenSquare = False     # True if green square is detected
forcedTurnCounter = 0   # not used: for 135-turn and pacman forced turns
# evacCounter = 0         # for ignoring silver strip detection after exitting evac zone
deposited = False       # if true, exit evac zone
# seenPurple = False
lastreset = 0
blueCube = False

cx_black = width / 2    # for x-value of black centroid (green squares)
cccounter = 0
task = 0

while True:
    task = 0
    # print('In Evaccccc')

    frame = vs.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    speed = 50  # default 30 for linetrack, varied for cube centering
    angle = 90   # fixed during green square and blue cube centering, varied for linetrack
    task = 0

    #cv2.imshow('Frame', frame)
    black_mask = cv2.inRange(frame, lower_black_silver, upper_black_silver)
    cv2.imshow('Black', black_mask)

    ## SILVER BALL DETECTION ###
    eroded_black_mask = cv2.erode(black_mask, ball_kernel)
    dilated_black_mask = cv2.dilate(eroded_black_mask, ball_kernel)
    subtracted_mask = black_mask - dilated_black_mask
    subtracted_mask[100:, :] = 0
    
    circle_mask = np.zeros(shape = (height,width), dtype = np.uint8)
    circled_mask = np.zeros(shape = (height,width), dtype = np.uint8)

    #print(np.sum(black_mask)/255)
    if (np.sum(subtracted_mask)) and (np.sum(black_mask)/255) > 40:
        M = cv2.moments(subtracted_mask)

        cx_ball = int(M['m10'] / M['m00'])
        cy_ball = int(M["m01"] / M["m00"])
        
        circle_mask = cv2.circle(circle_mask, (cx_ball, cy_ball), min(cy_ball + 5, 60), 255, -1)
        circled_mask = cv2.bitwise_and(subtracted_mask, subtracted_mask, mask = circle_mask)
    
        #cv2.imshow('Circle Mask', circle_mask)
        cv2.imshow('Circled Mask', circled_mask)
        percentage_white = np.sum(circled_mask) / np.sum(circle_mask) if np.sum(circle_mask) else 0
        #print('Percentage White:', percentage_white)
        magicno = (percentage_white * (cy_ball) * 100)
        print(magicno)
        if magicno > 80:  # original: 0.01
            print('Silver Ball detected, centering')
            
            if cx_ball < 80:  # turn left
                angle = 90  # max steer rate, turn on the spot
            else: # turn right
                angle = -90
            
            task = 0

            #print(cx_ball, cy_ball)
            speed = min(abs((cx_ball - 80) / 80) * 20, 20) # speed based on error (x-vectors)
            # print('Speed:', speed)
            if cx_ball > 70 and cx_ball < 90:  # cube is roughly centered
                # print('cy_ball:                                                            ', cy_ball)
                if cy_ball > 25:   # if ball is nearby, turn to center on it
                    print('Picking up Silver Ball')
                    task = 4
                else:
                    speed = 30
                    angle = 0

#     if debugOriginal:
#         cv2.imshow('Original', frame)
#     if debugBlack:
#         cv2.imshow('Black Mask', black_mask)
#     if debugGreen:
#         cv2.imshow('Green Mask', green_mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
vs.stop()