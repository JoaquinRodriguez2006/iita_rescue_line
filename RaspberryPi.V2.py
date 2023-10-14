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
 
### THRESHOLDS ###
lower_black = np.array([0, 0, 0])     # BGR
# upper_black = np.array([95, 95, 95])
upper_black = np.array([100,105,105])
 
lower_black_evac = np.array([0, 0, 0])     # BGR
upper_black_evac = np.array([60, 60, 60])
 
lower_black_silver = np.array([0, 0, 0])     # BGR
upper_black_silver = np.array([80, 80, 80])
 
lower_green = np.array([70, 80, 80])  # HSV
upper_green = np.array([80, 255, 255])
 
lower_green_evac = np.array([70, 125, 80])  # HSV
upper_green_evac = np.array([80, 255, 255])
 
lower_blue = np.array([110, 80, 50]) # HSV
upper_blue = np.array([125, 255, 255])
 
lower_red = np.array([0, 100, 100]) # HSV
upper_red = np.array([20, 255, 255])
# lower_red = np.array([170, 70, 100])
# upper_red = np.array([180, 255, 255])
 
lower_purple = np.array([115, 40, 140])
upper_purple = np.array([137, 255, 255])
 
lower_orange = np.array([5, 60, 70])
upper_orange = np.array([25, 255, 255])
 
lower_marker = np.array([115, 40, 140]) # PURPLE THIS TIME
upper_marker = np.array([137, 255, 255])
 
min_square_size = 20    # for filtering green squares # old: 361
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
 
blue_state = False
 
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

while True:
    ### FIRST LINE TRACK ###
    while True:
        data = ser.read()
        # CONTROLS THE SWITCH - WHEN IT STOPS RECEIVING DATA, UNDERSTANDS THAT THE SWITCH IS OFF. OTHERWISE, IT'S ON.
        # switch is on
        if cccounter > 0:
            cccounter -= 1
            data = b'\xff'  # WHENERVER xff IS TRUE, THE SWITCH IS OFF
        # switch is off
        if data == b'\xff': # switch is off
            if ser.in_waiting > 500:
                ser.reset_input_buffer()
                cccounter = 50
            counter = 0
            prev_angle = 0
            greenSquare = False
            deposited = False
            blueCube = False
 
        frame = vs.read()
        frame[:25, :, :] = 255  # block out horizon
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
        ### DATA SENT TO TEENSY ###
        speed = 30  # default 30 for linetrack, varied for cube centering
        angle = 0   # fixed during green square and blue cube centering, varied for linetrack
        task = 0    # 0 = no green (default), 1 = left green, 2 = right green, 3 = double green (reverse), 4 = pick up cube, 5 = wall track, 6 = move straight to enter evac zone, 7 = exit evac zone  
        # line_middle is defined below: for reacquiring line after obstacle
 
        ### PURPLE MARKER ###
        # marker_mask = cv2.inRange(hsv_frame, lower_marker, upper_marker)
        # print('Mean:', np.mean(marker_mask) / 255.0)
        # if (np.mean(marker_mask) / 255.0 > 0.5):
        #     seenPurple = True
 
        # else:
        #     seenPurple = False
 
        ### DETECT BLACK ###
        black_mask = cv2.inRange(frame, lower_black, upper_black)
        black_mask = cv2.erode(black_mask, kernel)  # remove noisy pixels
        black_mask = cv2.dilate(black_mask, kernel)

        ### DETECT RED LINE ###
        red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
        #if np.mean(red_mask) > 50:
            # print('Red')
        black_mask = cv2.bitwise_or(black_mask, red_mask)
 
        black_mask_linegap = cv2.bitwise_and(black_mask, black_mask, mask = linegap_mask)   # mask outside pixels
        y_black_uncropped = cv2.bitwise_and(y_com, y_com, mask = black_mask_linegap)
        x_black_uncropped = cv2.bitwise_and(x_com, x_com, mask = black_mask_linegap) * x_scaling_factor
 
        ### EVAC ENTRANCE DETECTION ###
        summed_pixels = list(np.amax(black_mask, axis = 1)) # if row has black line: 1, else 0
 
        ### IGNORE LINES ABOVE/ON OTHER TILES ###
        summed_pixels.reverse()
        lineIndex = summed_pixels.index(max(summed_pixels)) # get index of lowest line
        gapIndex = summed_pixels.index(0, lineIndex)        # get index of lowest white gap above line
       
        # print('Line Index:', height - lineIndex)
        # print('Gap Index:', height - gapIndex)
        black_mask[:height - gapIndex, :] = 0   # mask everything above lowest line starting from white gap
 
        ### EVAC ENTRANCE ###
        masked_summed_pixels = list(np.amax(black_mask, axis = 1)) # if row has black line: 1, else 0
        furthestLineIndex = masked_summed_pixels.index(max(masked_summed_pixels))   # find index of furthest black line
        print('Furthest Line Index:', furthestLineIndex)
        if furthestLineIndex > 75:  # last 1/5 of frame, line is ending
            slicedFrame = frame[int(0.8 * furthestLineIndex):furthestLineIndex + 1, :]
            std = np.std(slicedFrame) / np.mean(slicedFrame)
            print('Normalized Standard Deviation:', std)
 
            # cv2.imshow('Silver Sliced Frame', slicedFrame)
 
            # if std > 0.28 and np.mean(cv2.inRange(slicedFrame, lower_red, upper_red)) < 10 and np.mean(cv2.inRange(slicedFrame, lower_green, upper_green)) < 10:   # do roomba
            if std > 0.3 and std < 0.9 and data != b'\xff' and (not blue_state) and (not greenSquare):   # can loosen, and check if angle is close to 0
            #if std > 0.4:
                print('Silver Strip')
                output = [255, round(speed),
                254, round(angle) + 90,
                253, 6,
                252, 0]
                ser.write(output)
                break
 
        ### 135-TURNS & PACMAN (DISABLED FOR NOW) ###
        y_black = cv2.bitwise_and(y_com, y_com, mask = black_mask)
        # print('Longestttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt:', np.max(y_black))
        x_black = cv2.bitwise_and(x_com, x_com, mask = black_mask) # weigh bottom pixels more
 
        x_max = np.max(x_black) # for detecting line gap later
        x_min = np.min(x_black) # have to be gotten first before top black stuff are masked out
 
        x_black = x_black * x_scaling_factor
 
        # sobelled = (cv2.Sobel(black_mask, cv2.CV_8U, 1, 0, ksize = 1))  # obtain gradients, if gap -> line: 1, line -> line: 0
        # print(np.sum(sobelled, axis = 1) / 510)
 
        # ret, edge_count = cv2.threshold(np.sum(sobelled, axis = 1) / 510, 1.5, 255, cv2.THRESH_BINARY)  # for every row, check that there are 2 areas of black -> (120,) of 0s and 1s
        # edge_mask = np.tile(edge_count, (1, width)).astype(np.uint8)    # turn (120,) array to (120,160) array (same size as frame)
 
        # black_contours, hierarchy = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # contoured_mask = np.zeros(shape = (height, width), dtype = np.uint8)
        # stuff_to_colour = np.zeros(shape = (height, width), dtype = np.uint8)
 
        # print(black_contours)
 
        # if len(black_contours): # if there are black contours
        #     max_black_contour = cv2.convexHull(max(black_contours, key = cv2.contourArea))   # get convex hull of largest contour
        #     cv2.drawContours(contoured_mask, [max_black_contour], -1, 255, -1)  # draw contour on mask
 
        #     stuff_to_colour = contoured_mask - black_mask   # extracting tiny square to colour green
        #     stuff_to_colour = cv2.bitwise_and(stuff_to_colour, stuff_to_colour, mask = edge_mask)   # only colour areas within bend
        #     stuff_to_colour = cv2.erode(stuff_to_colour, kernel)    # denoise
        #     stuff_to_colour = cv2.dilate(stuff_to_colour, kernel, iterations = 1)
 
        # cv2.imshow('Sobel', sobelled)
        # cv2.imshow('Edge Mask', edge_mask)
        # cv2.imshow('Stuff to Colour', stuff_to_colour)
       
        # cv2.imshow('y_black', y_black)
        # print('Evacs', evacs)
 
        ### NEW GREEN SQUARES ###
        green_mask = cv2.inRange(hsv_frame[90:, :, :], lower_green, upper_green)    # shape = (30, 160)
        # green_mask = cv2.bitwise_or(old_green_mask, stuff_to_colour[90:, :])    # colour for 135 turns/pacman
        # present_135 = False # True if 135 turn is present
 
        # if np.sum(stuff_to_colour[90:, :]) > min_square_size * 255:
        #     present_135 = True
       
        if np.sum(green_mask) > min_square_size * 255:
            green_pixels = np.amax(green_mask, axis = 0)    # for every column, if green: 1 else 0, shape = (160,)
           
            greenIndices = np.where(green_pixels == np.max(green_pixels))   # get indices of columns that have green
            leftIndex = greenIndices[0][0]
            rightIndex = greenIndices[0][-1]
            # print('Left Right Indices:', leftIndex, rightIndex)
            # print('Bound size:', rightIndex - leftIndex)
 
            slicedGreen = frame[60:90, leftIndex:rightIndex + 1, :]
 
            greenCentroidX = (rightIndex + leftIndex) / 2
            slicedBlackMaskAboveGreen = black_mask[60:90, leftIndex:rightIndex + 1]
            blackM = cv2.moments(black_mask[90:, :])
 
            if np.sum(black_mask[90:, :]):  # if there is black, prevents divide by 0 error
                cx_black = int(blackM["m10"] / blackM["m00"])   # get x-value of black centroid
 
            # print('cx_black:                           ', cx_black)
            # print('green centroid:                        ', greenCentroidX)
            # print('Black Sum:', np.sum(slicedBlackMaskAboveGreen) / (255 * 30 * (rightIndex - leftIndex)))
 
            if (np.sum(slicedBlackMaskAboveGreen) / (255 * 30 * (rightIndex - leftIndex))) > 0.32:   # if mean is high -> square before line
                greenSquare = True
                # print('Green Square Detectedddd')
                filtered_green_mask = cv2.erode(green_mask, kernel)
                filtered_green_mask = cv2.dilate(green_mask, kernel)
                green_contours, hierarchy = cv2.findContours(filtered_green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if len(green_contours) > 1 and cx_black > leftIndex and cx_black < rightIndex and counter == 0 and np.sum(green_mask) > (2 * min_square_size * 255):
                    # print('Double Green')
                    task = 3
                    counter = 0
 
                elif greenCentroidX < cx_black:
                    # print('                                  Left Green')
                    counter = 200
                    # if present_135 and forcedTurnCounter == 0:  # 2-wheel turn for 135
                    #     angle = 90
                    #     forcedTurnCounter = 100
                    #     prev_angle = 90
                    angle = 46
 
                else:
                    # print('                                  Right Green')
                    counter = 200
                    # if present_135 and forcedTurnCounter == 0:  # 2-wheel turn for 135
                    #     angle = -90
                    #     forcedTurnCounter = 100
                    #     prev_angle = -90
                    angle = -46
 
            else:
                greenSquare = False
 
        else:
            greenSquare = False
 
        # print('Counter                               :', counter)
        if counter > 0:
            counter -= 1
 
        # if evacCounter > 0:
        #     evacCounter -= 1
 
        ### DETECT LINE AFTER OBSTACLE ###
        sliced_black_mask = black_mask[75:95, :]
        line_middle = round(np.mean(sliced_black_mask))
 
        ### SCALE ANGLE ###
        # print('Before:', angle)
        if counter == 0:    # don't power for green squares
            if not ((np.max(y_black) < 0.5) and (x_max - x_min) < 1.2): # only power if there's no line gap
                # print('Poweringgg') # the shorter the line, the greater the power (fractional root of decimal gives larger decimal)
                angle = ((abs(angle) / 90.0) ** (np.max(y_black) * (height/(height - 25)))) * 90.0 * (-1 if angle < 0 else 1)
                # print('Power:', np.max(y_black) * ((height) / (height - 25)))
        # print('After:', angle)
 
        # if forcedTurnCounter > 0:   # if doing forced turn (135, not needed for now), reset angle
        #     forcedTurnCounter -= 1
        #     angle = prev_angle
 
        ### SEND DATA TO TEENSY ###
        output = [255, round(speed),
                254, round(angle) + 90,
                253, task,
                252, line_middle]
        ser.write(output)
 
        ### DEBUGS ###
        # print("Speed:", speed, "Angle:", angle, "Task:", task, "Line Middle:", line_middle)
       
        if debugOriginal:
            cv2.imshow('Original', frame)
        if debugBlack:
            cv2.imshow('Black Mask', black_mask)
        if debugGreen:
            cv2.imshow('Green Mask', green_mask)
        if debugSliced:
            cv2.imshow('Sliced Black Mask', sliced_black_mask)
        if debugLinegapMask:
            cv2.imshow('Linegap Mask', linegap_mask)
        if debugSlicedGreen:
            cv2.imshow('Sliced Green', slicedGreen)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    while True:
        # print('In Evaccccc')
 
        frame = vs.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
        speed = 50  # default 30 for linetrack, varied for cube centering
        angle = 90   # fixed during green square and blue cube centering, varied for linetrack
        task = 5
 
        # marker_mask = cv2.inRange(hsv_frame, lower_marker, upper_marker)
        # print('Mean:', np.mean(marker_mask) / 255.0)
        # if (np.mean(marker_mask) / 255.0 > 0.5):
        #     print('Marker')
        #     evacCounter = 85
        #     seenPurple = True
        #     break
 
        # else:
        #     seenPurple = False
 
        # cv2.imshow('Frame', frame)
        data = ser.read()
 
        if data == b'\xff': # switch is off
            ### RESET ALL FLAGS ###
            counter = 0
            prev_angle = 0
            greenSquare = False
            deposited = False
            blueCube = False
            break
 
        evac_mask = cv2.inRange(frame, lower_black_evac, upper_black_evac)
        evac_mask = cv2.erode(evac_mask, kernel)
        evac_mask = cv2.dilate(evac_mask, kernel)
 
        evac_contours, heirarchy = cv2.findContours(evac_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        max_ratio = 0
 
        if len(evac_contours):
            max_index = np.argmax([cv2.contourArea(cnt) for cnt in evac_contours])
            max_evac_contour = evac_contours[max_index]
            max_area = cv2.contourArea(max_evac_contour)
            print('Area:', max_area)
 
            x,y,w,h = cv2.boundingRect(max_evac_contour)
            max_ratio = float(w)/h
 
        x_evac = 0
       
        if np.sum(evac_mask):
            x_evac = np.average(x_com, weights = evac_mask)
            # print('sum:', np.sum(evac_mask) / 255.0)
 
        # print('x_evac:', x_evac)
 
        if max_area < 1200 or max_area > 4000 or max_ratio < 6:
            angle = 90
            speed = 50
 
        else:
            if abs(x_evac) < 0.015 and np.sum(evac_mask) / 255.0 > 50 and (not deposited):
                task = 10
                deposited = True
           
            elif x_evac < 0:
                angle = 90
                speed = abs(x_evac) * 50
 
            else:
                angle = -90
                speed = abs(x_evac) * 50
       
        print('Red Sum:', np.sum(cv2.inRange(hsv_frame, lower_red, upper_red)) / 255.0)
        if np.sum(cv2.inRange(hsv_frame, lower_red, upper_red)) / 255.0 > 50 and deposited:
             # print('Red')
             speed = 0
             angle = 0
             task = 11
 
    #     ### DATA SENT TO TEENSY ###
    #     speed = 0
    #     angle = 0
    #     task = 5  # 0 = no green (default), 1 = left green, 2 = right green, 3 = double green (reverse), 4 = pick up cube, 5 = wall track, 6 = move straight to enter evac zone, 7 = exit evac zone  
    #     greenDistance = 0
 
    #     ### DETECT Gevac_mask = cv2.inRange(frame, lower_black, upper_black)REEN EXIT ###
    #     green_mask = cv2.inRange(hsv_frame, lower_green_evac, upper_green_evac)
 
    #     percentage_green = np.mean(green_mask[105:, :]) / 255.0
    #     print('Percentage Green                                       ', percentage_green)
    #     print('sum green maskkkkkkkkkkk:', np.sum(green_mask) / 255.0)
 
    #     if percentage_green > 0.5 and deposited:    # if exit is detected, exit evac zone and continue line track
    #         print('Exitting Evac')
    #         evacCounter = 100
    #         break
 
    #     elif (np.sum(green_mask) / 255.0) > 200:    # handle gap
    #         task = 7
 
    #     ### DETECT BLACK ###
    #     black_mask = cv2.inRange(frame, lower_black_evac, upper_black_evac)
    #     filtered_black_mask = black_mask.copy()
       
    #     x_black = cv2.bitwise_and(x_com, x_com, mask = black_mask) * ((1 - y_com)) # weigh bottom pixels more
    #     y_black = cv2.bitwise_and(y_com, y_com, mask = black_mask)
 
    #     blurred = cv2.GaussianBlur(black_mask, (5, 5), 0)
    #     edges = cv2.Canny(blurred, 50, 200)
    #     # cv2.imshow('Edges', edges)
 
    #     lines = cv2.HoughLines(edges, rho = 1, theta = np.pi / 180, threshold = 30)
    #     lines = [[line[0][0], line[0][1]] for line in lines] if lines is not None else []
 
    #     # print('Length Lines:', len(lines))
       
    #     if len(lines): # at least 1 line detected
    #         for line in lines:
    #             rho, theta = line
 
    #             a = np.cos(theta)
    #             b = np.sin(theta)
    #             x0 = a * rho
    #             y0 = b * rho
 
    #             x1 = int(x0 + 200 * (-b))   # 200 is arbitrary (frame's diagonal)
    #             y1 = int(y0 + 200 * a)
    #             x2 = int(x0 - 200 * (-b))
    #             y2 = int(y0 - 200 * a)
 
    #             cv2.line(filtered_black_mask, (x1, y1), (x2, y2), (0, 0, 0), 2)
 
    #     # cv2.imshow('Filtered Black Mask', filtered_black_mask)
 
        ### BLACK BALL DETECTION ###
 
        if np.sum(evac_mask):
            black_contours, hierarchy = cv2.findContours(evac_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # extract outermost contours
            black_contours = [c for c in black_contours if cv2.contourArea(c) > 100] # remove noise
            black_contours = sorted(black_contours, key = lambda x: cv2.contourArea(x), reverse = True)
 
            cx_ball = None
            cy_ball = None
 
            if len(black_contours) > 0:
                for black_contour in black_contours:
                    black_circle_mask = np.zeros(shape = (height,width), dtype = np.uint8)
                    black_circled_mask = np.zeros(shape = (height,width), dtype = np.uint8)
 
                    black_square_mask = np.zeros(shape = (height,width), dtype = np.uint8)
                    black_squared_mask = np.zeros(shape = (height,width), dtype = np.uint8)
 
                    M = cv2.moments(black_contour)
                    cx_contour = int(M['m10'] / M['m00'])
                    cy_contour = int(M["m01"] / M["m00"])
 
                    x,y,w,h = cv2.boundingRect(black_contour)
                    ratio = float(w)/h
 
                    #print('cy:', cy_contour, 'cx', cx_contour)
 
                    radius = min(w,h)
                    black_circle_mask = cv2.circle(black_circle_mask, (cx_contour, cy_contour), radius, 255, -1)
                    black_square_mask = cv2.rectangle(black_square_mask, ((cx_contour - radius), (cy_contour - radius)), ((cx_contour + radius), (cy_contour + radius)), 255, -1)
                   
                    black_circled_mask = cv2.bitwise_and(evac_mask, evac_mask, mask = black_circle_mask)
                    black_square_mask = black_square_mask - black_circle_mask
                    black_squared_mask = cv2.bitwise_and(evac_mask, evac_mask, mask = black_square_mask)
 
                    #cv2.imshow('Circle Mask', black_circle_mask)
                    #cv2.imshow('Square Mask', black_square_mask)
                    #cv2.imshow('Black Circled Mask', black_circled_mask)
                    #cv2.imshow('Squared Mask', black_squared_mask)
 
                    percentage_background_black = np.sum(black_squared_mask) / np.sum(black_square_mask) if np.sum(black_square_mask) else 0
                    percentage_ball = np.sum(black_circled_mask) / np.sum(black_circle_mask) if np.sum(black_circle_mask) else 0
                   
                    print('Percentage Background Black:', percentage_background_black,'Percentage Ball:', percentage_ball)
                    # percentage ball threshold: 10%
                    print('ratio', ratio)
                    if percentage_background_black < 0.03 and percentage_ball > 0.3 and ratio > 0.95 and ratio < 1.2:
                        print('Black Ball Detected, centering')
                        cx_ball = cx_contour
                        cy_ball = cy_contour
                        break
 
            if cx_ball is not None:  
                task = 40      
                if cx_ball < 80:  # turn left
                    angle = 90  # max steer rate, turn on the spot
                else: # turn right
                    angle = -90
               
                speed = min(abs((cx_ball - 80) / 80) * 20, 20) # speed based on error (x-vectors)
                # print('Speed:', speed)
                print('cy_ba;;', cy_ball, cx_ball)
                if cx_ball > 70 and cx_ball < 90:  # cube is roughly centered
                    if cy_ball > 21:   # if ball is nearby, turn to center on it
                        print('Picking up Black Ball')
                        task = 50
 
                    else:
                        speed = 30
                        angle = 0
       
        # print("Speed:", speed, "Angle:", angle, "Task:", task)
 
        black_mask = cv2.inRange(frame, lower_black_silver, upper_black_silver)
        # cv2.imshow('Black', black_mask)
 
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
            #cv2.imshow('Circled Mask', circled_mask)
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
               
                task = 40
 
                #print(cx_ball, cy_ball)
                speed = min(abs((cx_ball - 80) / 80) * 20, 20) # speed based on error (x-vectors)
                # print('Speed:', speed)
                if cx_ball > 70 and cx_ball < 90:  # cube is roughly centered
                    print('cy_ball:                                                            ', cy_ball)
                    if cy_ball > 25:   # if ball is nearby, turn to center on it
                        print('Picking up Silver Ball')
                        task = 60
                    else:
                        speed = 30
                        angle = 0
 
        output = [255, round(speed),
                254, round(angle) + 90,
                253, task,
                252, 0]
        ser.write(output)
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    #     if debugOriginal:
    #         cv2.imshow('Original', frame)
    #     if debugBlack:
    #         cv2.imshow('Black Mask', black_mask)
    #     if debugGreen:
    #         cv2.imshow('Green Mask', green_mask)
 
cv2.destroyAllWindows()
vs.stop()