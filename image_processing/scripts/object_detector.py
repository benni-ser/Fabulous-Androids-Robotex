import cv2
import numpy as np
import time
import os.path as osp
import os


BALL_SQUARENESS_THRESHOLD = 60.0  # squareness threshold (in percent)
BALL_V_UPPER_THRESHOLD = 40  # used to ignore 'ball objects' at the very top of the image
BASKET_V_UPPER_THRESHOLD = 20


class Detector:
    def __init__(self, colorConfig, name, mode):
        self.name = name
        f = open(colorConfig)
        self.minhue = int(f.readline())
        self.minsat = int(f.readline())
        self.minint = int(f.readline())
        self.maxhue = int(f.readline())
        self.maxsat = int(f.readline())
        self.maxint = int(f.readline())
        self.mode = mode.lower()
        f.close()

    def detect(self, frame, hsv):
        cx = -1
        cy = -1
        contour_area = -1
        w = -1
        h = -1
        # Threshold the HSV image to get only necessary colors
        lowerColor = np.array([self.minhue, self.minsat, self.minint])
        upperColor = np.array([self.maxhue, self.maxsat, self.maxint])
        mask = cv2.inRange(hsv, lowerColor, upperColor)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:

            contourArea = []
            # find the biggest area
            for i in contours:
                ix, iy, x, y, w, h, c_area = self.get_details(i)
                contourArea.append(c_area)

            j = 0
            s = 0
            for i in range(len(contours)):  # find biggest object + check if eligible
                if contourArea[i] > s and self.check_contour(contours[i]):
                    s = contourArea[i]
                    j = i

            #c = max(contours, key=cv2.contourArea)
            c = contours[j]
            area = cv2.minAreaRect(c)
            cx, cy, x, y, w, h, contour_area = self.get_details(c)

            if cx > 0 and cy > 0:
                # draw the book contour (in green)
                cv2.circle(res, (cx, cy), 5, (0, 255, 0), -1)
                cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return res, mask, cx, cy, contour_area, w, h

    def get_details(self, contour):
        contour_area = cv2.contourArea(contour)
        M = cv2.moments(contour)
        try:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        except:
            cx = -1
            cy = -1
            contour_area = -1
        x, y, w, h = cv2.boundingRect(contour)
        return cx, cy, x, y, w, h, contour_area

    def check_contour(self, contour):
        # convenience function
        cx, cy, x, y, w, h, contour_area = self.get_details(contour)
        if self.mode == 'ball':
            return self.check_ball(cx, cy, w, h, contour_area)
        elif self.mode == 'basket':
            return self.check_ball(cx, cy, w, h, contour_area)
        else:
            print("INVALID MODE FOR OBJECT DETECTOR!!!")
            return False

    def check_ball(self, cx, cy, w, h, contour_area):
        # checks if ball could actually be a ball
        # (by checking for negative values and comparing with certain size-related thresholds)
        if cx < 0 or cy < 0 or w < 0 or h < 0 or contour_area < 0:
            return False
        if cy < BALL_V_UPPER_THRESHOLD:  # should not be on upper camera edge
            return False
        squareness = round((float(min(w, h)) / max(w, h)) * 100, 2) if w > 0 and h > 0 else 0.0
        if squareness < BALL_SQUARENESS_THRESHOLD or contour_area > 3000:
            return False
        return True

    def check_basket(self, cx, cy, w, h, contour_area):
        # checks if basket could actually be a basket
        # (by checking for negative values and comparing with certain size-related thresholds)
        if cx < 0 or cy < 0 or w < 0 or h < 0 or contour_area < 0:
            return False
        if w > h:  # height should be greater than width
            return False
        if cy < BASKET_V_UPPER_THRESHOLD:  # should not be on upper camera edge
            return False
        return True
