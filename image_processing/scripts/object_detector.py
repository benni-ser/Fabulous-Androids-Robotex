import cv2
import numpy as np

BALL_SQUARENESS_THRESHOLD = 60.0  # squareness threshold (in percent)
BALL_V_UPPER_THRESHOLD = 40  # used to ignore 'ball objects' at the very top of the image
BASKET_AREA_THRESHOLD = 750  # threshold for minimal contour area of basket object


class Detector:
    def __init__(self, color_config, name, mode):
        self.name = name
        f = open(color_config)
        self.minhue = int(f.readline())
        self.maxhue = int(f.readline())
        self.minsat = int(f.readline())
        self.maxsat = int(f.readline())
        self.minint = int(f.readline())
        self.maxint = int(f.readline())
        self.mode = mode.lower()
        f.close()

    def detect(self, frame, hsv, lower_hsv=[], upper_hsv=[]):
        cx = -1
        cy = -1
        contour_area = -1
        w = -1
        h = -1
        # Threshold the HSV image to get only necessary colors
        lower_color = np.array(lower_hsv) if len(lower_hsv) > 0 else np.array([self.minhue, self.minsat, self.minint])
        upper_color = np.array(upper_hsv) if len(upper_hsv) > 0 else np.array([self.maxhue, self.maxsat, self.maxint])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        # kernel = np.ones((3, 3), np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:

            contour_area = []
            # find the biggest area
            for i in contours:
                ix, iy, x, y, w, h, c_area = get_details(i)
                contour_area.append(c_area)

            j = -1
            s = 0
            for i in range(len(contours)):  # find biggest object + check if eligible
                if contour_area[i] > s and self.check_contour(contours[i]):
                    s = contour_area[i]
                    j = i

            if j == -1:  # no suitable object found
                return res, -1, -1, -1, -1, -1

            # c = max(contours, key=cv2.contourArea)
            c = contours[j]
            # area = cv2.minAreaRect(c)
            cx, cy, x, y, w, h, contour_area = get_details(c)

            if cx > 0 and cy > 0:
                # draw center and bounding rectangle of object (in green)
                cv2.circle(res, (cx, cy), 5, (0, 255, 0), -1)
                cv2.rectangle(res, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return res, cx, cy, contour_area, w, h

    def check_contour(self, contour):
        # convenience function
        cx, cy, x, y, w, h, contour_area = get_details(contour)
        if self.mode == 'ball':
            return check_ball(contour)
        elif self.mode == 'basket':
            return check_basket(cx, cy, w, h, contour_area)
        else:
            print("INVALID MODE FOR OBJECT DETECTOR!!!")
            return False


def get_details(contour):
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


def check_ball(contour):
    # checks if ball could actually be a ball
    # (by checking for negative values and comparing with certain size-related thresholds)
    # TODO exclude objects that are too big considering their approximate distance
    cx, cy, x, y, w, h, contour_area = get_details(contour)
    if cx < 0 or cy < 0 or w < 0 or h < 0 or contour_area < 0:
        return False
    if cy < BALL_V_UPPER_THRESHOLD:  # should not be on upper camera edge
        return False
    squareness = round((float(min(w, h)) / max(w, h)) * 100, 2) if w > 0 and h > 0 else 0.0
    if (squareness < BALL_SQUARENESS_THRESHOLD and y + h < 480) or contour_area > 3000:
        return False
    return True


def check_basket(cx, cy, w, h, contour_area):
    # checks if basket could actually be a basket
    # (by checking for negative values and comparing with certain size-related thresholds)
    if cx < 0 or cy < 0 or w < 0 or h < 0 or contour_area < 0:
        return False
    if w > h:  # height should be greater than width
        return False
    if contour_area < BASKET_AREA_THRESHOLD:
        return False
    return True
