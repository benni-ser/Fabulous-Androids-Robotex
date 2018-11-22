import cv2
import numpy as np
import os.path as osp
import pyrealsense2 as rs
import sys
import time
from object_detector import Detector


def get_frame():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    if not aligned_depth_frame or not color_frame:
        return
    color_frame = np.asanyarray(color_frame.get_data())
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return color_frame, hsv_frame


def cancel():
    print("Parameters:")
    print("\t(optional) color config file (without .txt) [ball_green, basket_blue, baket_red]")
    print("\t(optional) detection mode [ball, basket]")
    print("Example: python color_calibration.py ball_green ball")
    sys.exit()


def nothing():
    pass


root_path = '/home/intel/catkin_ws/src/image_processing/config'
mode = ''
detector = None

# upper and lower boundaries
lh = 0
uh = 180
ls = 0
us = 255
lv = 0
uv = 255

if len(sys.argv) > 1:
    filename = osp.join(root_path, sys.argv[1] + '.txt')
    if osp.isfile(filename):
        f = open(filename)
        lh = int(f.readline())
        uh = int(f.readline())
        ls = int(f.readline())
        us = int(f.readline())
        lv = int(f.readline())
        uv = int(f.readline())

    if len(sys.argv) == 3:
        if sys.argv[2] in ['ball', 'basket']:
            mode = sys.argv[2]
            detector = Detector(filename, 'Detector', mode)


lower_hsv = np.array([lh, ls, lv])
upper_hsv = np.array([uh, us, uv])


# set up camera pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

# creating a window for later use
cv2.namedWindow('result')

# create trackbars
cv2.createTrackbar('LowerH', 'result', 0, 180, nothing)
cv2.setTrackbarPos('LowerH', 'result', lh)
cv2.createTrackbar('UpperH', 'result', 0, 180, nothing)
cv2.setTrackbarPos('UpperH', 'result', uh)

cv2.createTrackbar('LowerS', 'result', 0, 255, nothing)
cv2.setTrackbarPos('LowerS', 'result', ls)
cv2.createTrackbar('UpperS', 'result', 0, 255, nothing)
cv2.setTrackbarPos('UpperS', 'result', us)

cv2.createTrackbar('LowerV', 'result', 0, 255, nothing)
cv2.setTrackbarPos('LowerV', 'result', lv)
cv2.createTrackbar('UpperV', 'result', 0, 255, nothing)
cv2.setTrackbarPos('UpperV', 'result', uv)

font = cv2.FONT_HERSHEY_SIMPLEX

while 1:
    # Threshold the HSV image
    frame, hsv = get_frame()
    # frame = cv2.medianBlur(frame, 5)
    if mode:
        result, _, _, _, _, _ = detector.detect(frame, hsv, lower_hsv, upper_hsv)
    else:
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('result', result)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of Upper HSV trackbars
    uh = cv2.getTrackbarPos('UpperH', 'result')
    us = cv2.getTrackbarPos('UpperS', 'result')
    uv = cv2.getTrackbarPos('UpperV', 'result')
    upper_blue = np.array([uh, us, uv])
    # get current positions of Lower HSV trackbars
    lh = cv2.getTrackbarPos('LowerH', 'result')
    ls = cv2.getTrackbarPos('LowerS', 'result')
    lv = cv2.getTrackbarPos('LowerV', 'result')
    upper_hsv = np.array([uh, us, uv])
    lower_hsv = np.array([lh, ls, lv])

    time.sleep(.1)


cv2.destroyAllWindows()
