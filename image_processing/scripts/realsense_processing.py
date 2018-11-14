#!/usr/bin/env python
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
#from image_processing.object_detector import Detector
from object_detector import Detector
from general.msg import Point
import os

# constants, configs etc.
RATE = 4
SAVE_BALL_IMGS = True
SAVE_BASKET_IMGS = False


class RealsenseProcessing():
    def __init__(self):
        rospy.init_node("realsense_processing", anonymous=True)
        self.pub_ball = rospy.Publisher('ball_coordinates', Point, queue_size=10)
        self.pub_basket = rospy.Publisher('basket_coordinates', Point, queue_size=10)
        self.pipeline = None
        self.align = None
        self.depth_image = None
        self.regular_image = None
        self.yuv = None
        self.hsv = None

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.regular_image = np.asanyarray(color_frame.get_data())
        self.yuv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)
        self.hsv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2HSV)


def check_ball(cx, cy, w, h, contour_area):
    # checks if ball could actually be a ball
    # (by checking for negative values and comparing with certain size-related thresholds)
    if cx < 0 or cy < 0 or w < 0 or h < 0 or contour_area < 0:
        return False
    squareness = round((float(min(w, h)) / max(w, h)) * 100, 2) if w > 0 and h > 0 else 0.0
    if squareness < 60.0 or contour_area > 3000:
        return False
    return True


def save_images():
    if SAVE_BALL_IMGS or SAVE_BASKET_IMGS:
        path = "/home/intel/pics"
        index = [int(float(name[:5])) for name in os.listdir(path) if os.path.isfile(os.path.join(path, name)) and len(name) > 5 and name[:5].isdigit()]
        index = max(index) + 1 if index else 0
        cv2.imwrite("{}/{}-pic_({},{})_sq{}.png".format(path, str(index).zfill(5), cx, cy, squareness), cam_proc.regular_image)
        if SAVE_BALL_IMGS:
            cv2.imwrite("{}/{}-ball_({},{})_sq{}.png".format(path, str(index).zfill(5), cx, cy, squareness), res)
        if SAVE_BASKET_IMGS:
            cv2.imwrite("{}/{}-basket_({},{})_sq{}.png".format(path, str(index).zfill(5), cx, cy, squareness), res)


if __name__ == '__main__':
    try:
        cam_proc = RealsenseProcessing()
        cam_proc.run()
        rate = rospy.Rate(RATE)
        i = 0
        while not rospy.is_shutdown():
            cam_proc.get_frame()
            ball_detector = Detector("/home/intel/catkin_ws/src/image_processing/config/ball_colour_file.txt", "BallDetector")
            res, mask, cx, cy, contour_area, w, h = ball_detector.detect(cam_proc.regular_image, cam_proc.hsv)
            cam_proc.pub_ball.publish(Point(cx, cy, 0) if check_ball(cx, cy, w, h, contour_area) else Point(-1, -1, 0))

            basket_detector = Detector("/home/intel/catkin_ws/src/image_processing/config/basket_colour_file.txt", "BasketDetector")
            basket_res, basket_mask, basket_cx, basket_cy, basket_contour_area, basket_w, basket_h = basket_detector.detect(cam_proc.regular_image, cam_proc.hsv)
            cam_proc.pub_basket.publish(Point(basket_cx, basket_cy, 0))

            if i % (RATE * 3) == 0:  # for testing purposes
                # test = np.array(cam_proc.hsv)
                # l, w, v = test.shape
                # print("Color of middle point: "+str(test[l/2, w/2, :]))
                print("Ball{} detected!".format("" if check_ball(cx, cy, w, h, contour_area) else " NOT"))
                print("contour_area: " + str(contour_area))
                print("w:{:3}\th:{:3}\tw+h:{}".format(str(w), str(h), str(w + h)))
                squareness = round((float(min(w, h)) / max(w, h)) * 100, 2) if w > 0 and h > 0 else 0.0
                print("\"Squareness\" (in percent): {}".format(str(squareness)))
                print("cx: " + str(cx))
                print("cy: " + str(cy))
                print("______________________")
                save_images()
            i += 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
