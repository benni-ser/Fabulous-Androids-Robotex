#!/usr/bin/env python
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
# from image_processing.object_detector import Detector
from object_detector import Detector
from general.msg import Point
import os
import os.path as osp

# Configuration
RATE = 16
SAVE_BALL_IMGS = True
SAVE_BASKET_IMGS = True
SAVE_FREQUENCY = 1  # save picture every x seconds
PRINT_INFO = False
BASKET_COLOR = "red"  # options: 'blue' or 'red'

COLOR_CONFIG_PATH = "/home/intel/catkin_ws/src/image_processing/config"
IMG_PATH = "/home/intel/pics"


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
        self.session_idx = [int(float(name[:4])) for name in os.listdir(IMG_PATH) if
                            osp.isfile(osp.join(IMG_PATH, name)) and len(name) > 4 and name[:4].isdigit()]
        self.session_idx = max(self.session_idx) + 1 if self.session_idx else 0
        self.image_idx = 0

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

    def save_images(self):
        if SAVE_BALL_IMGS or SAVE_BASKET_IMGS:
            root = "{}/{}-{}-".format(IMG_PATH, str(self.session_idx).zfill(4), str(self.image_idx).zfill(4))
            cv2.imwrite("{}pic_({},{})_sq{}.png".format(root, cx, cy, squareness), cam_proc.regular_image)
            if SAVE_BALL_IMGS:
                details = "{},{},{},{},{}".format(cx, cy, w, h, contour_area)
                cv2.imwrite("{}ball_({}).png".format(root, details), ball_res)
            if SAVE_BASKET_IMGS:
                details = "{},{},{},{},{}".format(basket_cx, int(round(basket_cy + basket_h / 2)), basket_w, basket_h,
                                                  basket_contour_area)
                cv2.imwrite("{}basket_({}).png".format(root, details), basket_res)
            print("Saved images: Session {}, Image {}".format(self.session_idx, self.image_idx))
            self.image_idx += 1


if __name__ == '__main__':
    try:
        cam_proc = RealsenseProcessing()
        cam_proc.run()
        rate = rospy.Rate(RATE)
        i = 0
        while not rospy.is_shutdown():
            cam_proc.get_frame()

            ball_detector = Detector(osp.join(COLOR_CONFIG_PATH, "ball_green.txt"), "BallDetector", "ball")
            ball_res, cx, cy, contour_area, w, h = ball_detector.detect(cam_proc.regular_image, cam_proc.hsv)
            cam_proc.pub_ball.publish(Point(cx, cy, 0))

            basket_detector = Detector(osp.join(COLOR_CONFIG_PATH, "basket_{}.txt".format(BASKET_COLOR)),
                                       "BasketDetector", "basket")
            basket_res, basket_cx, basket_cy, basket_contour_area, basket_w, basket_h = basket_detector.detect(
                cam_proc.regular_image, cam_proc.hsv)
            cam_proc.pub_basket.publish(Point(basket_cx, int(round(basket_cy + basket_h / 2)), 0))
            # print("Lower border: " + str(int(round(basket_cy + basket_h/2))))

            if i % int(RATE * SAVE_FREQUENCY) == 0 and i != 0:  # for debugging/analysis purposes
                squareness = round((float(min(w, h)) / max(w, h)) * 100, 2) if w > 0 and h > 0 else 0.0
                cam_proc.save_images()
                if PRINT_INFO:
                    # test = np.array(cam_proc.hsv)
                    # l, w, v = test.shape
                    # print("Color of middle point: "+str(test[l/2, w/2, :]))
                    print("Ball{} detected!".format("" if cx != -1 else " NOT"))
                    print("contour_area: " + str(contour_area))
                    print("w:{:3}\th:{:3}\tw+h:{}".format(str(w), str(h), str(w + h)))
                    print("\"Squareness\" (in percent): {}".format(str(squareness)))
                    print("cx: " + str(cx))
                    print("cy: " + str(cy))
                    print("______________________")
            i += 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
