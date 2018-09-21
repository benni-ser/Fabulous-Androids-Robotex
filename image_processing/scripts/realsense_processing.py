#!/usr/bin/env python
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
from image_processing.ball_color_processing import detector

class RealsenseProcessing():
    def __init__(self):
        rospy.init_node("realsense_processing", anonymous=True)
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


if __name__ == '__main__':
    try:
        camera_proc = RealsenseProcessing()
        camera_proc.run()
	rate = rospy.Rate(1)
	i = 0
	while not rospy.is_shutdown():
            camera_proc.get_frame()
	    real_ball_detector = detector("/home/intel/catkin_ws/src/image_processing/config/ball_colour_file.txt", "BallDetector")
	    result = real_ball_detector.detect(camera_proc.regular_image, camera_proc.hsv)
            test = np.array(camera_proc.hsv)
            l, w, v = test.shape
            if i % 1 == 0:
                print(str(test[l/2, w/2, :]))
            i += 1
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
