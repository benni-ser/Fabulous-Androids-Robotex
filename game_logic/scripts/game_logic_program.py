#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
from image_processing.msg import Point
import cv2

class Gamer():
	def __init__(self):
		rospy.init_node("game_logic", anonymous=True)
	        rospy.Subscriber("ball_coordinates", Point, self.callback)
	
	def callback(self, point):
		print(str(point))
		if point.x > 220 and point.x < 420: # point should be in middle third
			print("Ball is in the middle third!")
			# and point.y > 160


if __name__ == '__main__':
    try:
	g = Gamer()
	rate = rospy.Rate(60)
	while not rospy.is_shutdown():
		rate.sleep()
    except rospy.ROSInterruptException:
	pass
