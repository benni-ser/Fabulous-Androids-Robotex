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
	def callback(self, message):
		print(str(message))
		print("Test1")


if __name__ == '__main__':
    try:
	g = Gamer()
    except rospy.ROSInterruptException:
	pass