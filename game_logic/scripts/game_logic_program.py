#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
from general.msg import Point
from general.msg import Speeds
import cv2

CENTER_LEFT_BORDER = 220
CENTER_RIGHT_BORDER = 420

class Logic():
        def __init__(self):
		self.ball_detected = False
                rospy.init_node("game_logic", anonymous=True)
                rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
                self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)

        def ball_callback(self, point):
                print(str(point))
		self.ball_detected = True
                if point.x >= CENTER_LEFT_BORDER and point.x <= CENTER_RIGHT_BORDER: # point should be in middle third
			print("Ball is in the center!")
			self.speed_pub.publish(Speeds(0, 0, 0, 0))
                        # and point.y > 160
                elif point.x < CENTER_LEFT_BORDER and point.x >= 0:
			print("Ball is left of center")
			self.speed_pub.publish(rotate(-5))
		elif point.x > CENTER_RIGHT_BORDER:
			print("Ball is right of center")
                        self.speed_pub.publish(rotate(5))
		else: # x = -1
			self.ball_detected = False

def move_forward(speed):
	return Speeds(speed, (-1) * speed, 0, 0)

def move_backwards(speed):
	return Speeds((-1)*speed, speed, 0, 0)

def rotate(speed):
	# rotates around its own axis
	# speed: positive -> turn right; negative -> turn left
	return Speeds(speed, speed, speed, 0)

def circle(speed):
	# rotates around axis in front of the robot
	# speed: positive -> move left; negative -> move right
	return Speeds(0, 0, speed, 0)



if __name__ == '__main__':
    try:
        logic = Logic()
        # rospy.spin()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
		if not logic.ball_detected:
			logic.speed_pub.publish(rotate(-10))
                rate.sleep()
    except rospy.ROSInterruptException:
        pass
