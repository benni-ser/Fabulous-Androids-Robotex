#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
from general.msg import Point
from general.msg import Speeds
import cv2
import math

CENTER_WIDTH = 10
CENTER_LEFT_BORDER = 320 - CENTER_WIDTH
CENTER_RIGHT_BORDER = 320 + CENTER_WIDTH

CENTERED = "Ball is centered"
LEFT_OF_CENTER = "Ball is left of center"
RIGHT_OF_CENTER = "Ball is right of center"
NOT_DETECTED = "No ball detected"

W1ANGLE = 120
W2ANGLE = 210
W3ANGLE = 0

ROBOT_SPEED = 10

class Logic():
    def __init__(self):
        self.state = NOT_DETECTED
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)

    def ball_callback(self, point):
        print(str(point))
        if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:  # point should be in middle third
            print(CENTERED)
            self.state = CENTERED
            # and point.y > 160
        elif 0 <= point.x < CENTER_LEFT_BORDER:
            print(LEFT_OF_CENTER)
            self.state = LEFT_OF_CENTER
        elif point.x > CENTER_RIGHT_BORDER:
            print(RIGHT_OF_CENTER)
            self.state = RIGHT_OF_CENTER
        else:  # x = -1
            self.state = NOT_DETECTED


def move_forward(speed):
    return Speeds(speed, (-1) * speed, 0, 0)


def move_backwards(speed):
    return Speeds((-1) * speed, speed, 0, 0)


def rotate(speed):
    # rotates around its own axis
    # speed: positive -> turn right; negative -> turn left
    return Speeds(speed, speed, speed, 0)


def circle(speed):
    # rotates around axis in front of the robot
    # speed: positive -> move left; negative -> move right
    return Speeds(0, 0, speed, 0)

def calculate_linear_velocity(robotSpeed, robotDirectionAngle, wheelAngle):
    print(math.radians(robotDirectionAngle - wheelAngle))
    wheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle))
    return wheelLinearVelocity

if __name__ == '__main__':
    try:
        l = Logic()
        # rospy.spin()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if l.state == NOT_DETECTED:
                l.speed_pub.publish(rotate(10))
            elif l.state == LEFT_OF_CENTER:
		#SUBTASK1
                w1speed = calculate_linear_velocity(ROBOT_SPEED, 0, W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED, 0, W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED, 0, W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
		print("Speeds: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))

		#SUBTASK2
		'''w1speed = calculate_linear_velocity(ROBOT_SPEED, , W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED, , W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED, , W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))'''
            elif l.state == RIGHT_OF_CENTER:
		#SUBTASK1
                w1speed = calculate_linear_velocity(ROBOT_SPEED, 180, W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED, 180, W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED, 180, W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
		print("Speeds: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))


		#SUBTASK2
		'''w1speed = calculate_linear_velocity(ROBOT_SPEED,  , W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED,  , W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED,  , W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))'''
            elif l.state == CENTERED:
                #l.speed_pub.publish(Speeds(0, 0, 0, 10))

		#SUBTASK1
		w1speed = calculate_linear_velocity(ROBOT_SPEED, 90, W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED, 90, W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED, 90, W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
		print("Speeds: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))

		#SUBTASK2
		'''w1speed = calculate_linear_velocity(ROBOT_SPEED,  , W1ANGLE)
                w2speed = calculate_linear_velocity(ROBOT_SPEED,  , W2ANGLE)
                w3speed = calculate_linear_velocity(ROBOT_SPEED,  , W3ANGLE)
                l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))'''
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
