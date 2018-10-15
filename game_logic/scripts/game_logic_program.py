#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
from general.msg import Point
from general.msg import Speeds
from general.msg import BasketPoint
import cv2
import math

CENTER_WIDTH_LEFT = 20
CENTER_WIDTH_RIGHT = 20
CENTER_LEFT_BORDER = 360 - CENTER_WIDTH_LEFT
CENTER_RIGHT_BORDER = 360 + CENTER_WIDTH_RIGHT

BASKET_WIDTH_LEFT = 40
BASKET_WIDTH_RIGHT = 40
BASKET_LEFT_BORDER = 360 - BASKET_WIDTH_LEFT
BASKET_RIGHT_BORDER = 360 + BASKET_WIDTH_RIGHT

CENTERED = "Ball is centered"
LEFT_OF_CENTER = "Ball is left of center"
RIGHT_OF_CENTER = "Ball is right of center"
NOT_DETECTED = "No ball detected"
STOP = "Robot must stop"
FINISH = "Ball process finished"
BASKET_NOT_DETECTED = "Basket not detected"
BASKET_LEFT_OF_CENTER = "Basket left of center"
BASKET_RIGHT_OF_CENTER = "Basket right of center"
BASKET_CENTERED = "Basket centered"
THROW_BALL = "Throw ball"

X = -1
Y = -1

BASKET_X = -1
BASKET_Y = -1

W1ANGLE = 60
W2ANGLE = 300
W3ANGLE = 180
CAM_FOV = 69.4 # field of view of camera (in degrees)

ROBOT_SPEED = 8

class Logic():
    def __init__(self):
        self.state = NOT_DETECTED
	self.b_state = BASKET_NOT_DETECTED
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
	rospy.Subscriber("basket_coordinates", BasketPoint, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)
    #Task1
    def ball_callback(self, point):
        X = point.x
        Y = point.y
        print("Ball point: \n" + str(point)+ '\n')
	if self.state != STOP and self.state != FINISH and self.state != THROW_BALL:
        	if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:  # point should be in middle third
            		if point.y < 460:
	    			print(CENTERED + '\n')
            			self.state = CENTERED
		    	else:
				print(STOP + '\n')
				self.state = STOP
        	elif 0 <= point.x < CENTER_LEFT_BORDER:
            		print(LEFT_OF_CENTER)
            		self.state = LEFT_OF_CENTER
        	elif point.x > CENTER_RIGHT_BORDER:
            		print(RIGHT_OF_CENTER)
            		self.state = RIGHT_OF_CENTER
        	else:  # x = -1
            		self.state = NOT_DETECTED
    #Task2
    '''def ball_callback(self,point):
	global X
	X = point.x
        global Y
	Y = point.y
        print(str(point))
        if self.state != STOP:
                if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:  # point should be in middle third
                        if point.y < 460:
                                print(CENTERED)
                                self.state = CENTERED
                        else:
                                print(STOP)
                                self.state = STOP
                elif 0 <= point.x < CENTER_LEFT_BORDER:
                        if point.y < 460:
                        	print(LEFT_OF_CENTER)
                        	self.state = LEFT_OF_CENTER
                        else:
                                print(STOP)
                                self.state = STOP
                elif point.x > CENTER_RIGHT_BORDER:
                        if point.y < 460:
                        	print(RIGHT_OF_CENTER)
                        	self.state = RIGHT_OF_CENTER
                        else:
                                print(STOP)
                                self.state = STOP
                else:  # x = -1
                        self.state = NOT_DETECTED'''

    def basket_callback(self, basketpoint):
		global BASKET_X
		BASKET_X = basketpoint.x
        	global BASKET_Y
		BASKET_Y = basketpoint.y
        	print("Basket point:\n" + str(basketpoint)+'\n')
        	if self.state == FINISH or self.b_state == BASKET_NOT_DETECTED or self.b_state == BASKET_LEFT_OF_CENTER or self.b_state == BASKET_RIGHT_OF_CENTER:
                	if BASKET_LEFT_BORDER <= basketpoint.x <= BASKET_RIGHT_BORDER:  # point should be in middle third
                        	self.b_state = BASKET_CENTERED
                	elif 0 <= basketpoint.x < BASKET_LEFT_BORDER:
                        	self.b_state = BASKET_LEFT_OF_CENTER
                	elif basketpoint.x > BASKET_RIGHT_BORDER:
                        	self.b_state = BASKET_RIGHT_OF_CENTER
                	else:  # x = -1
                        	self.b_state = BASKET_NOT_DETECTED


def move_forward(speed=ROBOT_SPEED):
    return Speeds(speed, (-1) * speed, 0, 0)


def move_backwards(speed=ROBOT_SPEED):
    return Speeds((-1) * speed, speed, 0, 0)


def rotate_left(speed=ROBOT_SPEED):
    # rotates left around its own axis
    return Speeds(-speed, -speed, -speed, 0)

def rotate_right(speed=ROBOT_SPEED):
    # rotates right around its own axis
    return Speeds(speed, speed, speed, 0)


def circle(speed=ROBOT_SPEED):
    # rotates around axis in front of the robot
    # speed: positive -> move left; negative -> move right
    return Speeds(0, 0, speed, 0)


def calc_linear_velocity(robotSpeed, robotDirectionAngle, wheelAngle):
    print(math.radians(robotDirectionAngle - wheelAngle))
    wheelLinearVelocity = robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle))
    return round(wheelLinearVelocity, 2)

#Task1
def drive_to_ball(l):
    if l.state == LEFT_OF_CENTER:
        #l.speed_pub.publish(rotate_left())
	w1speed = calc_linear_velocity(ROBOT_SPEED, 180, W1ANGLE)
        w2speed = calc_linear_velocity(ROBOT_SPEED, 180, W2ANGLE)
        w3speed = calc_linear_velocity(ROBOT_SPEED, 180, W3ANGLE)
        l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
        print("Moving left: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))
    elif l.state == RIGHT_OF_CENTER:
        #l.speed_pub.publish(rotate_right())
	w1speed = calc_linear_velocity(ROBOT_SPEED, 0, W1ANGLE)
        w2speed = calc_linear_velocity(ROBOT_SPEED, 0, W2ANGLE)
        w3speed = calc_linear_velocity(ROBOT_SPEED, 0, W3ANGLE)
        l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
        print("Moving right: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))
    if l.state == CENTERED:
        #l.speed_pub.publish(Speeds(0, 0, 0, 10))

        #SUBTASK1
        w1speed = calc_linear_velocity(ROBOT_SPEED, 90, W1ANGLE)
        w2speed = calc_linear_velocity(ROBOT_SPEED, 90, W2ANGLE)
        w3speed = calc_linear_velocity(ROBOT_SPEED, 90, W3ANGLE)
        l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
        print("Moving forward: "+str(w1speed)+":"+str(w2speed)+":"+str(w3speed))

#Task2
def drive_to_ball_angle(l):
    # drives to the ball while keeping it at the edge of the camera
    # intended for second subtask of week 3
    if l.state != CENTERED:
	plus = 0
	if l.state == LEFT_OF_CENTER:
		plus = 5
	if l.state == RIGHT_OF_CENTER:
		plus = -5
    	robot_angle = calculate_robot_angle(X) + plus
    	print("Robot_angle: " + str(robot_angle) + " Robot x : " + str(X))
    	w1speed = calc_linear_velocity(ROBOT_SPEED, robot_angle, W1ANGLE)
    	w2speed = calc_linear_velocity(ROBOT_SPEED, robot_angle, W2ANGLE)
    	w3speed = calc_linear_velocity(ROBOT_SPEED, robot_angle, W3ANGLE)
    	l.speed_pub.publish(Speeds(w1speed, w2speed, w3speed, 0))
    else:
	l.speed_pub.publish(rotate_right())
def calculate_robot_angle(x):
	return ((70 - ((x*70)/640)) + 55)



if __name__ == '__main__':
    try:
        l = Logic()
        #rospy.spin()
	#Worked find ball and basket
        rate = rospy.Rate(2)
	#rate = rospy.Rate(8)
	x = 0
	r = 0
	#l.state = THROW_BALL
        while not rospy.is_shutdown():
	    print("Main state: "+l.state+'\n')
	    print("Basket state: "+l.b_state+'\n')
            if l.state == NOT_DETECTED:
		if x == 0:
			l.speed_pub.publish(move_forward(20))
			x = 1
		else:
			l.speed_pub.publish(rotate_right(1))
			x = 0
            elif l.state != STOP and l.state != FINISH and l.state != THROW_BALL:
		#Task1
                drive_to_ball(l)
		#Task2
		#drive_to_ball_angle(l)
            elif l.state == STOP:
		if r < 1:
			#Task1
			l.speed_pub.publish(move_forward())
			#Task2
			#l.speed_pub.publish(move_forward())
			r = r + 1
			rate.sleep()
		else:
			l.state = FINISH
		l.speed_pub.publish(Speeds(0,0,0,0))
            elif l.state == FINISH:
		if l.b_state == BASKET_LEFT_OF_CENTER:
			print(BASKET_LEFT_OF_CENTER)
			l.speed_pub.publish(circle(-5))
		elif l.b_state == BASKET_RIGHT_OF_CENTER:
			print(BASKET_RIGHT_OF_CENTER)
			l.speed_pub.publish(circle(5))
		elif l.b_state == BASKET_CENTERED:
			print(BASKET_CENTERED)
			l.speed_pub.publish(Speeds(0,0,0,0))
			l.state = THROW_BALL
		else:
			print(BASKET_NOT_DETECTED)
			l.speed_pub.publish(rotate_right())
	    elif l.state == THROW_BALL:
		l.speed_pub.publish(Speeds(10,-10,0,2000))
	    rate.sleep()
    except rospy.ROSInterruptException:
        pass
