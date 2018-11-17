#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
import math

RATE = 8
ROBOT_SPEED = 15  # general speed used for the robot

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DISTANCE_THRESHOLD = 390  # based on vertical ball position
CENTER = 390  # 390 because camera is not in center
CENTER_HALF_WIDTH = 25
CENTER_LEFT_BORDER = CENTER - CENTER_HALF_WIDTH
CENTER_RIGHT_BORDER = CENTER + CENTER_HALF_WIDTH

BASKET_LEFT_BORDER = CENTER - CENTER_HALF_WIDTH
BASKET_RIGHT_BORDER = CENTER + CENTER_HALF_WIDTH

NOT_DETECTED = "not detected"
CENTERED = "centered"
LEFT_OF_CENTER = "left of center"
RIGHT_OF_CENTER = "right of center"
STOP = "Robot must stop"  # obsolete
FINISH = "Ball is right in front of us"  # Ball is in right position, basket still has to be centered
THROW_BALL = "Throw ball"

W1ANGLE = 60
W2ANGLE = 300
W3ANGLE = 180
CAM_FOV = 69.4  # field of view of camera (in degrees) 55.3 - 124.7


class Logic:
    def __init__(self):
        self.ball_state = NOT_DETECTED
        self.basket_state = NOT_DETECTED
        self.ball_x = -1
        self.ball_y = -1
        self.basket_x = -1
        self.basket_y = -1
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)

    # Task1
    def ball_callback(self, point):
        self.ball_x = point.x
        self.ball_y = point.y
        print("Ball point: \n" + str(point) + '\n')
        if self.ball_state != THROW_BALL:
            if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:  # point should be in middle third
                if point.y > DISTANCE_THRESHOLD:  # Ball is both centered and close enough
                    self.ball_state = FINISH
                else:
                    self.ball_state = CENTERED
            elif 0 <= point.x < CENTER_LEFT_BORDER:
                self.ball_state = LEFT_OF_CENTER
            elif point.x > CENTER_RIGHT_BORDER:
                self.ball_state = RIGHT_OF_CENTER
            else:  # x = -1
                self.ball_state = NOT_DETECTED

    def basket_callback(self, basketpoint):
        self.basket_x = basketpoint.x
        self.basket_y = basketpoint.y
        print("Basket point:\n" + str(basketpoint) + '\n')
        #if self.ball_state == FINISH or self.basket_state == NOT_DETECTED or self.basket_state == LEFT_OF_CENTER or self.basket_state == RIGHT_OF_CENTER:
        if BASKET_LEFT_BORDER <= basketpoint.x <= BASKET_RIGHT_BORDER:  # point should be in middle third
            self.basket_state = CENTERED
        elif 0 <= basketpoint.x < BASKET_LEFT_BORDER:
            self.basket_state = LEFT_OF_CENTER
        elif basketpoint.x > BASKET_RIGHT_BORDER:
            self.basket_state = RIGHT_OF_CENTER
        else:  # x = -1
            self.basket_state = NOT_DETECTED

    def calc_and_send_speeds(self, direction_angle=90, speed=0, rotation=0):
        # convenience method to save about five characters
        self.speed_pub.publish(calc_speeds(direction_angle, speed, rotation))


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


def calc_speeds(direction_angle=90, speed=0, rotation=0):
    # calculates omni-motion speeds for each wheel
    # direction_angle: 90 -> forward, 0 -> left, 180 -> right
    # rotation_speed: positive -> turn right, negative -> turn left
    # Formula -> wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + wheelDistanceFromCenter * robotAngularVelocity
    w1speed = round(speed * math.cos(math.radians(direction_angle - W1ANGLE)) + 0.15 * rotation, 2)
    w2speed = round(speed * math.cos(math.radians(direction_angle - W2ANGLE)) + 0.15 * rotation, 2)
    w3speed = round(speed * math.cos(math.radians(direction_angle - W3ANGLE)) + 0.15 * rotation, 2)
    print(str(w1speed) + ";" + str(w2speed) + ";" + str(w3speed))
    return Speeds(w1speed, w2speed, w3speed, 0)


def drive_to_ball(l):
    # aim: drive to ball until it is centered and in front of robot (y ~ 400)
    # TODO try to check if basket is already visible while driving to ball
    if l.ball_y > DISTANCE_THRESHOLD:
        if l.ball_state == LEFT_OF_CENTER:
            l.calc_and_send_speeds(90, 0, -10)  # rotate left
        elif l.ball_state == RIGHT_OF_CENTER:
            l.calc_and_send_speeds(90, 0, 10)  # rotate right
        elif l.ball_state == CENTERED:
            l.calc_and_send_speeds(90, 0, 0)
    else:
        angle = 90 - (CENTER - l.ball_x) * (CAM_FOV / IMAGE_WIDTH)  # 0.1084375
        if l.ball_state == LEFT_OF_CENTER:
            l.calc_and_send_speeds(angle, ROBOT_SPEED, -10)
        elif l.ball_state == RIGHT_OF_CENTER:
            l.calc_and_send_speeds(angle, ROBOT_SPEED, 10)
        elif l.ball_state == CENTERED:
            l.calc_and_send_speeds(angle, ROBOT_SPEED, 0)


def find_basket(l):
    # called if ball is centered, but basket has yet to be found and centered as well
    if l.basket_state == LEFT_OF_CENTER:
        l.speed_pub.publish(circle(-5))
    elif l.basket_state == RIGHT_OF_CENTER:
        l.speed_pub.publish(circle(5))
    elif l.basket_state == CENTERED:
        l.speed_pub.publish(Speeds(0, 0, 0, 0))
        l.ball_state = THROW_BALL
    else:
        l.speed_pub.publish(rotate_right())


if __name__ == '__main__':
    try:
        l = Logic()
        # rospy.spin()
        rate = rospy.Rate(RATE)
        i = 0  # counting variable for searching a ball for a certain period
        j = 0  # counting variable throwing a ball for a certain period
        while not rospy.is_shutdown():
            print("Main state: " + l.ball_state + '\n')
            print("Basket state: " + l.basket_state + '\n')
            if l.ball_state == NOT_DETECTED:
                if i < RATE * 7:  # turn for 7 seconds (should be adjusted), to find the ball
                    l.calc_and_send_speeds(rotation=10)
                    i += 1
                elif i < RATE * 9:  # move forward for 2 seconds, if ball still not detected
                    l.calc_and_send_speeds(speed=10)
                    i += 1
                else:  # start over with rotating, if ball not detected
                    i = 0
            elif l.ball_state in [CENTERED, LEFT_OF_CENTER, RIGHT_OF_CENTER]:
                drive_to_ball(l)
            elif l.ball_state == FINISH:
                # circle around ball until basket is centered
                find_basket(l)
            elif l.ball_state == THROW_BALL:
                if j < RATE * 3:  # try to throw ball for 3 seconds (needs adjusting)
                    l.speed_pub.publish(Speeds(10, -10, 0, 2000))
                    j += 1
                else:  # start over with ball searching after throw
                    l.ball_state = NOT_DETECTED
                    j = 0
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
