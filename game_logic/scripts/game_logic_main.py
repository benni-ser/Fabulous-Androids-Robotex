#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
from general.msg import Thrower
import math

RATE = 8
ROBOT_SPEED = 15  # general speed used for the robot

THROWER_SPEED = 1500  # min 1200, max
THROWER_ANGLE = 800  # min 800, max 1500

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DISTANCE_THRESHOLD = 390  # based on vertical ball position
CENTER = 320  # 640 / 2
CENTER_HALF_WIDTH = 25
CENTER_LEFT_BORDER = CENTER - CENTER_HALF_WIDTH
CENTER_RIGHT_BORDER = CENTER + CENTER_HALF_WIDTH

BASKET_LEFT_BORDER = CENTER - CENTER_HALF_WIDTH
BASKET_RIGHT_BORDER = CENTER + CENTER_HALF_WIDTH

NOT_DETECTED = "not detected"
CENTERED = "centered"
LEFT = "left of center"
RIGHT = "right of center"
FINISH = "Ball right in front"  # Ball is in right position, basket still has to be centered
THROW_BALL = "Throw ball"

W1ANGLE = 60
W2ANGLE = 300
W3ANGLE = 180
CAM_FOV = 69.4  # field of view of camera (in degrees)
DEGREE_PER_PIXEL = CAM_FOV / IMAGE_WIDTH  # 0.1084375


class Logic:
    def __init__(self):
        self.ball_state = NOT_DETECTED
        self.basket_state = NOT_DETECTED
        self.last_basket = NOT_DETECTED  # last basket seen
        self.ball_x = -1
        self.ball_y = -1
        self.basket_x = -1
        self.basket_y = -1
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)
        self.thrower_pub = rospy.Publisher("thrower_config", Thrower, queue_size=10)

    # Task1
    def ball_callback(self, point):
        self.ball_x = point.x
        self.ball_y = point.y
        # print("Ball point: \n" + str(point) + '\n')
        if self.ball_state != THROW_BALL:
            if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:
                if point.y > DISTANCE_THRESHOLD:  # Ball is both centered and close enough
                    self.ball_state = FINISH
                else:
                    self.ball_state = CENTERED
            elif 0 <= point.x < CENTER_LEFT_BORDER:
                self.ball_state = LEFT
            elif point.x > CENTER_RIGHT_BORDER:
                self.ball_state = RIGHT
            else:  # x = -1
                self.ball_state = NOT_DETECTED

    def basket_callback(self, basketpoint):
        self.basket_x = basketpoint.x
        self.basket_y = basketpoint.y
        if BASKET_LEFT_BORDER <= basketpoint.x <= BASKET_RIGHT_BORDER:
            self.basket_state = CENTERED
        elif 0 <= basketpoint.x < BASKET_LEFT_BORDER:
            self.basket_state = LEFT
            self.last_basket = LEFT
        elif basketpoint.x > BASKET_RIGHT_BORDER:
            self.basket_state = RIGHT
            self.last_basket = RIGHT
        else:  # x = -1
            self.basket_state = NOT_DETECTED

    def calc_and_send_speeds(self, direction_angle=90, speed=0, rotation=0):
        # convenience method to save about five characters
        print("Game logic sends")
        self.speed_pub.publish(calc_speeds(direction_angle, speed, rotation))


def circle(speed=ROBOT_SPEED):
    # rotates around axis in front of the robot
    # speed: positive -> move left; negative -> move right
    return Speeds(0, 0, speed)


def calc_speeds(direction_angle=90, speed=0, rotation=0):
    # calculates omni-motion speeds for each wheel
    # direction_angle: 90 -> forward, 0 -> left, 180 -> right
    # rotation_speed: positive -> turn right, negative -> turn left
    # Formula: wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + robotAngularVelocity
    w1speed = round(speed * math.cos(math.radians(direction_angle - W1ANGLE)) + rotation, 2)
    w2speed = round(speed * math.cos(math.radians(direction_angle - W2ANGLE)) + rotation, 2)
    w3speed = round(speed * math.cos(math.radians(direction_angle - W3ANGLE)) + rotation, 2)
    return Speeds(w1speed, w2speed, w3speed)


def drive_to_ball(l):
    # aim: drive to ball until it is centered and in front of robot (y ~ 400)
    # TODO parameter for ball distance -> adjust speed (and rotational speed)
    ball_angle = (CENTER - l.ball_x) * DEGREE_PER_PIXEL  # angle between central axis and ball (max: +/- 34.7)
    rotational_speed = 0
    if abs(ball_angle) > 1.0:
        rotational_speed = -min(ROBOT_SPEED, abs(ball_angle * ROBOT_SPEED * 0.05))
    rotational_speed = -rotational_speed if ball_angle < 0 else rotational_speed
    moving_speed = 0 if l.ball_y > DISTANCE_THRESHOLD else ROBOT_SPEED  # if ball is very close just rotate to center it
    # basket_bias = (l.ball_x - l.basket_y) * (90.0 / IMAGE_WIDTH) if l.basket_state != NOT_DETECTED else 0
    moving_angle = (90 - ball_angle)#  + basket_bias
    l.calc_and_send_speeds(moving_angle, moving_speed, rotational_speed)


def find_basket(l):
    # called if ball is centered, but basket has yet to be found and centered as well
    if l.basket_state == LEFT or (l.basket_state == NOT_DETECTED and l.last_basket == LEFT):
        l.speed_pub.publish(circle(-ROBOT_SPEED))
    elif l.basket_state == RIGHT or (l.basket_state == NOT_DETECTED and l.last_basket == RIGHT):
        l.speed_pub.publish(circle(ROBOT_SPEED))
    elif l.basket_state == CENTERED:
        l.speed_pub.publish(Speeds(0, 0, 0))
        l.ball_state = THROW_BALL
    else:  # both basket_state and last_basket are NOT_DETECTED
        l.speed_pub.publish(circle(-ROBOT_SPEED))


if __name__ == '__main__':
    try:
        l = Logic()
        # rospy.spin()
        rate = rospy.Rate(RATE)
        i = 0  # counting variable for searching a ball for a certain period
        j = 0  # counting variable throwing a ball for a certain period
        while not rospy.is_shutdown():
            #print("Ball state: x -> {}\ty -> {}  \t{}".format(l.ball_x, l.ball_y, l.ball_state))
            #print("Basket state: x -> {}\ty -> {}  \t{}".format(l.basket_x, l.basket_y, l.basket_state))
            if l.ball_state == NOT_DETECTED:
                if i < RATE * 6:  # turn for x seconds (should be adjusted), to find the ball
                    l.calc_and_send_speeds(rotation=ROBOT_SPEED)
                    i += 1
                elif i < RATE * 9:  # move forward for a few seconds, if ball still not detected
                    l.calc_and_send_speeds(speed=ROBOT_SPEED)
                    i += 1
                else:  # start over with rotating, if ball not detected
                    i = 0
            elif l.ball_state in [CENTERED, LEFT, RIGHT]:
                drive_to_ball(l)
            elif l.ball_state == FINISH:
                # circle around ball until basket is centered
                find_basket(l)
            elif l.ball_state == THROW_BALL:
                if j < RATE * 3:  # try to throw ball for 3 seconds (needs adjusting)
                    l.speed_pub.publish(Speeds(10, -10, 0))
                    l.thrower_pub.publish(Thrower(THROWER_SPEED, THROWER_ANGLE))
                    j += 1
                else:  # start over with ball searching after throw
                    l.ball_state = NOT_DETECTED
                    l.thrower_pub.publish(Thrower(0, THROWER_ANGLE))
                    j = 0
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
