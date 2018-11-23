#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
from general.msg import Thrower
import math

RATE = 16
ROBOT_SPEED = 30  # general speed used for the robot

THROWER_SPEED = 1500  # min 1200, max
THROWER_ANGLES = [800, 1200]  # min 800, max 1500

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DISTANCE_THRESHOLD = 390  # based on vertical ball position
CENTER = 320  # 640 / 2
CENTER_HALF_WIDTH = 20
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
        self.basket_distance = -1
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
        print("Ball state:\tx -> {}\ty -> {}  \t{}".format(self.ball_x, self.ball_y, self.ball_state))

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
        print("Basket state:\tx -> {}\ty -> {}  \t{}".format(self.basket_x, self.basket_y, self.basket_state))

    def calc_and_send_speeds(self, direction_angle=90, speed=0, rotation=0):
        # convenience method to save about five characters
        print("Game logic sends")
        self.speed_pub.publish(calc_speeds(direction_angle, speed, rotation))

    def drive_to_ball(self):
        # aim: drive to ball until it is centered and in front of robot (y ~ 400)
        # TODO parameter for ball distance -> adjust speed (and rotational speed)
        ball_angle = (self.ball_x - CENTER) * DEGREE_PER_PIXEL  # angle between central axis and ball (max: +/- 34.7)
        rotational_speed = 0
        if abs(ball_angle) > 1.0:
            rotational_speed = min(ROBOT_SPEED, abs(ball_angle * ROBOT_SPEED * 0.05))
            rotational_speed = -rotational_speed if ball_angle < 0 else rotational_speed
        moving_speed = 0 if self.ball_y > DISTANCE_THRESHOLD else ROBOT_SPEED  # if ball is very close just rotate to center it
        basket_bias = (self.ball_x - self.basket_y) * (90.0 / IMAGE_WIDTH) if self.basket_state != NOT_DETECTED else 0
        moving_angle = (90 - ball_angle) + basket_bias
        self.calc_and_send_speeds(moving_angle, moving_speed, rotational_speed)


    def find_basket(self):
        # called if ball is centered, but basket has yet to be found and centered as well
        if self.basket_state == NOT_DETECTED:
            self.speed_pub.publish(circle(-ROBOT_SPEED if self.last_basket == LEFT else ROBOT_SPEED))
        else:
            basket_angle = (self.basket_x - CENTER) * DEGREE_PER_PIXEL
            if abs(basket_angle) > 1.0:
                circle_speed = min(ROBOT_SPEED, abs(basket_angle * ROBOT_SPEED * 0.1))
                circle_speed = -circle_speed if basket_angle < 0 else circle_speed
                self.speed_pub.publish(circle(int(round(circle_speed))))
            else:
                self.speed_pub.publish(Speeds(0, 0, 0))
                self.ball_state = THROW_BALL


    def throw_ball(self):
        # basket_y: max ~30, min ~330
        # speed: 1400 - 2100 -> 700 range
        # angle: 800 - 1500 -> 700 range
        # TODO speeds are too high in practice
        basket_distance = min(max(0, IMAGE_HEIGHT - self.basket_y - 150), 300)  # 150-450 -> 0-300
        speed = int(round(1400 + basket_distance * 2.33, -1))
        angle = int(round(800 + basket_distance * 2.33, -1))
        self.speed_pub.publish(Speeds(10, -10, 0))
        self.thrower_pub.publish(Thrower(speed, angle))


    def act_competition_mode(self, i, j):
        if self.ball_state == NOT_DETECTED:
            if i <  RATE * 2:  # drive forward for x seconds (should be adjusted), to find the ball
                self.calc_and_send_speeds(speed=ROBOT_SPEED)
                i += 1
            elif i < RATE * 7:  # turn for a few seconds, if ball still not detected
                self.calc_and_send_speeds(rotation=ROBOT_SPEED)
                i += 1
            else:  # start over with rotating, if ball not detected
                i = 0
        elif self.ball_state in [CENTERED, LEFT, RIGHT]:
            self.drive_to_ball()
        elif self.ball_state == FINISH:
            # circle around ball until basket is centered
            self.find_basket()
        elif self.ball_state == THROW_BALL:
            if j < RATE * 3:  # try to throw ball for 3 seconds (needs adjusting)
                self.throw_ball()
                j += 1
            else:  # start over with ball searching after throw
                self.ball_state = NOT_DETECTED
                self.thrower_pub.publish(Thrower(0, -1))
                j = 0
        return i, j


def circle(speed=ROBOT_SPEED):
    # rotates around axis in front of the robot
    # speed: positive -> move left; negative -> move right
    return Speeds(0, 0, speed)


def calc_speeds(direction_angle=90, speed=0, rotation=0):
    # calculates omni-motion speeds for each wheel
    # direction_angle: 90 -> forward, 0 -> left, 180 -> right
    # rotation_speed: positive -> turn right, negative -> turn left
    # Formula: wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + robotAngularVelocity
    w1speed = int(round(speed * math.cos(math.radians(direction_angle - W1ANGLE)) + rotation))
    w2speed = int(round(speed * math.cos(math.radians(direction_angle - W2ANGLE)) + rotation))
    w3speed = int(round(speed * math.cos(math.radians(direction_angle - W3ANGLE)) + rotation))
    return Speeds(w1speed, w2speed, w3speed)


if __name__ == '__main__':
    try:
        logic = Logic()
        # rospy.spin()
        rate = rospy.Rate(RATE)
        i = 0  # counting variable for searching a ball for a certain period
        j = 0  # counting variable throwing a ball for a certain period
        while not rospy.is_shutdown():
            i, j = logic.act_competition_mode(i, j)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
