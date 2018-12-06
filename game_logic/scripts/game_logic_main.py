#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
import math
import numpy as np
import bisect
import time

RATE = 16
ROBOT_SPEED = 25.0  # general speed used for the robot

# configs for thrower calibration mode
THROWER_CALIBRATION_MODE = False
thrower_test_speeds = [1550, 1550, 1550]
thrower_test_angle = 800
FREE_THROW_MODE = True
free_throw_speed = 1550

# thrower-related settings
THROWER_ANGLES = [800, 1200]  # min 800, max 1500
ANGLE_OFFSETS = [0, 2]  # in horizontal degrees, for each thrower angle
ANGLE_THRESHOLD = 0.6
THROWER_SPEED_BIASES = [0, -25] # in thrower speed, for each thrower angle

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DISTANCE_THRESHOLDS = [390, 290, 200, 120]  # for speeds [0, ROBOT_SPEED(RS), RS*1.5, RS*2]
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
FINISH = "Ball right in front"  # Ball is in right position, but basket is not yet centered
THROW_BALL = "Throw ball"

THROWER_SPEEDS_PATH = "/home/intel/catkin_ws/src/game_logic/config/thrower_speeds"

WHEEL_ANGLES = [60, 300, 180]
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
        self.speeds = [0, 0, 0]
        self.thrower_speed = 0
        self.servo = 0
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)
        # in-memory lookup table for thrower speeds (2-dimensional)
        self.thrower_lookups = []
        for idx in range(len(THROWER_ANGLES)):
            lookup = np.loadtxt("{}_{}.csv".format(THROWER_SPEEDS_PATH, idx), dtype='int16', delimiter=",", skiprows=1)
            if len(lookup.shape) == 1:
                lookup = np.reshape(lookup, (1, lookup.size))
            self.thrower_lookups.append(lookup[lookup[:, 0].argsort()]) # order by basket_y from close to far away

    def ball_callback(self, point):
        self.ball_x = point.x
        self.ball_y = point.y
        if THROWER_CALIBRATION_MODE:
            return
        if self.ball_state != THROW_BALL:
            if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:
                #if self.ball_state == FINISH:
                    #self.ball_state = CENTERED if point.y < DISTANCE_THRESHOLD - 7 else FINISH
                if point.y > DISTANCE_THRESHOLDS[0]:  # Ball is both centered and close enough
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

    def publish_speeds(self):
        # print("Sending speeds: {}".format(self.speeds))
        servo = THROWER_ANGLES[self.servo] if self.servo < 10 else self.servo
        to_send = Speeds(self.speeds[0], self.speeds[1], self.speeds[2], self.thrower_speed, servo)
        self.speed_pub.publish(to_send)

    def drive_to_ball(self):
        # aim: drive to ball until it is centered and in front of robot (y ~ 400)
        # TODO parameter for ball distance -> adjust speed (and rotational speed)
        ball_angle = (self.ball_x - CENTER) * DEGREE_PER_PIXEL  # angle between central axis and ball (max: +/- 34.7)

        # DISTANCE_THRESHOLDS = [390, 290, 200, 120]
        moving_speed = 0  # if ball is very close just rotate
        if DISTANCE_THRESHOLDS[0] > self.ball_y > DISTANCE_THRESHOLDS[1]:
            moving_speed = ROBOT_SPEED * 0.75
        elif DISTANCE_THRESHOLDS[1] > self.ball_y > DISTANCE_THRESHOLDS[2]:
            moving_speed = ROBOT_SPEED * 1.0
        elif DISTANCE_THRESHOLDS[2] > self.ball_y > DISTANCE_THRESHOLDS[3]:
            moving_speed = ROBOT_SPEED * 1.5
        elif DISTANCE_THRESHOLDS[3] > self.ball_y:
            moving_speed = ROBOT_SPEED * 1.7

        rotational_speed = 0
        if abs(ball_angle) > ANGLE_THRESHOLD:  # if ball not in center -> rotate towards it
            rotational_speed = abs(ball_angle * ROBOT_SPEED * 0.05)
            rotational_speed = -rotational_speed if ball_angle < 0 else rotational_speed
        if moving_speed == 0:  # if moving_speed == 0, rotate faster
            rotational_speed *= 1.5
            rotational_speed = max(7.0, abs(rotational_speed))
        rotational_speed = min(ROBOT_SPEED, abs(rotational_speed))
        rotational_speed = -rotational_speed if ball_angle < 0 else rotational_speed
        # basket_bias = (self.ball_x - self.basket_y) * (90.0 / IMAGE_WIDTH) if self.basket_state != NOT_DETECTED else 0
        moving_angle = (90 - ball_angle) # + basket_bias
        self.calc_speeds(moving_angle, moving_speed, rotational_speed)

    def find_basket(self):
        # called if ball is centered, but basket has yet to be found and centered as well
        if self.basket_state == NOT_DETECTED:
            self.circle(-ROBOT_SPEED if self.last_basket == LEFT else ROBOT_SPEED)
        else:
            basket_angle = (self.basket_x - CENTER) * DEGREE_PER_PIXEL# angle from center to basket
            #print("Basket angle: {}".format(basket_angle))
            basket_angle += ANGLE_OFFSETS[0] if self.basket_y > 100 else ANGLE_OFFSETS[1]
            if abs(basket_angle) > ANGLE_THRESHOLD:
                circle_speed = max(7.0, min(ROBOT_SPEED, abs(basket_angle * ROBOT_SPEED * 0.15)))
                circle_speed = -circle_speed if basket_angle < 0 else circle_speed
                self.circle(int(round(circle_speed)))
            else:
                self.speeds = [0, 0, 0]
                self.ball_state = THROW_BALL

    def throw_ball(self, speed=-1, angle=-1):
        # basket_y: max ~70, min ~450
        # speed: 1400 - 2100 -> 700 range
        # angle: 800 - 1500 -> 700 range
        self.speeds = [10, -10, 0]  # drive forward
        if angle == -1:
            self.servo = 0 if self.basket_y > 100 else 1
        else:
            self.servo = angle
        if speed == -1:
            self.calc_thrower_speed()
            if self.servo == 0:
                self.thrower_speed += THROWER_SPEED_BIASES[0]
            else:
                self.thrower_speed += THROWER_SPEED_BIASES[1]
        else:
            self.thrower_speed = int(round(speed, -1))  # round to tens
        print("Throwing ball->\tbasket_y: {}\tspeed: {}".format(self.basket_y, self.thrower_speed))

    def circle(self, speed=ROBOT_SPEED):
        # rotates around axis in front of the robot
        # speed: positive -> move left; negative -> move right
        self.speeds = [0, 0, speed]

    def calc_speeds(self, direction_angle=90.0, speed=0.0, rotation=0.0):
        # calculates omni-motion speeds for each wheel
        # direction_angle: 90 -> forward, 0 -> left, 180 -> right
        # rotation_speed: positive -> turn right, negative -> turn left
        # Formula: wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + robotAngularVelocity
        for w in range(3):
            self.speeds[w] = int(round(speed * math.cos(math.radians(direction_angle - WHEEL_ANGLES[w])) + rotation))

    def calc_thrower_speed(self):
        # get (sorted) list with all position/speed combis that used the same angle
        # get two closest instances and interpolate new speed based on position
        # lookup is ordered from closest (high value) to farthest instance (lower value)
        lookup = self.thrower_lookups[self.servo]
        distances = lookup[:,0]
        k = bisect.bisect_left(distances.tolist(), self.basket_y)
        if  k == 0 or k == len(distances) or lookup[k, 1] == self.basket_y:
            self.thrower_speed = lookup[min(k, len(distances)-1), 1]
        else:
            lower_anchor = lookup[k]  # closest reference point that is closer to basket than current position
            upper_anchor = lookup[k-1]  # closest reference point that is further away than current position
            speed_per_pixel = abs(float(lower_anchor[1]) - upper_anchor[1]) / abs(float(lower_anchor[0]) - upper_anchor[0])
            final_raw_speed = lower_anchor[1] + speed_per_pixel * abs(lower_anchor[0] - self.basket_y)
            self.thrower_speed = int(round(final_raw_speed * 2, -1) / 2) # round to multiples of 5

    def act_competition_mode(self, i, j):
        if self.ball_state != THROW_BALL:
            self.thrower_speed = 0
            self.servo = 0
            j = 0
        if self.ball_state != NOT_DETECTED:
            i = 0

        if self.ball_state == NOT_DETECTED:
            if i < int(round(RATE * 1)):  # drive forward for x seconds (should be adjusted), to find the ball
                self.calc_speeds(speed=min(20, ROBOT_SPEED))
                i += 1
            elif i < int(round(RATE * 6)):  # drive forward for x seconds (should be adjusted), to find the ball
                self.calc_speeds(rotation=min(20, ROBOT_SPEED))
                i += 1
            elif i < int(round(RATE * 7)):  # turn for a few seconds, if ball still not detected
                self.calc_speeds(speed=min(20, ROBOT_SPEED))
                i += 1
            else:  # start over with rotating, if ball not detected
                i = 0
        elif self.ball_state in [CENTERED, LEFT, RIGHT]:
            self.drive_to_ball()
        elif self.ball_state == FINISH:
            # circle around ball until basket is centered
            self.find_basket()
        elif self.ball_state == THROW_BALL:
            if self.basket_state == NOT_DETECTED:
                self.ball_state = FINISH
            elif j < int(round(RATE * 2.5)):  # try to throw ball for 2 seconds (needs adjusting)
                self.throw_ball()
                j += 1
            else:  # start over with ball searching after throw
                self.ball_state = NOT_DETECTED
                j = 0
        self.publish_speeds()
        return i, j

    def act_thrower_calibration_mode(self, i, j):
        if i >= 0:
            if self.ball_state == THROW_BALL:
                if i < int(round(RATE * 2.5)):  # try to throw ball for 3 seconds
                    self.throw_ball(speed=thrower_test_speeds[j], angle=thrower_test_angle)
                    i += 1
                elif i < int(round(RATE * 5)):  # drive back to original position
                    self.speeds = [-10, 10, 0]
                    self.thrower_speed = 0
                    i += 1
                else:  # start over
                    # self.ball_state = FINISH
                    i = 0 if j < len(thrower_test_speeds)-1 else -1
                    j += 1
            else:
                i = 0
                self.find_basket()
        else:
            self.speeds = [0, 0, 0]
            self.thrower_speed = 0
        self.publish_speeds()
        return i, j

    def act_free_throw_mode(self, i, j):
        if self.ball_state != THROW_BALL:
            self.thrower_speed = 0
            self.servo = 0
            i = 0

        if self.ball_state == NOT_DETECTED:
            self.calc_speeds() # do not move
        elif self.ball_state in [CENTERED, LEFT, RIGHT]:
            self.drive_to_ball()
        elif self.ball_state == FINISH:
            # circle around ball until basket is centered
            self.find_basket()
        elif self.ball_state == THROW_BALL:
            if self.basket_state == NOT_DETECTED:
                self.ball_state = FINISH
            elif i < int(round(RATE * 2.5)):  # try to throw ball for 2 seconds
                self.throw_ball(speed=free_throw_speed)
                i += 1
            elif i < int(round(RATE * 5)):  # move backwards
                self.speeds = [-20, 20, 0]
                i += 1
            else:  # start over with ball searching after throw
                self.ball_state = NOT_DETECTED
                i = 0
        self.publish_speeds()
        return i, j


if __name__ == '__main__':
    try:
        logic = Logic()
        # rospy.spin()
        rate = rospy.Rate(RATE)
        i = 0  # counting variable for searching a ball for a certain period
        j = 0  # counting variable throwing a ball for a certain period
        time.sleep(.5)  # wait for camera to send positions
        while not rospy.is_shutdown():
            if THROWER_CALIBRATION_MODE:
                i, j = logic.act_thrower_calibration_mode(i, j)
            elif FREE_THROW_MODE:
                i, j, = logic.act_free_throw_mode(i, j)
            else:
                i, j = logic.act_competition_mode(i, j)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
