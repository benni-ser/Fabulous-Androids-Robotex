#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
import math

RATE = 16
ROBOT_SPEED = 20  # general speed used for the robot

THROWER_CALIBRATION_MODE = True
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
FINISH = "Ball right in front"  # Ball is in right position, but basket is not yet centered
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
        self.speeds = [0, 0, 0]
        self.thrower_speed = 0
        self.servo = 0
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)
        # self.thrower_pub = rospy.Publisher("thrower_config", Thrower, queue_size=10)

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

    def publish_speeds(self):
        to_send = Speeds(self.speeds[0], self.speeds[1], self.speeds[2], self.thrower_speed, self.servo)
        self.speed_pub.publish(to_send)

    def drive_to_ball(self):
        # aim: drive to ball until it is centered and in front of robot (y ~ 400)
        # TODO parameter for ball distance -> adjust speed (and rotational speed)
        ball_angle = (self.ball_x - CENTER) * DEGREE_PER_PIXEL  # angle between central axis and ball (max: +/- 34.7)
        rotational_speed = 0
        if abs(ball_angle) > 1.0:
            rotational_speed = min(ROBOT_SPEED, abs(ball_angle * ROBOT_SPEED * 0.05))
            rotational_speed = -rotational_speed if ball_angle < 0 else rotational_speed
        moving_speed = 0 if self.ball_y > DISTANCE_THRESHOLD else ROBOT_SPEED  # if ball is very close just rotate
        rotational_speed = min(ROBOT_SPEED, rotational_speed * 2) if moving_speed == 0 else rotational_speed # if moving_speed == 0, rotate faster
        # basket_bias = (self.ball_x - self.basket_y) * (90.0 / IMAGE_WIDTH) if self.basket_state != NOT_DETECTED else 0
        moving_angle = (90 - ball_angle) # + basket_bias
        self.calc_speeds(moving_angle, moving_speed, rotational_speed)

    def find_basket(self):
        # called if ball is centered, but basket has yet to be found and centered as well
        if self.basket_state == NOT_DETECTED:
            self.circle(-ROBOT_SPEED if self.last_basket == LEFT else ROBOT_SPEED)
        else:
            basket_angle = (self.basket_x - CENTER) * DEGREE_PER_PIXEL
            if abs(basket_angle) > 1.0:
                circle_speed = max(7, min(ROBOT_SPEED, abs(basket_angle * ROBOT_SPEED * 0.1)))
                circle_speed = -circle_speed if basket_angle < 0 else circle_speed
                self.circle(int(round(circle_speed)))
            else:
                self.speeds = [0, 0, 0]
                self.ball_state = THROW_BALL

    def throw_ball(self, speed=-1, angle=-1):
        # basket_y: max ~70, min ~450
        # speed: 1400 - 2100 -> 700 range
        # angle: 800 - 1500 -> 700 range
        # TODO speeds are too high in practice
        # TODO map directly to thrower speeds with lookup table
        if speed != -1:
            basket_distance = min(max(0, IMAGE_HEIGHT - self.basket_y - 25), 400)  # not yet real distance
            # basket_distance = round(1.0101**(basket_distance-20), 2) # between 0 and 40
            basket_distance = round(23.3 - 0.888 * (1 - math.exp(basket_distance*0.0154)), 2)
            print("Basket distance: {}".format(basket_distance))
            speed = 1400 + basket_distance * 1.2  # 15, 12, 10, 10, 8.5
        angle = 800 if angle == -1 else angle # + basket_distance * 17.5
        self.speeds = [10, -10, 0]  # drive forward
        self.thrower_speed = int(round(speed, -1))  # round to tens
        self.servo = int(round(angle, -1))  # round to tens

    def circle(self, speed=ROBOT_SPEED):
        # rotates around axis in front of the robot
        # speed: positive -> move left; negative -> move right
        self.speeds = [0, 0, speed]

    def calc_speeds(self, direction_angle=90.0, speed=0.0, rotation=0.0):
        # calculates omni-motion speeds for each wheel
        # direction_angle: 90 -> forward, 0 -> left, 180 -> right
        # rotation_speed: positive -> turn right, negative -> turn left
        # Formula: wheelLinearVelocity = robotSpeed * cos(robotDirectionAngle - wheelAngle) + robotAngularVelocity
        w1speed = int(round(speed * math.cos(math.radians(direction_angle - W1ANGLE)) + rotation))
        w2speed = int(round(speed * math.cos(math.radians(direction_angle - W2ANGLE)) + rotation))
        w3speed = int(round(speed * math.cos(math.radians(direction_angle - W3ANGLE)) + rotation))
        self.speeds = [w1speed, w2speed, w3speed]

    def act_competition_mode(self, i, j):
        if self.ball_state != THROW_BALL:
            self.thrower_speed = 0
            self.servo = 0
        if self.ball_state == NOT_DETECTED:
            if i < RATE * 2:  # drive forward for x seconds (should be adjusted), to find the ball
                self.calc_speeds(speed=ROBOT_SPEED)
                i += 1
            elif i < RATE * 7:  # turn for a few seconds, if ball still not detected
                self.calc_speeds(rotation=ROBOT_SPEED)
                i += 1
            else:  # start over with rotating, if ball not detected
                i = 0
        elif self.ball_state in [CENTERED, LEFT, RIGHT]:
            self.drive_to_ball()
        elif self.ball_state == FINISH:
            # circle around ball until basket is centered
            self.find_basket()
        elif self.ball_state == THROW_BALL:
            if j < RATE * 2:  # try to throw ball for 2 seconds (needs adjusting)
                self.throw_ball()
                j += 1
            if j < RATE * 4 and THROWER_CALIBRATION_MODE:
                self.speeds = [-10, 10, 0]
                self.thrower_speed = 0
                self.servo = -1
                j += 1
            else:  # start over with ball searching after throw
                self.ball_state = NOT_DETECTED if not THROWER_CALIBRATION_MODE else FINISH
                j = 0
        self.publish_speeds()
        return i, j


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
