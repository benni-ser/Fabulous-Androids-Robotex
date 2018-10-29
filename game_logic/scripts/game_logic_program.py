#! /usr/bin/env python
import rospy
from general.msg import Point
from general.msg import Speeds
import math

RATE = 2
ROBOT_SPEED = 8  # general speed used for the robot

CENTER_WIDTH_LEFT = 20
CENTER_WIDTH_RIGHT = 20
CENTER_LEFT_BORDER = 350 - CENTER_WIDTH_LEFT
CENTER_RIGHT_BORDER = 350 + CENTER_WIDTH_RIGHT

BASKET_WIDTH_LEFT = 40
BASKET_WIDTH_RIGHT = 40
BASKET_LEFT_BORDER = 350 - BASKET_WIDTH_LEFT
BASKET_RIGHT_BORDER = 350 + BASKET_WIDTH_RIGHT

NOT_DETECTED = "not detected"
CENTERED = "centered"
LEFT_OF_CENTER = "left of center"
RIGHT_OF_CENTER = "right of center"
STOP = "Robot must stop"
FINISH = "Ball process finished"
THROW_BALL = "Throw ball"

W1ANGLE = 60
W2ANGLE = 300
W3ANGLE = 180
CAM_FOV = 69.4  # field of view of camera (in degrees)

ball_x = -1
ball_y = -1
basket_x = -1
basket_y = -1


class Logic:
    def __init__(self):
        self.ball_state = NOT_DETECTED
        self.basket_state = NOT_DETECTED
        rospy.init_node("game_logic", anonymous=True)
        rospy.Subscriber("ball_coordinates", Point, self.ball_callback)
        rospy.Subscriber("basket_coordinates", Point, self.basket_callback)
        self.speed_pub = rospy.Publisher("speeds", Speeds, queue_size=10)

    # Task1
    def ball_callback(self, point):
        global ball_x
        ball_x = point.x
        global ball_y
        ball_y = point.y
        print("Ball point: \n" + str(point) + '\n')
        if self.ball_state != STOP and self.ball_state != FINISH and self.ball_state != THROW_BALL:
            if CENTER_LEFT_BORDER <= point.x <= CENTER_RIGHT_BORDER:  # point should be in middle third
                if point.y < 460:
                    print(CENTERED + '\n')
                    self.ball_state = CENTERED
                else:
                    print(STOP + '\n')
                    self.ball_state = STOP
            elif 0 <= point.x < CENTER_LEFT_BORDER:
                print(LEFT_OF_CENTER)
                self.ball_state = LEFT_OF_CENTER
            elif point.x > CENTER_RIGHT_BORDER:
                print(RIGHT_OF_CENTER)
                self.ball_state = RIGHT_OF_CENTER
            else:  # x = -1
                self.ball_state = NOT_DETECTED

    def basket_callback(self, basketpoint):
        global basket_x
        basket_x = basketpoint.x
        global basket_y
        basket_y = basketpoint.y
        print("Basket point:\n" + str(basketpoint) + '\n')
        if self.ball_state == FINISH or self.basket_state == NOT_DETECTED or self.basket_state == LEFT_OF_CENTER or self.basket_state == RIGHT_OF_CENTER:
            if BASKET_LEFT_BORDER <= basketpoint.x <= BASKET_RIGHT_BORDER:  # point should be in middle third
                self.basket_state = CENTERED
            elif 0 <= basketpoint.x < BASKET_LEFT_BORDER:
                self.basket_state = LEFT_OF_CENTER
            elif basketpoint.x > BASKET_RIGHT_BORDER:
                self.basket_state = RIGHT_OF_CENTER
            else:  # x = -1
                self.basket_state = NOT_DETECTED


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


def calc_omnimotion_speeds(direction_angle, speed=ROBOT_SPEED):
    # print(math.radians(robotDirectionAngle - wheelAngle))
    w1speed = round(speed * math.cos(math.radians(direction_angle - W1ANGLE)), 2)
    w2speed = round(speed * math.cos(math.radians(direction_angle - W2ANGLE)), 2)
    w3speed = round(speed * math.cos(math.radians(direction_angle - W3ANGLE)), 2)
    return Speeds(w1speed, w2speed, w3speed, 0)


# Task1
def drive_to_ball(l):
    if l.state == LEFT_OF_CENTER:
        # l.speed_pub.publish(rotate_left())
        l.speed_pub.publish(calc_omnimotion_speeds(180))
        # print("Moving left: " + str(w1speed) + ":" + str(w2speed) + ":" + str(w3speed))
    elif l.state == RIGHT_OF_CENTER:
        # l.speed_pub.publish(rotate_right())
        l.speed_pub.publish(calc_omnimotion_speeds(0))
        # print("Moving right: " + str(w1speed) + ":" + str(w2speed) + ":" + str(w3speed))
    if l.state == CENTERED:
        # l.speed_pub.publish(Speeds(0, 0, 0, 10))

        # SUBTASK1
        l.speed_pub.publish(calc_omnimotion_speeds(90))
        # print("Moving forward: " + str(w1speed) + ":" + str(w2speed) + ":" + str(w3speed))


# Task2
def drive_to_ball_angle(l):
    # drives to the ball while keeping it at the edge of the camera
    # intended for second subtask of week 3
    if l.state != CENTERED:
        plus = 0
        if l.state == LEFT_OF_CENTER:
            plus = 5
        if l.state == RIGHT_OF_CENTER:
            plus = -5
        robot_angle = calculate_robot_angle(ball_x) + plus
        print("Robot_angle: " + str(robot_angle) + " Robot x : " + str(ball_x))
        l.speed_pub.publish(calc_omnimotion_speeds(robot_angle))
    else:
        l.speed_pub.publish(rotate_right())


def calculate_robot_angle(x):
    return (70 - ((x * 70) / 640)) + 55


if __name__ == '__main__':
    try:
        l = Logic()
        # rospy.spin()
        # Worked find ball and basket
        rate = rospy.Rate(RATE)
        # rate = rospy.Rate(8)
        x = 0
        r = 0
        # l.state = THROW_BALL
        while not rospy.is_shutdown():
            print("Main state: " + l.ball_state + '\n')
            print("Basket state: " + l.basket_state + '\n')
            if l.ball_state == NOT_DETECTED:
                if x == 0:
                    l.speed_pub.publish(move_forward(20))
                    x = 1
                else:
                    l.speed_pub.publish(rotate_right(1))
                    x = 0
            elif l.ball_state != STOP and l.ball_state != FINISH and l.ball_state != THROW_BALL:
                # Task1
                drive_to_ball(l)
            # Task2
            # drive_to_ball_angle(l)
            elif l.ball_state == STOP:
                if r < 1:
                    # Task1
                    l.speed_pub.publish(move_forward())
                    # Task2
                    # l.speed_pub.publish(move_forward())
                    r = r + 1
                    rate.sleep()
                else:
                    l.ball_state = FINISH
                l.speed_pub.publish(Speeds(0, 0, 0, 0))
            elif l.ball_state == FINISH:
                if l.basket_state == LEFT_OF_CENTER:
                    print(LEFT_OF_CENTER)
                    l.speed_pub.publish(circle(-5))
                elif l.basket_state == RIGHT_OF_CENTER:
                    print(RIGHT_OF_CENTER)
                    l.speed_pub.publish(circle(5))
                elif l.basket_state == CENTERED:
                    print(CENTERED)
                    l.speed_pub.publish(Speeds(0, 0, 0, 0))
                    l.ball_state = THROW_BALL
                else:
                    print(NOT_DETECTED)
                    l.speed_pub.publish(rotate_right())
            elif l.ball_state == THROW_BALL:
                l.speed_pub.publish(Speeds(10, -10, 0, 2000))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
