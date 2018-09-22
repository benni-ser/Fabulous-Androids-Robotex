#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
import cv2
from  general.msg import Speeds

class MainboardRunner():
    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("speeds", Speeds, self.speeds_callback)
	self.board = ComportMainboard()

    def run(self):
        self.board.run()
	rospy.spin()

	'''
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.move_forward(10)
            r.sleep()
	'''

        print("closing board")
        self.board.close()

    def speeds_callback(self, speeds):
	print(str(speeds))
	self.set_dir(speeds.left, speeds.right, speeds.back, speeds.thrower)

    def move_forward(self, speed):
        self.set_dir(speed, (-1) * speed, 0)

    def move_backwards(self, speed):
        self.set_dir((-1)*speed, speed, 0)

    def rotate(self, speed):
        # rotates around its own axis
        # speed: positive -> turn right; negative -> turn left
        self.set_dir(speed, speed, speed)

    def circle(self, speed):
        # rotates around axis in front of the robot
        # speed: positive -> move left; negative -> move right
        self.set_dir(0, 0, speed)

    def set_dir(self, front_left, front_right, back, thrower=0):
        self.board.write("sd:{}:{}:{}:{}".format(front_left, front_right, back, thrower))

    def get_dir(self):
        self.board.write('gs')
        return self.board.readline()



if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass
