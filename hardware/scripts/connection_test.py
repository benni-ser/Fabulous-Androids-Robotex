#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
import cv2

class MainboardRunner():

    board = None

    def run(self):
        rospy.init_node("connection_test", anonymous=True)
        self.board = ComportMainboard()
        self.board.run()

        '''print(self.board.getDirection())
            self.board.write("g")
        print(self.board.setDirection("10:10:0:0"))
        print(self.board.getDirection())'''

        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            #self.board.setDirection("10:10:0:0")
            print('test')
            r.sleep()

        #self.board.write('r\n')
        '''r=rospy.Rate(30)
            while not rospy.is_shutdown():
                #self.board.setDirection("10:10:0:0")
            print(self.board.getDirection())
            self.board.write("g")
            r.sleep()'''

        print("closing board")
        self.board.close()



if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass
