#! /usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from hardware.comport_mainboard import ComportMainboard
# import hardware_module.comport_mainboard as mainboard
# from hardware.msg import Launcher


class MainboardRunner():

    board = None
    launcherParams = String

    def launcher_params_callback(self, launcherMsg):
       self.launcherParams = launcherMsg

    def run(self):
        rospy.init_node("comport_mainboad", anonymous=True)
        rospy.Subscriber("hardware_module/launcher", String, self.launcher_params_callback)
        self.board = ComportMainboard()
        self.board.run()

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            self.board.write("r\n")
		break
            rate.sleep()

        print("closing board")
        self.board.close()

    def sendSpeeds(self):
        self.board.servo(self.launcherParams.servoPos)
        self.board.launch_motor(self.launcherParams.motorSpeed)

if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass
