#! /usr/bin/env python
import rospy
import math
import time
from comport_mainboard import ComportMainboard
from general.msg import Speeds
from general.msg import Thrower

RATE = 50
LISTEN_TO_REFEREE_COMMANDS = True
WAIT_FOR_START_SIGNAL = False  # set to True in competitions
FIELD_ID = "A"
ROBOT_ID = "A"
COMMAND_TIMEOUT = 0.8 # in seconds


class MainboardRunner:
    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("speeds", Speeds, self.speeds_callback)
        rospy.Subscriber("thrower_config", Thrower, self.thrower_callback)
        self.board = ComportMainboard()
        self.running = not WAIT_FOR_START_SIGNAL
        self.same_speed_last_time = time.time()
        self.same_thrower_last_time = time.time()
        self.last_Speeds = [0, 0, 0]
        self.last_Thrower = [0, 0]

    def run(self):
        self.board.run()
        if LISTEN_TO_REFEREE_COMMANDS:
            time.sleep(.5)
            r = rospy.Rate(RATE)
            while not rospy.is_shutdown():
                self.check_for_referee_commands()
                r.sleep()
        else:
            rospy.spin()

        print("closing board")
        self.board.close()

    def speeds_callback(self, speeds):
        if self.running:
            # print("Speeds: {}; {}; {}".format(speeds.left, speeds.right, speeds.back))
            self.set_speeds(speeds.left, speeds.right, speeds.back)

    def thrower_callback(self, thrower_config):
        if self.running:
            # print("Thrower: speed -> {}; angle -> {}".format(thrower_config.speed, thrower_config.angle))
            self.set_thrower(thrower_config.speed, thrower_config.angle)

    def set_speeds(self, front_left, front_right, back):
        if [front_left, front_right, back] != self.last_Speeds or self.is_timeout_reached("speeds"):
            print("Speeds: {}; {}; {}".format(front_left, front_right, back))
            self.board.write("sd:{}:{}:{}:0".format(front_left, front_right, back))
            self.same_speed_last_time = time.time()
            self.last_Speeds = [front_left, front_right, back]
            return # self.board.read_line()

    def set_thrower(self, speed, angle):
        if [speed, angle] != self.last_Thrower or self.is_timeout_reached("thrower"):
            print("Thrower: speed -> {}; angle -> {}".format(speed, angle))
            self.board.write("d:{}".format(speed))
            if angle != self.last_Thrower[1]:
                self.board.write("sv:{}".format(angle))
            self.same_thrower_last_time = time.time()
            self.last_Thrower = [speed, angle]
            return

    def get_speeds(self):
        self.board.write('gs')
        return self.board.read()

    def check_for_referee_commands(self):
        line = self.board.read() #_line(True)  # TODO check if this works
        if line and len(line) > 7 and line.startswith("<ref:a") and line[6] == FIELD_ID and (
                line[7] == ROBOT_ID or line[7] == "X"):
            print("REFEREE COMMAND RECEIVED: " + line)
            if line.startswith("START", 8):
                self.running = True
            elif line.startswith("STOP", 8):
                self.running = False
                self.set_speeds(0, 0, 0)
            elif not line.startswith("PING", 8):
                return
            if line[7] == ROBOT_ID:  # ACK should only be sent if robot was messaged individually
                self.board.write("rf:a{}{}ACK-----".format(FIELD_ID, ROBOT_ID))

    def is_timeout_reached(self, mode):
        last_time = self.same_speed_last_time if mode == 'speeds' else self.same_thrower_last_time
        return time.time() - last_time >= COMMAND_TIMEOUT


if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass
