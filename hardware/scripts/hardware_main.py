#! /usr/bin/env python
import rospy
import time
from comport_mainboard import ComportMainboard
from general.msg import Speeds

RATE = 50
LISTEN_TO_REFEREE_COMMANDS = True
WAIT_FOR_START_SIGNAL = False  # set to True in competitions
FIELD_ID = "A"
ROBOT_ID = "A"
COMMAND_TIMEOUT = 0.8  # in seconds


class MainboardRunner:
    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("speeds", Speeds, self.speeds_callback)
        # rospy.Subscriber("thrower_config", Thrower, self.thrower_callback)
        self.board = ComportMainboard()
        self.running = not WAIT_FOR_START_SIGNAL
        self.same_speed_last_time = time.time()
        self.same_thrower_last_time = time.time()
        self.last_speeds = [0, 0, 0]
        self.last_thrower_speed = 0
        self.last_servo = 0

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
            self.set_speeds(speeds.left, speeds.right, speeds.back, speeds.thrower_speed, speeds.angle)

    def set_speeds(self, front_left, front_right, back, thrower_speed, angle):
        if [front_left, front_right, back] != self.last_speeds or self.is_timeout_reached("speeds"):
            print("Speeds update: {}; {}; {}".format(front_left, front_right, back))
            self.board.write("sd:{}:{}:{}:0".format(front_left, front_right, back))
            self.same_speed_last_time = time.time()
            self.last_speeds = [front_left, front_right, back]
        if (thrower_speed != self.last_thrower_speed or self.is_timeout_reached("thrower")):
            if not(thrower_speed == 0 and self.last_thrower_speed == 0):
                print("Thrower update: speed -> {}".format(thrower_speed))
                self.board.write("d:{}".format(thrower_speed))
                self.same_thrower_last_time = time.time()
                self.last_thrower_speed = thrower_speed
        if angle <= 0 and angle != self.last_servo:
            print("Thrower update: angle -> {}".format(angle))
            self.board.write("sv:{}".format(angle))

    def get_speeds(self):
        self.board.write('gs')
        return self.board.read()

    def check_for_referee_commands(self):
        line = self.board.read_line()
        if line and len(line) > 7 and line.startswith("<ref:a") and line[6] == FIELD_ID and (
                line[7] == ROBOT_ID or line[7] == "X"):
            print("REFEREE COMMAND RECEIVED: " + line)
            if line.startswith("START", 8):
                self.running = True
            elif line.startswith("STOP", 8):
                self.running = False
                self.set_speeds(0, 0, 0, 0, 0)
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
