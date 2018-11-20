#! /usr/bin/env python
import rospy
from comport_mainboard import ComportMainboard
from general.msg import Speeds
from general.msg import Thrower

RATE = 50
LISTEN_TO_REFEREE_COMMANDS = True
FIELD_ID = "A"
ROBOT_ID = "A"


class MainboardRunner:
    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("speeds", Speeds, self.speeds_callback)
        rospy.Subscriber("thrower_config", Thrower, self.thrower_callback)
        self.board = ComportMainboard()
        self.running = not LISTEN_TO_REFEREE_COMMANDS
        self.last_Speeds = Speeds(0, 0, 0)
        self.last_Thrower = Thrower(0, 0)

    def run(self):
        self.board.run()
        if LISTEN_TO_REFEREE_COMMANDS:
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
            print("Speeds: {}; {}; {}".format(speeds.left, speeds.right, speeds.back))
            self.set_dir(speeds.left, speeds.right, speeds.back)

    def thrower_callback(self, thrower_config):
        if self.running:
            print("Thrower: speed -> {}; angle -> {}".format(thrower_config.speed, thrower_config.angle))
            self.set_thrower(thrower_config.speed, thrower_config.angle)

    def set_dir(self, front_left, front_right, back):
        self.board.write("sd:{}:{}:{}:0".format(front_left, front_right, back))
        return self.board.read_line()

    def set_thrower(self, speed, angle):
        self.board.write("d:{}".format(speed))
        self.board.write("sv:{}".format(angle))
        return self.board.read_line()

    def get_dir(self):
        self.board.write('gs')
        return self.board.read_line()

    def check_for_referee_commands(self):
        line = self.board.read()  # TODO check if this works
        if line and len(line) > 7 and line.startswith("<ref:a") and line[6] == FIELD_ID and (
                line[7] == ROBOT_ID or line[7] == "X"):
            print("REFEREE COMMAND RECEIVED: " + line)
            if line.startswith("START", 8):
                self.running = True
            elif line.startswith("STOP", 8):
                self.running = False
                self.set_dir(0, 0, 0)
            elif not line.startswith("PING", 8):
                return
            if line[7] == ROBOT_ID:  # ACK should only be sent if robot was messaged individually
                self.board.write("rf:a{}{}ACK-----".format(FIELD_ID, ROBOT_ID))


if __name__ == '__main__':
    try:
        mainboard_runner = MainboardRunner()
        mainboard_runner.run()
    except rospy.ROSInterruptException:
        pass
