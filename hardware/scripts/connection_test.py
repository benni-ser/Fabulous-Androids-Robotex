#! /usr/bin/env python
import rospy
#from hardware.comport_mainboard import ComportMainboard
from comport_mainboard import ComportMainboard
from general.msg import Speeds

RATE = 50
LISTEN_TO_REFEREE_COMMANDS = False
FIELD_ID = "A"
ROBOT_ID = "A"


class MainboardRunner:
    def __init__(self):
        rospy.init_node("connection_test", anonymous=True)
        rospy.Subscriber("speeds", Speeds, self.speeds_callback)
        self.board = ComportMainboard()
        self.running = True

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
            print(str(speeds))
            self.set_dir(speeds.left, speeds.right, speeds.back, speeds.thrower)

    def set_dir(self, front_left, front_right, back, thrower=0):
        self.board.write("sd:{}:{}:{}:0".format(front_left, front_right, back))
        if thrower > 0:
            self.board.write("d:{}".format(thrower))
        return self.board.read_line()

    def get_dir(self):
        self.board.write('gs')
        return self.board.read_line()

    def check_for_referee_commands(self):
        line = self.board.read_line(False)
        if line and line.startswith("<ref:a") and line[6] == FIELD_ID and (line[7] == ROBOT_ID or line[7] == "X"):
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
