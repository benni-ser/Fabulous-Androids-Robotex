#! /usr/bin/env python
import rospy
import serial
import threading
import time
import subprocess
from std_msgs.msg import String
# import hardware_module.comport_mainboard as mainboard
# from hardware.msg import Launcher


class ComportMainboard(threading.Thread):
    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)

    def open(self):
        try:
            ports = subprocess.check_output('ls /dev/ttyACM0', shell=True).split('\n')[:-1]
        except:
            print('mainboard: /dev/ttyACM empty')
            return False
        self.connection_opened = False
        for port in ports:  # analyze serial ports
            try:
                while not self.connection_opened and not rospy.is_shutdown():
                    self.connection = serial.Serial(port, baudrate=115200, timeout=0.8, dsrdtr=True)
                    self.connection_opened = self.connection.isOpen()
                    time.sleep(0.5)
                self.connection.flush()
                print("mainboard: Port opened successfully")
            except Exception as e:
                print(e)
                continue

        return self.connection_opened

    def write(self, comm):
        if self.connection is not None:
            try:
                self.connection.write(comm + '\n')
            except:
                print('mainboard: err write ' + comm)

    def servo(self, value):
        msg = "v{}".format(value)
        if self.connection_opened:
            self.write(msg)

    def launch_motor(self, value):
        if self.connection_opened:
            self.write("d{}".format(value))


    def test_led(self):
        if self.connection_opened:
            self.write("r")


    def close(self):
        if self.connection is not None and self.connection.isOpen():  # close coil
            try:
                self.connection.close()
                print('mainboard: connection closed')
            except:
                print('mainboard: err connection close')
            self.connection = None

    def run(self):
        if self.open():  # open serial connections
            print('mainboard: opened')
        else:
            print('mainboard: opening failed')
            self.close()
            return


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
            # self.sendSpeeds()
            self.board.test_led()
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
