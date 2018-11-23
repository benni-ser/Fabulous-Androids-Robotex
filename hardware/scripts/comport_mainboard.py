import serial
import threading
import time
import subprocess
import rospy


class ComportMainboard(threading.Thread):
    connection = None
    connection_opened = False

    def __init__(self):
        threading.Thread.__init__(self)
        self.last_read = time.time()
        self.start_time = time.time()

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

    def read(self):
        command = ""
        c = self.connection.read()
        while not rospy.is_shutdown() and c != '\n':
            command += c
            c = self.connection.read()
        return command

    def read_line(self, flush=False):
        if self.connection_opened:
            if flush:
                self.connection.flush()

            ''' # for debugging
            time_since_last = round(time.time() - self.last_read, 2)
            exec_time = round(time.time() - self.start_time, 2)
            print("READ_LINE WAS CALLED")
            print("Time since last read: \t{} seconds".format(time_since_last))
            print("Total execution time: \t{} seconds".format(exec_time))
            self.last_read = time.time()
            '''

            if self.connection.in_waiting:
                # print("BUFFER NOT EMPTY")
                return self.connection.readline()
            else:
                return ''

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
