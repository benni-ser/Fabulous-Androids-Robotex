#! /usr/bin/env python
import rospy
import numpy as np
from hardware.comport_mainboard import ComportMainboard
import cv2

if __name__ == '__main__':
    try:
        mainboard_runner = ComportMainboard()
        mainboard_runner.run()
	frame = np.zeros((200,200))
	while(True):
    		cv2.imshow("Frame", frame)
    		key = cv2.waitKey(1) & 0xFF
    		#print(key)
    		# if the 'q' key is pressed, stop the loop
    		if key == ord("w"):
        		mainboard_runner.setDirection("0:0:0:0")
    		if key == ord("a"):
        		mainboard_runner.write("r\n")
		if key == ord("g"):
                        mainboard_runner.getDirection()
    		if key == ord("z"):
        		shutdown()
        		##cv2.imwrite("test.png", frame)
        		break
    except rospy.ROSInterruptException:
        pass
