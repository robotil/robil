#!/usr/bin/env python
import roslib; roslib.load_manifest('C48_StandUp')
import rospy
from Controller import Controller
import numpy as np

if __name__ == '__main__':
	rospy.init_node('Fall')
	print ("Robot is falling...")

	controller = Controller()
	pos = [0, 0, 0, 0,
			0, 0, -0.02, 0.04, -0.02, 0,
			0, 0, -0.02, 0.04, -0.02, 0,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0]

	orientation = np.random.uniform(-3.1, 3.1, size=1)
	if np.abs(orientation) > 1.6:
		pos[8] = -0.2 * np.cos(orientation)
		pos[8+6] = -0.2 * np.cos(orientation)
	else:
		pos[8] = -0.15 * np.cos(orientation)
		pos[8+6] = -0.15 * np.cos(orientation)
	pos[9] = 0.3 * np.sin(orientation)
	pos[9+6] = 0.3 * np.sin(orientation)
	controller.publish(pos, 0.4)
