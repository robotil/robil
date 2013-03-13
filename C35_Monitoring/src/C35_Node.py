#!/usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy

import actionlib
from MonitorTimeServer import MonitorTimeServer
from MonitorWayPointServer import MonitorWayPointServer

if __name__ == '__main__':
	rospy.init_node('C35_Monitoring')
	MonitorTimeServer("tests/simple_monitor_example.xml")
	MonitorWayPointServer("MonitorWayPoint") 
	rospy.spin()
