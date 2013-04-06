#!/usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy

from MonitorTimeServer import MonitorTimeServer
from MonitorWayPointServer import MonitorWayPointServer
from MonitorProgress import MonitorProgressServer

if __name__ == '__main__':
    rospy.init_node('C35_Monitoring')
    MonitorTimeServer("tests/simple_monitor_example.xml", "MonitorTime")
#    MonitorProgressServer("tests/simple_monitor_example.xml", "MonitorProgress")
    MonitorWayPointServer("MonitorWayPoint") 
    rospy.spin()
