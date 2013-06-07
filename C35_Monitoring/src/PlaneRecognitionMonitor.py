#! /usr/bin/env python

import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import  Float32
import tf
from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from C25_GlobalPosition.msg import C25C0_ROP
import math
	
SAMPLE = 500
	
class PlaneRecognitionMonitor(RobilTask):
    counter = 1
    pitchAvg = 0
    statHash = {}
    
    def callback(self, msg):
        if self.statHash != {}:
            roll, pitch, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            if (self.counter%SAMPLE != 0):
                print "Plane Recognition monitor got first message."            
                self.pitchAvg = self.pitchAvg + pitch            
            else:  
                self.pitchAvg = self.pitchAvg/SAMPLE
                slopeProb = self.statHash[self.pitchAvg]                
                self.resultPublisher.publish(float(slopeProb[0])/float(slopeProp[1]))
                self.pitchAvg = pitch
            self.counter = self.counter + 1      

    def __init__(self, name):
         print "Initializing Plane Recognition Monitor Node"
         #listen to /atlas/imu/pose/pose/orientation
         self.subscribers  = rospy.Subscriber('/atlas/imu', Imu, self._get_imu)
         self.resultPublisher = rospy.Publisher("C35/PlaneRecognitionMonitor",Float32)
         f= open('statistics', 'r')
         for line in f:
             parsedLine = line.split(' ') 
             self.statHash[parsedLine[0]] = [parsedLine[1], parsedLine[2]]
         print  self.statHash   
         RobilTask.__init__(self, name)
         
    def task(self, name, uid, parameters):
         print "Start Plane Recognition Monitor for the robot." 
		#print parameters
         d = rospy.Duration(1,0)
         
         print "MONITORING Plane Recognition:"
         while not rospy.is_shutdown():
            rospy.sleep(d)
         if self.isPreepted():
                print "Preempt monitoring task"
                return RTResult_PREEPTED()
         return RTResult_SUCCESSED("Finished in Success")
          
if __name__ == '__main__':
	print "Start PlaneRecognitionMonitor Node"
	rospy.init_node('PlaneRecognitionMonitor')
	PlaneRecognitionMonitor("PlaneRecognitionMonitor")
	rospy.spin()
	print "Plane Recognition Monitor Node Closed" 
