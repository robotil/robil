#!/usr/bin/env python
import roslib; roslib.load_manifest('C25_AtlasFell')
import rospy
import actionlib
import C25_AtlasFell.msg
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy
from math import pi
from std_msgs.msg import Bool
class Fell(object):
    
    def __init__(self, name):    
        self.RPY = []
        self.LA= []
        self.IMU = Imu()
        self.pub = rospy.Publisher('C35/falling', Bool)
        self.sub = rospy.Subscriber('/atlas/imu', Imu,self.IMU_callback)
        self._feedback = C25_AtlasFell.msg.FallenFeedback()
        self._result   = C25_AtlasFell.msg.FallenResult()
        rospy.loginfo('C25_AtlasFell service Online!')
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, C25_AtlasFell.msg.FallenAction, execute_cb=self.FellCallback, auto_start=False)
        self._as.start()
    
    def IMU_callback(self, data):
        self.IMU = data
        self.RPY = euler_from_quaternion( numpy.array((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w), dtype=numpy.float64))
        self.RPY = [self.RPY[0]*180/pi, self.RPY[1]*180/pi, self.RPY[2]*180/pi]
        self.LA = numpy.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
    
    def FellCallback(self, goal):
        # helper variables
        success = True
        self._feedback.complete = 50
        self._as.publish_feedback(self._feedback)
        self._result.success = 1
        self._result.description = "IMU under surveillance"
        self._result.plan = "IMU under surveillance"
        Status = False
        
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                return self.pub.publish(Status)

            if (self.RPY[1]>80 or self.RPY[1]<-80 ):
                Status = True
                #for i in range(1,1000):
                  #self.pub.publish(Status)
                #break
            else:
                Status = False
            self.pub.publish(Status)
            rospy.sleep(0.1)
            
            
            #----------end of while-----------------#
        rospy.loginfo('Exitting C25_AtlasFell module!!!')
        self._as.set_succeeded(self._result)
        #----------------hand brake released --------------------------#

if __name__ == '__main__':
    try:
        rospy.init_node('Fell')
        Fell(rospy.get_name())   
        rospy.spin()
    except rospy.ROSInterruptException: 
        rospy.loginfo('C25_AtlasFell did not initialize')


