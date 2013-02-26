#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import C51_CarOperation.msg
from C51_CarOperation.msg import Monitoring
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import  Float64
from Point2Point import P2P
import numpy
from geometry_msgs.msg import Pose
from plotGraph import plotGraph
from math import *
from LogData import LOG
from Point2Point import P2P
from tf.transformations import euler_from_quaternion
#from C31_PathPlanner.srv import C31_GetPath

class Drive(object):
    _feedback = C51_CarOperation.msg.DriveFeedback()
    _result   = C51_CarOperation.msg.DriveResult()
    world= Odometry()
    pub=0
    sub=0
    path=0
    def __init__(self, name, pathVec):    
        self._action_name = name
        self.path=pathVec
        self._as = actionlib.SimpleActionServer(self._action_name, C51_CarOperation.msg.DriveAction, execute_cb=self.DriveCallback, auto_start=False)
        self.feedback_publisher = rospy.Publisher("C51/m_feedback", Monitoring)
        self._as.start()
        

    def callback(self, data):
        self.world =data
        #print self.world
    def DistanceToWP(self, object):
        return float((object[0]-self.world.pose.pose.position.x)**2+(object[1]-self.world.pose.pose.position.y)**2)**0.5 #Distance^2 = (Xd-Xc)^2+(Yd-Yc)^2
    
    def OrientationErrorToWP(self, object):
        OrientationOfCar=euler_from_quaternion( numpy.array((self.world.pose.pose.orientation.x, self.world.pose.pose.orientation.y, self.world.pose.pose.orientation.z, self.world.pose.pose.orientation.w), dtype=numpy.float64))
        OrientationofObject =atan2(object[1]-self.world.pose.pose.position.y, object[0]-self.world.pose.pose.position.x)
        return 180/pi*(OrientationofObject-OrientationOfCar[2]) #Desired_Orientation = Orientation of the object - Orientation of the car
    def isPassedNormal(self, currx, curry, m, b, flag):
        #print (currx, curry, m, b, flag)
        if flag==1:
            if(b>currx):
                return True
            return False
        if flag==2:
            if (b<currx):
                return True
            return False        
        if flag==3:
            if curry<(currx*m+b):
                return True
            return False
        if flag==4:
            if curry>(currx*m+b):
                return True
            return False    
    def DriveCallback(self, goal):
        #self.path=getPath()
        self.path=[(26, -16),  (-20, -20), (-10, -10), (0, 0)]
        #self._feedback.WayPointsGiven = []
        #DRC Vehicles controllers online - should be replaced with C67 module
        #hb=handB()         #handbrake online
        gasP=Gas() #gas pedal online
        brakeP=Brake() #gas pedal online
        Steer=SW()      #steering wheel online        
        # helper variables
        success = True
        #Perform job
        if success:
            
            '''Library ----------------------------------
            for driving write the desired speed.. max:0.09
            --------------------->self.gasP.gas( desired speed)
            
            for turning max: 6*pi
            ---------------------->self.Steer.turn(desired turn)
            ''' 
            self._result.success = 1
            success = True
            self.sub = rospy.Subscriber('/ground_truth_odom', Odometry,self.callback ) #get atlas location
            while not rospy.is_shutdown():
                DATA=LOG()
                i=0
                brakeP.brake(0)
                for object in self.path:
                    if self._as.is_preempt_requested():
                        success = False
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        gasP.gas(0)
                        return
                    #self._feedback.WayPointsGiven.append(object)
                    DATA.WayPoint(object)
                    flag=b=m=0
                    if(object[1]-self.world.pose.pose.position.y==0): #define the normal to the way points for: "isPassedNormal" function
                        b=object[0]
                        if object[0]>self.world.pose.pose.position.x:
                            flag=1
                        else:
                            flag=2
                    else:
                        m=-1* float((object[0]-self.world.pose.pose.position.x)/(object[1]-self.world.pose.pose.position.y))
                        b= float(object[1])-m*float(object[0])
                        if object[1]>self.world.pose.pose.position.y:
                            flag=3
                        else:
                            flag=4
                    al=atan2(object[1]-self.world.pose.pose.position.y,  object[0]-self.world.pose.pose.position.x)*180/pi
                    try:
                        m1=atan2(self.path[i+1][1]-object[1], self.path[i+1][0]-object[0])*180/pi
                    except: 
                        m1=360
                    if (abs(al-m1)>20 and self.DistanceToWP(object)>5) :
                        print "-------------------------------"
                        print "Driving to way point x=%f, y=%f" %(object[0],object[1])
                        while (self.isPassedNormal(self.world.pose.pose.position.x,self.world.pose.pose.position.y, m, b,flag)):
                            if self._as.is_preempt_requested():
                                success = False
                                rospy.loginfo('%s: Preempted' % self._action_name)
                                self._as.set_preempted()
                                gasP.gas(0)
                                return
                            DATA.MyPath(self.world.pose.pose.position.x, self.world.pose.pose.position.y)
                            [speed, Cspeed]=P2P(self.DistanceToWP(object), self.OrientationErrorToWP(object)) 
                            gasP.gas(speed)
                            Steer.turn(Cspeed)
                        DATA.DistanceError([self.world.pose.pose.position.x-object[0], self.world.pose.pose.position.y-object[1]])
                        DATA.PassedWayPoint(object)
                        newMsg=Monitoring()
                        newMsg.LastWPpassed = object
                        newMsg.MyLoc4LastWP= [self.world.pose.pose.position.x , self.world.pose.pose.position.y]
                        self.feedback_publisher.publish(newMsg)
                        print newMsg,  "shmulik"
                        #self._feedback.LastWPpassed_Y = object[1]
                        #self._feedback.MyLoc4LastWP= [self.world.pose.pose.position.x , self.world.pose.pose.position.y]
                        #self._feedback.MyLoc4LastWP_Y=  self.world.pose.pose.position.y
                        #print self._feedback
                        self._feedback.complete = 32.22
                        self._as.publish_feedback(self._feedback)
                        print "arrived at Way point"
                    i+=1
                #self.path=getPath() #Note - the module is still not ready to be fully operable because it always considers your location as (0,0) and does not update your location.
                #print self.path
                self.path=[]
                if not self.path:
                    gasP.gas(0)
                    brakeP.brake(0)
                    Steer.turn(0)
                    self._result.success = 0
                    self._result.description = "finished driving car"
                    self._result.plan = "finished driving car"
                    break   
            
        #plotGraph(DATA)    
        if success:
            rospy.loginfo('Finished Driving!! Please run "FinishDrive" client, in order to pull hand brake.')
            self._as.set_succeeded(self._result)
        #----------------hand brake released --------------------------#

def getPath():
    #rospy.wait_for_service('C31_GlobalPathPlanner/getPath')
    try:
        print "Connecting to C31_GlobalPathPlanner/getPath"
        add_two_ints = rospy.ServiceProxy('C31_GlobalPathPlanner/getPath', C31_GetPath)
        array = add_two_ints().path.points
        array2=[(po.x, po.y) for po in array]
        if array2==[]:
            print "No way points!!! check preception module"
        else:
            print "Way Points collected"
        return array2
    except rospy.ServiceException, e:
       print "Check that C31 is running properly." 
       print "try running in terminal: rosservice call /C31_GlobalPathPlanner/getPath "
#=========================================================================================#
#----------DRC vehicle controllers.....note! - will be erased once the C67 module is complete----------------#
#=========================================================================================#
class handB:

    handbrake=1
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/hand_brake/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/hand_brake/state', Float64, self.handbrakeCallback)
    def handbrakeCallback(self, data):
        self.handbrake=data.data

    def releaseHB(self):
        while self.handbrake>0.5 :
            self.pub.publish(0.0)

    def pullHB(self):
        self.pub.publish(1.0)
        #while self.handbrake<0.5 :


class Gas:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/gas_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/gas_pedal/state', Float64, self.gasCallback)
    def gasCallback(self, data):
        self.status=data.data

    def gas(self, num):
        self.pub.publish(num)

class Brake:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/brake_pedal/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/brake_pedal/state', Float64, self.brakeCallback)
    def brakeCallback(self, data):
        self.status=data.data

    def brake(self, num):
        self.pub.publish(num)

class SW:
    status=0
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('drc_vehicle/hand_wheel/cmd', Float64)
        self.sub = rospy.Subscriber('/drc_vehicle/hand_wheel/state', Float64, self.SWCallback)
    def SWCallback(self, data):
        self.status=data.data

    def turn(self, num):
        self.pub.publish(num)
#==============================================================================================#
#==============================================================================================#

if __name__ == '__main__':
    try:
        rospy.init_node('WayPointsDriving')
        Drive(rospy.get_name(), [])   
        rospy.spin()
    except rospy.ROSInterruptException: 
        print "Did not reach goal"
    
