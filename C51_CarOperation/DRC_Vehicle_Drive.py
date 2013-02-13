#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
from LogData import LOG
from datetime import datetime
import numpy as np
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import  Float64
from math import *
from Point2Point import P2P
from geometry_msgs.msg import Pose
import sys
#from C31_PathPlanner.srv import C31_GetPath
from tf.transformations import euler_from_quaternion
import numpy
import matplotlib.pyplot as plt
from plotGraph import listUpt
from plotGraph import plotGraph
from math import *
import os

class node:
    world= Odometry()
    pub=0
    sub=0
    path=0
    
    def __init__(self,myVec):
       self.path=myVec
       #print self.path
    
    def callback(self, data):
        self.world=data.pose
    
    def DistanceToWP(self, object):
        return float((object[0]-self.world.pose.position.x)**2+(object[1]-self.world.pose.position.y)**2)**0.5 #Distance^2 = (Xd-Xc)^2+(Yd-Yc)^2
    def OrientationErrorToWP(self, object):
        OrientationOfCar=euler_from_quaternion( numpy.array((self.world.pose.orientation.x, self.world.pose.orientation.y, self.world.pose.orientation.z, self.world.pose.orientation.w), dtype=numpy.float64))
        OrientationofObject =atan2(object[1]-self.world.pose.position.y, object[0]-self.world.pose.position.x)
        return 180/pi*(OrientationofObject-OrientationOfCar[2]) #Desired_Orientation = Orientation of the object - Orientation of the car
    def talker(self):
        '''Library ----------------------------------
        for driving write the desired speed.. max:0.09
        --------------------->driveC.gas( desired speed)
        
        for turning max: 6*pi
        ---------------------->Steer.turn(desired turn)
        ''' 
        self.sub = rospy.Subscriber('/ground_truth_odom', Odometry,self.callback )
        hb=handB()         #handbrake online
        hb.releaseHB()     #release handbrake
        driveC=Drive() #gas pedal online
        Steer=SW()      #steering wheel online
        #----------------hand brake released --------------------------#
        while not rospy.is_shutdown():
            DATA=LOG()
            i=0
            for object in self.path:
                DATA.WayPoint(object)
                flag=b=m=0
                if(object[1]-self.world.pose.position.y==0): #define the normal to the way points for: "isPassedNormal" function
                    b=object[0]
                    if object[0]>self.world.pose.position.x:
                        flag=1
                    else:
                        flag=2
                else:
                    m=-1* float((object[0]-self.world.pose.position.x)/(object[1]-self.world.pose.position.y))
                    b= float(object[1])-m*float(object[0])
                    if object[1]>self.world.pose.position.y:
                        flag=3
                    else:
                        flag=4
                al=atan2(object[1]-self.world.pose.position.y,  object[0]-self.world.pose.position.x)*180/pi
                try:
                    m1=atan2(self.path[i+1][1]-object[1], self.path[i+1][0]-object[0])*180/pi
                except: 
                    #print "last"
                    m1=360
                #print "alpha is: %f,next alpha is: %f distance is: %f"%(al, m1, self.DistanceToWP(object))
                if (abs(al-m1)>20 and self.DistanceToWP(object)>5) :
                    print "-------------------------------"
                    print "Driving to way point x=%f, y=%f" %(object[0],object[1])
                    while (isPassedNormal(self.world.pose.position.x, self.world.pose.position.y, m, b,flag)):
                        DATA.MyPath(self.world.pose.position.x, self.world.pose.position.y)
                        [speed, Cspeed]=P2P(self.DistanceToWP(object), self.OrientationErrorToWP(object)) 
                        driveC.gas(speed)
                        Steer.turn(Cspeed)
                    DATA.DistanceError([self.world.pose.position.x-object[0], self.world.pose.position.y-object[1]])
                    DATA.PassedWayPoint(object)
                    print "arrived at Way point"
                i+=1
            #self.path=getPath() #Note - the module is still not ready to be fully operable because it always considers your location as (0,0) and does not update your location.
            #print self.path
            self.path=[]
            if not self.path:
                driveC.gas(0)
                Steer.turn(0)
                break
        #------------------Pull hand brake - off -------------------------#
        hb.pullHB()   
        #plotGraph(DATA)       

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


class Drive:
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



def isPassedNormal(currx, curry, m, b, flag):
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

if __name__ == '__main__':
    try:
        rospy.init_node('talker')
        #array=getPath()
        #array=[(10, 19), (0, 0)]
        array=[(26, -16),  (-20, -20), (-10, -10), (0, 0)]
        #array=[(6, -6)  , (0, 0)]

        d=node(array)
        d.talker()   
        #print array
    except rospy.ROSInterruptException: pass


