#!/usr/bin/env python
import roslib; roslib.load_manifest('C51_CarOperation')
import rospy
import actionlib
import C51_CarOperation.msg
from C51_CarOperation.msg import Monitoring
from geometry_msgs.msg import Twist
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from std_msgs.msg import  Float64, Int32
from Point2Point import P2P
import numpy
from geometry_msgs.msg import Pose, Polygon, Twist
from plotGraph import plotGraph
from math import *
from LogData import LOG
from Point2Point import P2P
from tf.transformations import euler_from_quaternion
import time
from datetime import datetime, timedelta
from C25_GlobalPosition.msg import C25C0_ROP
from C25_GlobalPosition.srv import C25BDI
from C42_State.msg import MotionType
from C31_PathPlanner.msg import C31_Waypoints
from C31_PathPlanner.srv import C31_GetPath
import Tkinter
from GUI import App
from ObsticalDetection import plotG
import Queue
import threading
from WheelClient import Wheel_client
from CalibrateWheel import WheelCalibrate_client, runCalibrate
GUI = 0
LEFT = -1
RIGHT = 1
TIME = 0.4

class RTD():
    def __init__(self):
        self.object = [0, 0]
        self.location = [0, 0]
        self.speed = 0.0
        self.turn = 0.0
        self.distance = 0
        self.OriEr = 0
        self.AllWP=[]
        self.ActSpeed = 0.0
        self.dO=0
        self.ObsticalPoints=[]
class INPUTrtd:
    def __init__(self):
        self.MAP=0
        self.GOAL=[-1, -1]
        self.WP = []

def TurnSteering(wheel_Q):
        Init_Angle=1.57
        Wheel_before=10
        while 1:
            try:
                host=wheel_Q.get(timeout=0.1)
            except:
                host=0
            print host
            if abs(host)>1.5:
                Wheel_client(Init_Angle, 1.57+host*0.5)
            else:
                Wheel_client(Init_Angle, 1.57+host*0.25)
            Init_Angle=999
            
def task(root,app,q_in,  q_out):
    host=RTD()
    try:
        host = q_out.get(timeout=0.01)
        q_out.task_done()
        app.updateWP(host.AllWP)
    except Queue.Empty:
                # do whatever background check needs doing
                pass
    #print host.location
    INP=INPUTrtd()

    q_in.put(INP)
    #app.distancePlot(host.ObsticalPoints)
    app.showMAP(host.ObsticalPoints)
    app.showMAP2(host.ObsticalPoints)
    try:
        pass
        #app.canvas.draw()
    except:
        pass    
    if host.location:
        app.updateLocation([int(host.location[0]), int(host.location[1])])
        app.updateActSpeed(int(host.ActSpeed*100)/100.0)
        app.updateObject([int(host.object[0]), int(host.object[1])])
        app.updateSpeed(host.speed)
        app.updateCircularSp(host.turn)
        app.updateDistance(host.distance)
        app.updateOrientation(host.OriEr)
        app.updatedO(host.dO)
    
    else:
        pass
    if not app.exitVal:
        root.after(20,task, root, app,q_in,  q_out)
    else:
        root.quit
    

def doGui(q_in, q_out):
    root = Tkinter.Tk()
    
    app = App(root)
    
    root.after(20,task, root, app,q_in,  q_out)
    
    root.mainloop()


class Drive(object):
    _feedback = C51_CarOperation.msg.DriveFeedback()
    _result   = C51_CarOperation.msg.DriveResult()
    world= Odometry()
    pub=0
    sub=0
    path=0
    def __init__(self, name, pathVec):  
        self.rtd=RTD()

        self.xPosition = 0
        self.yPosition = 0
        self.zOrientation = 0
        self.ActSpeed=0
        self.OE = [0, 0]
        self.dOE = 0
        self.Htime = 0
        if GUI:
            self.q_in = Queue.Queue()
            self.q_out = Queue.Queue()
            t = threading.Thread(target=doGui, args =  (self.q_in, self.q_out, ) )
            t.daemon = True
            t.start()
            a=RTD()
            self.q_out.put(a)        
        self.Obsticalpoints=[]
        self.ObsticalTime=[]
        sub = rospy.Subscriber('/my_cloud', Polygon,self.Obs_callback)    
        #sub = rospy.Subscriber('/RoadExtract', Polygon,self.Obs_callback)    
        
        self.sub = rospy.Subscriber('/C25/publish', C25C0_ROP,self.MyLocation_callback) #get atlas location by subscribing to C25 module       
        #self.sub = rospy.Subscriber('/ground_truth_odom', Odometry,self.MyLocation_callback2) #get atlas location by subscribing to C25 module   
        self.MotionTypeMSG = MotionType()
        self.MotionTypeMSG.motion =5        
        self.motionTypePublisher = rospy.Publisher("/motion_state/motion_type", MotionType)        

        

        self._action_name = name
        self.path=pathVec
        self._as = actionlib.SimpleActionServer(self._action_name, C51_CarOperation.msg.DriveAction, execute_cb=self.DriveCallback, auto_start=False)
        self.feedback_publisher = rospy.Publisher("C51/m_feedback", Monitoring)
        self._as.start()
    def Obs_callback(self, data):
        a=datetime.now()
        F=-0.5
        H=1.5
        #H=self.floorFilter(data)
        #print H
        for pt in data.points:
            if pt.z>F  and pt.z<H:
                self.Obsticalpoints.append((pt.x, -pt.y, -pt.z))
                self.ObsticalTime.append(a)
        
        self.filter()
    def floorFilter(self, data):
        floorH=10
        epsilon=0.1
        for pt in data.points:
            if pt.z<floorH:
                floorH = pt.z
        return floorH+epsilon
    def filter(self):
        n = datetime.now()
        try:
            while (n-self.ObsticalTime[0]> timedelta(seconds=TIME)):
                self.Obsticalpoints.pop(0)
                self.ObsticalTime.pop(0)
            
        except:
            pass
        if GUI:
            self.rtd.ObsticalPoints=self.Obsticalpoints
            self.q_out.put(self.rtd)
#    def MAPPING(self):
#        while not rospy.is_shutdown():
#            rospy.sleep(0.5)
#            f =self.factorGenerator()
#            print "VL", f[3],"L", f[0],"Z", f[1],"R",f[2], "VR", f[4]
#            # [L  ST  R HL  HR]
##            [collisionPt, side]=self.willCollide()
##            if collisionPt:
##                if side==LEFT:
##                    print "will colide in %f meters. can pass on the LEFT side" %collisionPt[0]
##                elif side==RIGHT:
##                    print "will colide in %f meters. can pass on the RIGHT side" %collisionPt[0]
##                else:
##                    print "will colide in %f meters. can't pass!!!!" %collisionPt[0]
            


    def exit_Drive(self, DATA, num):
        DATA.FinishDrive = DATA.getTime()
        DATA.print2file()
        print "Finished generating log.txt"
        
        if (not (DATA.MyPathLst==[])):
            plotGraph(DATA)    
        
        self._result.success = num
        self._result.plan = "finished driving car"
        self._result.description = "finished driving car"
    
    def MyLocation_callback(self, data):
        self.Htime=(10**-9)*data.pose.header.stamp.nsecs+data.pose.header.stamp.secs
        self.xPosition =data.pose.pose.pose.position.x
        self.yPosition =data.pose.pose.pose.position.y
        Orientation=euler_from_quaternion( numpy.array((data.pose.pose.pose.orientation.x, data.pose.pose.pose.orientation.y, data.pose.pose.pose.orientation.z, data.pose.pose.pose.orientation.w), dtype=numpy.float64))
        self.zOrientation = Orientation[2]
        self.ActSpeed = (data.pose.twist.twist.linear.x**2+data.pose.twist.twist.linear.y**2)**0.5
        if GUI:
            self.rtd.ActSpeed=self.ActSpeed
            self.rtd.location = [self.xPosition, self.yPosition]
            self.q_out.put(self.rtd)
#    def MyLocation_callback2(self, data):
#        self.Htime=(10**-9)*data.header.stamp.nsecs+data.header.stamp.secs
#        self.xPosition =data.pose.pose.position.x
#        self.yPosition =data.pose.pose.position.y
#        Orientation=euler_from_quaternion( numpy.array((data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w), dtype=numpy.float64))
#        self.zOrientation = Orientation[2]
#        self.ActSpeed = (data.twist.twist.linear.x**2+data.twist.twist.linear.y**2)**0.5
#        if GUI:
#            self.rtd.ActSpeed=self.ActSpeed
#            self.rtd.location = [self.xPosition, self.yPosition]
#            self.q_out.put(self.rtd)
    def DistanceToWP(self, object):
        return float((object[0]-self.xPosition)**2+(object[1]-self.yPosition)**2)**0.5 #Distance^2 = (Xd-Xc)^2+(Yd-Yc)^2
    
    def OrientationErrorToWP(self, object):
        OrientationofObject =atan2(object[1]-self.yPosition, object[0]-self.xPosition)
        self.ChangeInOrientation(180/pi*(OrientationofObject-self.zOrientation)) #Desired_Orientation = Orientation of the object - Orientation of the car
    def ChangeInOrientation(self, theta):
        try:
            self.dOE = (theta - self.OE[0])/(self.Htime - self.OE[1])
        except:
            pass
        self.OE = [theta, self.Htime]
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
    def NormalProperties(self, object):
        flag=b=m=0
        if(object[1]-self.yPosition==0): #define the normal to the way points for: "isPassedNormal" function
            b=object[0]
            if object[0]>self.xPosition:
                flag=1
            else:
                flag=2
        else:
            m=-1* float((object[0]-self.xPosition)/(object[1]-self.yPosition))
            b= float(object[1])-m*float(object[0])
            if object[1]>self.yPosition:
                flag=3
            else:
                flag=4
        
        return ([flag, b, m])
    def DriveCallback(self, goal):
        self.motionTypePublisher.publish(self.MotionTypeMSG)
        num = Int32(0)
#        rospy.wait_for_service('/C25/BDIswitch')
#        try:
#            State= rospy.ServiceProxy('/C25/BDIswitch', C25BDI)
#            resp = State(num)
#        except rospy.ServiceException, e:
#            print "Service /C25/BDISwitch call failed: %s"%e
#            log.info("Might be driving without Global Position")
        gasP=Gas() #gas pedal online
        #brakeP=Brake() #gas pedal online
        #Steer=SW()      #steering wheel online        
        self._result.success = 1
             
        #rospy.sleep(5.0)
        
        wheel_dat=0
        wheel_Q = Queue.Queue()
        wheel_Q.put(wheel_dat)
        wheel_t = threading.Thread(target=TurnSteering, args =  (wheel_Q,  ) )
        wheel_t.daemon = True
        wheel_t.start()
        
        self.C31_path = numpy.array([])
        self.sub = rospy.Subscriber('/path', C31_Waypoints,self.getPath) #get atlas location by subscribing to C25 module        
        self.path= self.C31_path#(5, -1.5), (18, -1.5), [(51,0), (97,0), (102, 5),(113, 13.5),  (146.5, 46.35), (148, 56.45), (148, 92), (160.45, 102), (198, 102) ]#
        DATA=LOG()
        DATA.StartDrive = DATA.getTime()
        
        while not rospy.is_shutdown():
            i=0
            #brakeP.brake(0)
            if not self.path:
                flag=0
                while (not self.path and flag!=10):
                    rospy.sleep(0.02)
                    self.path=self.C31_path
                    flag+=1
            if self.path:
                if GUI:
                    self.rtd.AllWP = [[(int(k*100))/100.0, float(int(j*100))/100] for k, j in self.path]
                    self.q_out.put(self.rtd)
                for object in self.path:
                    if self._as.is_preempt_requested():
                        success = False
                        rospy.loginfo('%s: Preempted' % self._action_name)
                        self._as.set_preempted()
                        #gasP.gas(0)
                        #brakeP.brake(1)
                        self.exit_Drive(DATA, 1)
                        return
                    DATA.WayPoint(object)
                    [flag, b, m] = self.NormalProperties(object) #create the normal Properties needed, i.e. flag, slope and b for y=m*x+b
                    al=atan2(object[1]-self.yPosition,  object[0]-self.xPosition)*180/pi
                    try:
                        m1=atan2(self.path[i+1][1]-object[1], self.path[i+1][0]-object[0])*180/pi
                    except: 
                        m1=360
                    if (self.DistanceToWP(object)>5) : #abs(al-m1)>20 and 
                        print "-------------------------------"
                        print "Driving to way point x=%f, y=%f" %(object[0],object[1])
                        while (self.isPassedNormal(self.xPosition,self.yPosition, m, b,flag)):
                            
                            if self._as.is_preempt_requested():
                                rospy.loginfo('%s: Preempted' % self._action_name)
                                self._as.set_preempted()
                                gasP.gas(0)
                                #brakeP.brake(1)
                                self.exit_Drive(DATA, 1)
                                return
                            DATA.MyPath(self.xPosition, self.yPosition)
                            self.OrientationErrorToWP(object)
                            

                            factor = self.factorGenerator()
                            
#                            for f in range(0, len(factor)):
#                                if not factor[f]==1:
#                                    factor[f]=0
                            #[dSpeed, Cspeed]=P2P(self.DistanceToWP(object), self.OE[0], self.dOE, [1, 1, 1, 1, 1])
#                            print "without:", Cspeed                                    
                            [dSpeed, Cspeed]=P2P(self.DistanceToWP(object), self.OE[0], self.dOE, factor)
                            if factor==[0, 0, 0, 0, 0]:
                                dSpeed=0
#                            print "with:", Cspeed
                            
                            #rospy.sleep(0.05)
                            #dSpeed = 0.5
                            #[  acc, brk] = self.SpeedController(dSpeed/2.5)
                            Cspeed = Cspeed*pi
                            #print brk, acc
                            #rospy.sleep(0.5)
                            
                            gasP.gas(acc)
                            #brakeP.brake(brk)
                            while not wheel_Q.empty():
                                a = wheel_Q.get()
                            #print "sending"
                            wheel_Q.put(Cspeed)
                            
                            #Steer.turn(-Cspeed)
                            if GUI:
                                self.rtd.object=object
                                self.rtd.location = [self.xPosition,self.yPosition]
                                self.rtd.speed = dSpeed
                                self.rtd.turn = Cspeed
                                self.rtd.distance = self.DistanceToWP(object)
                                self.rtd.OriEr = self.OE[0]
                                self.rtd.dO = self.dOE
                                self.rtd.ActSpeed = self.ActSpeed
                                self.q_out.put(self.rtd) #data to GUI!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        DATA.DistanceError(sqrt((self.xPosition-object[0])*(self.xPosition-object[0])+( self.yPosition-object[1])*( self.yPosition-object[1])))
                        DATA.PassedWayPoint(object)
                        newMsg=Monitoring()
                       
                        newMsg.LastWPpassed = object
                        newMsg.MyLoc4LastWP= [self.xPosition , self.yPosition]
                        self.feedback_publisher.publish(newMsg)
                        print newMsg
                        self._feedback.complete = 32.22
                        self._as.publish_feedback(self._feedback)
                        print "arrived at Way point"
                        print "-------------------------------"
                    i+=1
                    
            self.path=self.C31_path #[]#
#            try:
#                if (sqrt((self.path[-1][0]-self.xPosition)*(self.path[-1][0]-self.xPosition) +(self.path[-1][1]-self.yPosition)*(self.path[-1][1]-self.yPosition))<0.5):
#                    print "get out!!!!!!!!!!!!!!!!!!!!!!", self.path[-1], self.xPosition, self.yPosition
#                    gasP.gas(0)
#                    brakeP.brake(1)
#                    self.exit_Drive(DATA, 0)
#                    break   
#            except:
#                print "path array is empty", self.path
#                gasP.gas(0)
#                brakeP.brake(1)
#                self.exit_Drive(DATA, 0)
#                break   
            
        gasP.gas(0)
        rospy.loginfo('Finished Driving!! Please run "FinishDrive" client, in order to pull hand brake.')
        self._as.set_succeeded(self._result)
    def SpeedController(self, desiredSpd):
        error=desiredSpd-self.ActSpeed
        kp = 1
        if error>0:
            return [error*kp, 0]
        else:
            return [0, -error*kp/10]
#    def SteeringController(self, desiredTurn):
#        error=desiredTurn-self.ActSpeed
#        kp = 1
#        if error>0:
#            return [error*kp, 0]
#        else:
#            return [0, -error*kp/10]            
    def getPath(self, data):
#        try:
            self.C31_path = []
            for point in data.points:
                self.C31_path.append([point.x, point.y])
            if not self.C31_path:
                rospy.loginfo('Can not access C31_Path topic!')
    def willCollide(self):
        DB = self.Obsticalpoints
        HazardPoints=[]
        minPointR=[100, 100]
        minPointL=[100, -100]
        DIR = 0
        Xmin = -5
        Xmax = 5
        
        left=1
        YminL = -2.5
        YmaxL = -0.5
        
        right=1
        YminR = 0.5
        YmaxR = 2.5
        
        for pot in DB:
            if pot[0]<11 and pot[1]>-0.2 and pot[1]<1:
                if (minPointL[0]>pot[0]+0.5):
                    minPointL = pot
                elif minPointL[1]>pot[1]:
                    minPointL = pot
                elif minPointL[0]>pot[0]:
                    minPointL = pot
                if (minPointR[0]>pot[0]+0.5):
                    minPointR = pot
                elif minPointR[1]<pot[1]:
                    minPointR = pot
                elif minPointR[0]>pot[0]:
                    minPointR = pot
        if minPointR[0]==100 and minPointL[0]==100:
            return [[], 0]
        for pt in DB:#check left
            if (pt[0]>minPointL[0]+Xmin) and (pt[0]<minPointL[0]+Xmax) and (pt[1]>minPointL[1]+YminL) and (pt[1]<minPointL[1]+YmaxL):
                left=0
            if (pt[0]>minPointR[0]+Xmin) and (pt[0]<minPointR[0]+Xmax) and (pt[1]>minPointR[1]+YminR) and (pt[1]<minPointR[1]+YmaxR):
                right=0
        if left:
            return [minPointL, LEFT]
        if right:
            return [minPointR, RIGHT]
        return [minPointL, 0]
    def maneuver(self, DIR, pt):
        if DIR==LEFT:
            self.path.append(self.xPosition+pt[0]-3)
            self.path.append(self.yPosition+pt[1]-3)
    def factorGenerator(self):
        VL=L=Z=R=VR=1
        DB = self.Obsticalpoints

        for pt in DB:
            if pt[1]>-1.5 and pt[1]<2: #create Straight forward factor
                #print pt
                if pt[0]<3:
                    rospy.loginfo("Will surely collide!!!")
                    rospy.sleep(0.05)
                    return [0, 0, 0, 0, 0]
                z=0.051*exp(0.208*pt[0])
                if z<Z:#choose lowest
                    Z=z
            if pt[1]>1 and pt[1]<3.5:#create right factor
                if pt[0]>0.5:
                    r=0.051*exp(0.208*pt[0])
                    if r<R:
                        R=r
#            if  pt[1]>3 and pt[1]<6:#create very right factor
#                if pt[0]>2:
#                    vr=1/(15.0-2)*(pt[0]-2)
#                    if vr<VR:
#                        VR=vr
            if pt[1]>-2.5 and pt[1]<-0.5:#create left factor
                if pt[0]>0.5:
                    l=0.051*exp(0.208*pt[0])
                    if l<L:
                        L=l
#            if  pt[1]>-4.5 and pt[1]<-2:#create very right factor
#                if pt[0]>2:
#                    vl=1/(15.0-2)*(pt[0]-2)
#                    if vl<VL:
#                        VL=vl                    
        return [L, Z, R, VL, VR]# [L  ST  R HL  HR]
#def getPath():
#    #rospy.wait_for_service('C31_GlobalPathPlanner/getPath')
#    try:
#        rospy.loginfo ("Connecting to C31_GlobalPathPlanner/getPath")
#        add_two_ints = rospy.ServiceProxy('C31_GlobalPathPlanner/getPath', C31_GetPath)
#        array = add_two_ints().path.points
#        array2=[(po.x, po.y) for po in array]
#        if array2==[]:
#            rospy.loginfo ("No way points!!! check perception module")
#        else:
#            rospy.loginfo ( "Way Points collected")
#        return array2
#    except rospy.ServiceException, e:
#        rospy.loginfo ( "Check that C31 is running properly." )
#        rospy.loginfo ( "try running in terminal: rosservice call /C31_GlobalPathPlanner/getPath ")



#=========================================================================================#
#----------DRC vehicle controllers.....note! - will be erased once the C67 module is complete----------------#
#=========================================================================================#

car='drc_vehicle'#'golf_cart'#


class Gas:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('PedalsManipulation/move_right_pedal', Float64)
#        self.pub = rospy.Publisher(car+'/gas_pedal/cmd', Float64)
#        self.sub = rospy.Subscriber('/'+car+'/gas_pedal/state', Float64, self.gasCallback)
    def gasCallback(self, data):
        self.status=data.data

    def gas(self, num):
        s = Float64(num*10)
        self.pub.publish(s)

class Brake:
    status=-11
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher('PedalsManipulation/move_right_pedal', Float64)        
        #self.pub = rospy.Publisher(car+'/brake_pedal/cmd', Float64)
        #self.sub = rospy.Subscriber('/'+car+'/brake_pedal/state', Float64, self.brakeCallback)
    def brakeCallback(self, data):
        self.status=data.data

    def brake(self, num):
        s = Float64(num*100)
        self.pub.publish(s)

class SW:
    status=0
    pub=0
    sub=0
    def __init__(self):
        self.pub = rospy.Publisher(car+'/hand_wheel/cmd', Float64)
        self.sub = rospy.Subscriber('/'+car+'/hand_wheel/state', Float64, self.SWCallback)
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
    
