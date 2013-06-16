import roslib; roslib.load_manifest('WalkingObstacleDetection')
import rospy
from geometry_msgs.msg import Polygon
import numpy
from math import *
import matplotlib.pyplot as plt
from C25_GlobalPosition.msg import C25C0_ROP
from tf.transformations import euler_from_quaternion
from datetime import datetime, timedelta
TIME = 3 #How long to keep points before deleting them.. Note - the longer the lag is the longer you should keep it.

class ObstaclePoints(): #This class collects the laser from the topic
    def  __init__(self):
        self.Obsticalpoints=[] #points list
        self.ObsticalTime=[] #points time
        Obstaclesub = rospy.Subscriber('/LaserCloudForWalking', Polygon,self.Obs_callback) #collect points from the topic
    def Obs_callback(self, data):
        a=datetime.now() #what is the time?
        for pt in data.points: #append all the points collected in the lists
                self.Obsticalpoints.append([pt.x, -pt.y])
                self.ObsticalTime.append(a)
        self.filter()        #filter over due points
    def filter(self): 
        n = datetime.now() #what is the time?
        try:
            while (n-self.ObsticalTime[0]> timedelta(seconds=TIME)): #remove points that are the time deference is larger than the Parameter TIME.
                self.Obsticalpoints.pop(0)
                self.ObsticalTime.pop(0)
        except:
            pass
class plotG(): #for plotting purposes
    def __init__(self):
        plt.ion()
        self.fig = plt.figure(1)
        self.ax = self.fig.add_subplot(111) 
        x = []#np.array([])
        y= []#np.array([])    
        self.Time=[]
        self.line1, = self.ax.plot(x, y, 'bo') # Returns a tuple of line objects, thus the comma
        self.line2,  = self.ax.plot(x, y, 'ro') # Returns a tuple of line objects, thus the comma
        plt.xlim([ -10, 10])
        plt.ylim([-2, 10])
        plt.grid()
        plt.ylabel("Astern           Ahead")
        plt.xlabel("Left         Right")
        
    def printGraph(self,point):
        x1=[]
        x2=[]
        y1=[]
        y2=[]
        for pt in point:
            if sqrt(pt[0]**2+pt[1]**2)<1:
                x2.append(pt[0])
                y2.append(pt[1])
            else:
                x1.append(pt[0])
                y1.append(pt[1])
        if len(x2)<5:
            x2=[]
            y2=[]
        self.line1.set_xdata(y1)
        self.line1.set_ydata(x1)
        self.line2.set_xdata(y2)
        self.line2.set_ydata(x2)

if __name__ == '__main__':
    try:
        rospy.init_node('ObsitcalManuever')
        pointPrinter = plotG() #create class plotG
        point2 = ObstaclePoints() #create class ObstaclePoints
        while not rospy.is_shutdown():
            pointPrinter.printGraph(point2.Obsticalpoints) #print the points in Obsticalpoints
            pointPrinter.fig.canvas.draw() #renew figure
            rospy.sleep(0.2)            #sleep for 200 msec
    except rospy.ROSInterruptException: 
        pass
