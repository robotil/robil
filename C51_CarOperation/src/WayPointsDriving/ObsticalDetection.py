import roslib; roslib.load_manifest('C51_CarOperation')
from point_cloud import create_cloud_xyz32
import rospy
from point_cloud import read_points
import actionlib
import C51_CarOperation.msg
from geometry_msgs.msg import Polygon
import rospy
import numpy as np
from math import *
import numpy
import matplotlib.pyplot as plt
from C25_GlobalPosition.msg import C25C0_ROP
from tf.transformations import euler_from_quaternion
from datetime import datetime, timedelta
TIME = 3
class plotG():
    def __init__(self):
        
        self.world = C25C0_ROP()
        self.noPointsCtr = 0
        self.LastOri = 0
        self.LastLocation = C25C0_ROP()
        plt.ion()
        self.fig = plt.figure(1)
        self.ax = self.fig.add_subplot(111) 
        self.point=[]
        self.x = []#np.array([])
        self.y= []#np.array([])    
        self.Time=[]
        
        sub = rospy.Subscriber('/my_cloud', Polygon,self.Obs_callback)
        self.line1, = self.ax.plot(self.x, self.y, 'bo') # Returns a tuple of line objects, thus the comma
        self.line2,  = self.ax.plot(self.x, self.y, 'ro') # Returns a tuple of line objects, thus the comma
        self.lineAllWP, = self.ax.plot(self.x, self.y, 'g*', markersize=24) # Returns a tuple of line objects, thus the comma
        self.lineGOAL, = self.ax.plot(self.x, self.y, 'r*', markersize=24) # Returns a tuple of line objects, thus the comma
        plt.xlim([ -10, 10])
        plt.ylim([0, 30])
        plt.grid()
            
        plt.ylabel("Astern           Ahead")
        plt.xlabel("Left         Right")
    def Obs_callback(self, data):
        a=datetime.now()
        for pt in data.points:
            self.point.append((pt.x, -pt.y))
            self.Time.append(a)
    def filter(self):
        n = datetime.now()
        try:
            while (n-self.Time[0]> timedelta(seconds=TIME)):
                self.point.pop(0)
                self.Time.pop(0)
        except:
            pass
        
    def printGraph(self,PRINT,  GOAL, WP):
        if PRINT:

            x1=[]
            x2=[]
            y1=[]
            y2=[]
            for pt in self.point:
                if (pt[1]>-0.2 and pt[1]<1.8) and pt[0]<6:
                    x2.append(pt[0])
                    y2.append(pt[1])
                else:
                    x1.append(pt[0])
                    y1.append(pt[1])
            #[(x.append(i), y.append(j))  for i, j in self.point]
            wpx=[]
            wpy=[]
            [(wpx.append(i), wpy.append(j))  for i, j in WP]
            #print wpx, wpy
            if len(x1)==len(y1):
                    
                    self.line1.set_xdata(y1)
                    self.line1.set_ydata(x1)
                    self.line2.set_xdata(y2)
                    self.line2.set_ydata(x2)
                    self.lineAllWP.set_xdata(wpy)
                    self.lineAllWP.set_ydata(wpx)        
                    self.lineGOAL.set_xdata(GOAL[1])
                    self.lineGOAL.set_ydata(GOAL[0]) 
            else:
                self.point=[]
        else:
            self.fig.quit()
    def DetectCollision(self, GOAL):
        for pt in self.point:
            if pt[1]>=GOAL[1]-0.2 and pt[1]<=GOAL[1]+1.8 and pt[0]<GOAL[0]:
                #print "will collide"
                pass
        


if __name__ == '__main__':

    try:
        GOAL =[12, -1]
        WP = [(12, -1), (14, 2), (15,0 )]
        rospy.init_node('ObsitcalManuever')
        subaba = plotG()


        while not rospy.is_shutdown():
            subaba.filter()
            subaba.DetectCollision(GOAL)
            subaba.printGraph(1, GOAL, WP)
            rospy.sleep(0.01)
            subaba.fig.canvas.draw()
        #rospy.spin()
    except rospy.ROSInterruptException: 
        pass
