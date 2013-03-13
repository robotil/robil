import rospy
import numpy as np
from math import *
import numpy
import matplotlib.pyplot as plt
def listUpt(list, obj):
        list.append(obj)
        return list
def plotGraph(DATA):
        plt.figure

        #=====================#
        #++print path of vehicle++#
        #=====================#
        myPath=DATA.MyPathLst[:]
        myPathX=[]
        myPathY=[]
        myPathTime=[]

        for val in myPath:
            myPathX.append(val[0])
            myPathY.append(val[1])
            myPathTime.append(val[2])
        plt.plot(myPathX, myPathY,  label="Vehicle's Path")
        plt.plot(myPathX[0], myPathY[0],'g*',  markersize=24,   label="Start Point")
        plt.plot(myPathX[-1], myPathY[-1],'r*',  markersize=20,   label="Stop Point")        
       #========================#
        #++print Way Points given++#
       #========================#
        AWP=DATA.WayPointLst[:]
        AWPx=[]
        AWPy=[]
        AWPx.append(myPathX[0])
        AWPy.append(myPathY[1])
        
        for val in AWP:
            AWPx.append(val[0])
            AWPy.append(val[1])
        AWPX = np.array(AWPx)
        AWPY = np.array(AWPy)
        plt.quiver(AWPX[:-1], AWPY[:-1], AWPX[1:]-AWPX[:-1], AWPY[1:]-AWPY[:-1],label="WP Path", scale_units='xy', angles='xy', scale=1)                
        #=========================#
        #++print Way Points passed++#
        #=========================#        
        WP=DATA.PassedWayPointLst[:]
        WPX=[]
        WPY=[]
        WPTime=[]
        
        for val in WP:
            WPX.append(val[0])
            WPY.append(val[1])
            WPTime.append(val[2])
        plt.plot(WPX, WPY,'m*',  markersize=14, label="Way Points passed")
        
        #============#
        #++cosmetics++#
        #============#        
        plt.grid()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Autonomus car drive via given way points using FLC")
        plt.legend(loc="best")
        plt.grid
        plt.savefig("Path.png")
        #plt.show()
