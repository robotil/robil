import rospy
import numpy as np
from math import *
import numpy
import matplotlib.pyplot as plt
def listUpt(list, obj):
        list.append(obj)
        return list
def plotGraph(DATA):
        print DATA.DistanceErrorLst
        #print DATA.MyPathLst
        print DATA.PassedWayPointLst
        print DATA.WayPointLst
        print DATA.WayPointLst[:][:][0]
        for i, val in enumerate(DATA.WayPointLst):
            print i, val
            #plt.plot(DATA.WayPointLst[i][0], DATA.WayPointLst[i][1], 'r*',  markersize=14, label='WP Path')
        #plt.plot(xo, yo, label='WP Path')
        #plt.quiver(xo[:-1], yo[:-1], xo[1:]-xo[:-1], yo[1:]-yo[:-1],label='WP Path', scale_units='xy', angles='xy', scale=1)
        myPath=DATA.MyPathLst[::100]
        for i, val in enumerate(myPath):
            print i
            plt.plot(DATA.MyPathLst[i][0], DATA.MyPathLst[i][1], label="Golf Cart's Path")
        #plt.plot(xe, ye,'r*',  markersize=14, label="arrived at way Point")
        #plt.grid()
        #plt.xlabel("X")
        #plt.ylabel("Y")
        #plt.title("Autonomus car drive via given way points using FLC")
        #plt.legend(loc="best")
        #plt.grid
        #row_label=['Way Point', 'End of WP', 'Distance Error']
        plt.show()

def plotGraphs( actlist, clist, erlist,olist):
        lst1, lst2 = [], []
        xe=[]
        ye=[]
        for el in olist:
            lst1.append(el[0])
            lst2.append(el[1])
        xo=np.array(lst1)
        yo=np.array(lst2)
        lst1, lst2 = [], []
        for el in clist:
            lst1.append(el[0])
            lst2.append(el[1])
        xc=np.array(lst1)
        yc=np.array(lst2)
        for el in erlist:
            xe.append(el[0])
            ye.append(el[1])
        plt.figure
        #for object in list:
        #   plt.plot(object[0], object[1])
        #plt.plot(list)
        plt.plot(xo, yo, label='WP Path')
        plt.quiver(xo[:-1], yo[:-1], xo[1:]-xo[:-1], yo[1:]-yo[:-1],label='WP Path', scale_units='xy', angles='xy', scale=1)
        plt.plot(xc, yc, label="Golf Cart's Path")
        plt.plot(xe, ye,'r*',  markersize=14, label="arrived at way Point")
        plt.grid()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Autonomus car drive via given way points using FLC")
        plt.legend(loc="best")
        plt.grid
        row_label=['Way Point', 'End of WP', 'Distance Error']
        j=1
        col_label=[]
        for i in xe:
            label='WP%d'%j
            j+=1
            col_label.append(label)
        table_vals=[olist, erlist, errorplot(actlist, erlist)]
        '''        print "start"
        pub = rospy.Publisher('/drc_world/robot_enter_car',Pose)
        msg=Pose()
        pub.publish(msg)
        rospy.sleep(5)
        print "continue"
        the_table = plt.table(cellText=table_vals,
                  colWidths = [0.1]*(len(col_label)),
                  rowLabels=row_label,
                  colLabels=col_label, 
                  loc='bottom middle')
        #plt.text(0,-4,'Table Title',size=10)
        #print the_table
        '''

        plt.savefig("example.png")
        plt.show()
def errorplot(olist, erlist):
        lst1, lst2 = [], []
        for el in olist:
            lst1.append(el[0])
            lst2.append(el[1])
        xo=np.array(lst1)
        yo=np.array(lst2)
        lst1, lst2 = [], []
        for el in erlist:
            lst1.append(el[0])
            lst2.append(el[1])
        xc=np.array(lst1)
        yc=np.array(lst2)
        ex=xc-xo
        ey=yc-yo
        DistError=(ex**2+ey**2)**0.5
        print DistError
        return DistError
