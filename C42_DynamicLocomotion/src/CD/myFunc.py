import numpy
from math import acos, pi, sin
#from pylab import *
def findIndex(pose, Path):
        pos = numpy.array(pose)
        path = numpy.array(Path)
        nearest = 0
        min_dist = numpy.linalg.norm(pos-path[0])
        i=0
        #Find the closest point
        for pt in path:
            dist = numpy.sqrt((pos[0]-pt[0])**2+(pos[1]-pt[1])**2)
            if dist<= min_dist:
                min_dist = dist
                nearest = i
            
            i+=1
        
        if nearest == len(path)-1 and nearest ==0:
            return path
        if nearest == len(path)-1:
            C=numpy.array(path[nearest])
            B=numpy.array(path[nearest-1])
            b=B-C
            c=C-pos
            a=B-pos
            G=acos(numpy.dot(b, -c)/(numpy.linalg.norm(b)*numpy.linalg.norm(c)))
#            if G>pi*0.5:
#                nearest+=1
                
            return path[nearest]
        
        
        C=numpy.array(path[nearest])
        B=numpy.array(path[nearest+1])
        b=B-C
        c=C-pos
        a=B-pos
        G=acos(numpy.dot(b, -c)/(numpy.linalg.norm(b)*numpy.linalg.norm(c)))
        if G<pi*0.5:
            nearest+=1
        
        p=[]
        for i in range(nearest, len(path)):
            p.append(Path[i])
        return p
#        
#
#pose = (0.8, 0.5)
#X=numpy.linspace(0, pi, 10)
#Y=[]
#Y=[sin(x) for x in X]
#path=[]
#for i in range(0, 10):
#        path.append([X[i], Y[i]])
##print path
#
#a=findIndex(pose, path)
#print a
#Xn = a[0][0]
#Yn = a[0][1]
#figure()
#plot(X, Y, 'bo', pose[0], pose[1], 'rx', Xn, Yn, 'ro')
#show()
