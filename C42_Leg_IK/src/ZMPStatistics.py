# -*- coding: utf-8 -*-
import time
import sys
import matplotlib.pyplot as plt

TIME_INTERVAL = 500
TIME_BEFORE_FALLING = 3

    
    
def findFallingTime(f):
    prevLine=0
    for line in f:
        parsedLine = line.split(' ')
        if (parsedLine[1]=='HAS'):              
            fallingTime = float(prevLine[0])
            return fallingTime
                
        prevLine = parsedLine 
        
        
def collectStatistics(fallingtime, f):
    #time of minimum deltax/deltay, deltax/deltay, average deltax/deltay
    DX = [0,float("inf"),0]
    DY = [0,float("inf"),0]  
    statDX = []
    statDY = []
    fallMinDX = {}
    notFallMinDX = {}
    fallMinDY = {}
    notFallMinDY = {}
    f.seek(0)
    count = 1
    
    for line in f:
        if 'HAS' in line:
            DX[2] = DX[2]/count
            DY[2] = DY[2]/count
            statDX.append(DX)
            statDY.append(DY)
            (fallMinDX, notFallMinDX) = builtMap(fallMinDX, notFallMinDX,DX, fallingtime)
            (fallMinDY, notFallMinDY) = builtMap(fallMinDY, notFallMinDY,DY, fallingtime)
                
            print fallMinDX
            print notFallMinDX
            print fallMinDY
            print notFallMinDY  
            return (statDX,statDY)
        else:   
            parsedLine = line.split(' ')
            DX[2] = DX[2] + float(parsedLine[3])
            DY[2] = DY[2] + float(parsedLine[4])
            if float(parsedLine[3]) < DX[1]:
                DX[1] = float(parsedLine[3])
                DX[0] = float(parsedLine[0])
                     
            if float(parsedLine[4]) < DY[1]:
                DY[1] = float(parsedLine[4])
                DY[0] = float(parsedLine[0])
                
            count = count+1
            
            if (count >= TIME_INTERVAL): 
                DX[2] = DX[2]/TIME_INTERVAL
                DY[2] = DY[2]/TIME_INTERVAL
                statDX.append(DX)
                statDY.append(DY)
                (fallMinDX, notFallMinDX) = builtMap(fallMinDX, notFallMinDX,DX, fallingtime)
                (fallMinDY, notFallMinDY) = builtMap(fallMinDY, notFallMinDY,DY, fallingtime)
                DX = [0,float("inf"),0]
                DY = [0,float("inf"),0] 
                count = 0
   
          
    return (statDX,statDY)

def plotZMP(statDX,statDY,ft):
    dxtime=[]
    dxmin=[]
    dxavg=[]
    dytime=[]
    dymin=[]
    dyavg=[]
    for x in statDX:
       dxtime.append(x[0]) 
       dxmin.append(x[1]) 
       dxavg.append(x[2])
    for y in statDX:
       dytime.append(y[0]) 
       dymin.append(y[1]) 
       dyavg.append(y[2]) 
       
    plt.plot(dxtime, dxmin, 'ro')
    plt.axis([0, ft, 0, 3])   
    
def builtMap(fall, notFall, d,ft):
    #more than  TIME_BEFORE_FALLING   
    if (ft-d[0]>TIME_BEFORE_FALLING):
        if round(d[1],1) in notFall:
            notFall[round(d[1],1)]= notFall[round(d[1],1)]+1
        else:
            notFall[round(d[1],1)]=1
    #less than  TIME_BEFORE_FALLING           
    else:        
        if round(d[1],1) in fall:
            fall[round(d[1],1)]= fall[round(d[1],1)]+1
        else:
            fall[round(d[1],1)]=1
            
    return(fall, notFall)
    
if __name__ == "__main__":
    
    f = open('zmp.txt', 'r')
    
    fallingTime = findFallingTime(f)
    
    if (float(fallingTime)>0):
        print 'First falling time:',fallingTime  
        (statDX,statDY) = collectStatistics(float(fallingTime), f)
        
    #plotZMP(statDX,statDY,float(fallingTime))    

    #print statDX
    #print statDY
