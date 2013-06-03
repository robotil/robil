# -*- coding: utf-8 -*-
import time
import sys




def findMin(f):
    absMinDX = [0,float("inf")]
    absMinDY = [0,float("inf")]
    fallingTime = 0                 #exact falling time
    flag = 1
    deltaXMinusTime = 0
    deltaYMinusTime = 0
    localMinDX=[0,0]
    localMinDY=[0,0]
    prevLine = 0
    for line in f:
        parsedLine = line.split(' ')
        if (parsedLine[1]!='HAS'):              
            if float(parsedLine[3]) < absMinDX[1]:
                absMinDX[1] = float(parsedLine[3])
                absMinDX[0] = float(parsedLine[0])
                lineMinDX = line
                if deltaXMinusTime==0:
                    if absMinDX[1]<0:
                        deltaXMinusTime = parsedLine[0]
                
                
            if float(parsedLine[4]) < absMinDY[1]:
                absMinDY[1] = float(parsedLine[4])
                absMinDY[0] = float(parsedLine[0])
                lineMinDY = line
                if deltaYMinusTime==0:
                    if absMinDY[1]<0:
                        deltaYMinusTime = parsedLine[0]
                    
                    
        else:
            if flag:
                fallingTime = float(prevLine[0])
                localMinDX[0] = absMinDX[0]
                localMinDY[0] = absMinDY[0]
                localMinDX[1] = absMinDX[1]
                localMinDY[1] = absMinDY[1]
                flag = 0
                
        prevLine = parsedLine 

        
    print 'First time DeltaX<0, DeltaY<0:'    
    print deltaXMinusTime    
    print deltaYMinusTime
    print 'Time and min val of DeltaX, DeltaY before first falling:'    
    print localMinDX    
    print localMinDY
    
    return (fallingTime, lineMinDX, lineMinDY)
    
def fallingDataLastSec(f,t):
    tmp = str(t-1)
    f.seek(0)
    count = 0
    lastSecDB=[]
    flag  = 0
    for line in f:
        if tmp in line:
            flag = 1 
        if flag and count<1001:   
            lastSecDB.append(line.split(' ')) 
            count = count+1
              
    return lastSecDB
            

if __name__ == "__main__":
    f = open('zmp.txt', 'r')
    
    (fallingTime, lineMinDX, lineMinDY) = findMin(f)
    
    if (fallingTime>0):
        print 'First falling time:',fallingTime  
        lastSecDB = fallingDataLastSec(f,fallingTime)

    print 'Time and min val of DeltaX, DeltaY before first falling:'        
    print lineMinDX
    print lineMinDY
