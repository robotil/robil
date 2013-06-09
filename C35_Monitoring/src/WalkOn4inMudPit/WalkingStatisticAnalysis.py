import roslib
roslib.load_manifest('C35_Monitoring')
import tf
from tf_conversions import posemath
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math
import rospy
import sys

SAMPLE = 500

def createStsFile ():
    statistics = []
    mmuOneSec = [0,0,0,0]   
    data = [0,0,0]
    count = 0
    statHash = {}
    avg = 0
    smpl = 0
    files = ['DW1_flat','DW3_in_mut_flat','DW4_exit','DW5_after_exit_flat' ]
    fOutput = open('statistics2', 'w')
    for f in files:
        fInput = open(f, 'r')
        for line in fInput:
            parsedLine = line.split(' ') 
            if parsedLine[0] != '---\n':
                mmuOneSec[count%4] = parsedLine[1]           
                count = count + 1
                if (count%4 == 0) and (count != 0):
                    roll, pitch, yaw = euler_from_quaternion([mmuOneSec[0], mmuOneSec[1], mmuOneSec[2], mmuOneSec[3]])
                    data[0] = roll 
                    data[1] = pitch
                    data[2] = yaw
                    avg = avg + pitch
                    statistics.append(data)
                    if (smpl%SAMPLE == 0) and (smpl != 0):
                        avg = avg/SAMPLE
                        if (f=='DW4_exit'):
                            if round(avg,1) in statHash:
                                statHash[round(avg,1)]= [statHash[round(avg,1)][0]+1,statHash[round(avg,1)][1]+1 ]
                            else:
                                statHash[round(avg,1)]=[1,1]
                        else:
                            if round(avg,1) in statHash:
                                statHash[round(avg,1)]= [statHash[round(avg,1)][0]+1,statHash[round(avg,1)][1]]
                            else:
                                statHash[round(avg,1)]=[1,0]
                        avg = 0        
                    smpl = smpl + 1
        fInput.close          
    for val in statHash:
        fOutput.write('%s %s %s \n' %(val, statHash[val][0], statHash[val][1] ))            
                    
        
    fOutput.close        
		


if __name__ == "__main__":
    
    createStsFile()
    
    
