#!/usr/bin/env python

import os
import time
import rospy
import sys



def run_C48_StandUp(): #start the gazebo and the standup-scripts
    #os.system('roslaunch atlas_utils atlas.launch &')
    #print "sleep timeeeeeeeeeeeeeeeeeeeeeeeeeeeeee!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    #time.sleep(50)
    print "#####################fall#####################"
    os.system('rosrun C48_StandUp Fall.py ')
    #open a new konsole
    #os.system('konsole')
    print "#####################stand up server#####################"
    os.system('rosrun C48_StandUp C48_StandUp.py &')
    #open a new konsole
    #os.system('konsole')
    #os.system('bash')
    #os.system('cp ~/robil/C48_StandUp/TestTree/StandUp.xml ~/robil/C34_Designer/plans/StandUp.xml')
    print "#####################executor started#####################"
    os.system('rosrun C34_Executer executer &')
    #open a new konsole
    #os.system('konsole') 
    #os.system('rosrun C34_Designer run.sh &')
    #os.system('konsole')
    

    
def run_Executor_Tree():#run resume and stop the tree
    #open a new konsole
    time.sleep(10)
    print "#####################run plan#####################"
    os.system('rosservice call /executer/run MYTREE plans/StandUp.xml')
    print "#####################resume plan#####################"
    os.system('rosservice call /executer/resume MYTREE ')
    time.sleep(20)
    print "#####################stop plan#####################"
    os.system('rosservice call /executer/stop MYTREE')
#def byebye():
#    pass
# Main function.
if __name__ == '__main__':
	tests = sys.argv[1]

	os.system('roslaunch atlas_utils atlas.launch &')

	for i in range((int)(tests)):
	  #print "in iteration " + i.tostring()
          run_C48_StandUp() 
          run_Executor_Tree()
          #run_Executor_Tree()
          
	#rospy.on_shutdown(byebye)
	
      
