#! /usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from RobilTaskPy import *
from std_msgs.msg import String
#from std_msgs.msg import Bool	#Boolean value indicating the robot has fallen.
#from nav_msgs.msg import Odom
from sandia_hand_msgs.msg import RawTactile
#from sandia_hand_msgs.srv import SimpleGraspSrv
#from sandia_hand_msgs.srv import SimpleGraspSrvResponse

		
class GraspingMonitor(RobilTask):
	detected_problem = False
 # f0: index finger, f1: middle finger, f2: little finger, 
 #f3: thumb, palm: palm. 
 #Tactile sensors on finger have an array of size 18. 
 #Index 0~5 corresponds to bottom-half of finger, 
 #and index 6~17 corresponds to top half. 
 #The output value of each sensor is within the range 
 #from 26500 to 33500. The lower bound, 26500,
 # means zero or minimal force feedback.

      
	init_time = -1
	started_task = False
	run_num = 0
	
	def callback(self, msg):
            #print("Start: ", end ='')
           # print (str)(GraspingMonitor.run_num)
            #GraspingMonitor.run_num += 1           
            index_finger  = list(msg.f0)
            middle_finger = list(msg.f1)
            little_finger = list(msg.f2)
            thumb_finger  = list(msg.f3)
            palm          = list(msg.palm)
            s_publish =""
            for i in range(len(index_finger)):
                #normalize value range(0,1)
                #check if float type 
                    index_finger[i]   =  (float(index_finger[i])-26500) / (7000)
                    middle_finger[i]  =  (float(middle_finger[i])-26500)/ (7000)
                    little_finger[i]  =  (float(little_finger[i])-26500)/ (7000)
                    thumb_finger[i]   =  (float(thumb_finger[i])-26500) / (7000)
                    
                    if(index_finger [i] > 0):
                        finger_half  = ""
                        if( i < 6):
                            finger_half = " bottom "
                        else:
                                finger_half = " top "
                        print ("index finger"+ finger_half +"of finger force is" +str(index_finger [i]))
                        s_publish +="f0:"+ +str(index_finger [i])+";"
                    if(middle_finger[i] > 0):
                        if( i < 6):
                            finger_half = " bottom "
                        else:
                                finger_half = " top "
                        print ("middle finger"+ finger_half +"of finger force is: " +str(middle_finger[i]))
                        s_publish +="f1:"+ +str(middle_finger[i])+";"
                    
                    if(little_finger[i] > 0):
                        if( i < 6):
                            finger_half = " bottom "
                        else:
                            finger_half = " top "
                        print ("little finger"+ finger_half +"of finger force is: " +str(little_finger[i]))
                        s_publish +="f2:"+ +str(little_finger[i])+";"
                    
                    if(thumb_finger[i] > 0):
                        if( i < 6):
                            finger_half = " bottom "
                        else:
                            finger_half = " top "
                        print ("thumb"+ finger_half +"of finger force is: " +str(thumb_finger[i]) )
                        s_publish +="f3:"+ +str(thumb_finger[i])+";"
                    
            for i in range(len(palm)): 
                    palm[i]   =  (float(palm[i])-26500) / (7000)
                    if(palm[i] > 0):
                        print ("palm force is: " +str(palm[i]) )
                        s_publish +="palm:"+ +str(palm[i])+";"
            if s_publish !="":            
                self.resultPublisher.publish(s_publish)
#  #index_finger
#            print ("index_finger: ")
#            print ("bottom-half:")
#            
#            for i in range(6):
#                print((str)(index_finger[i])+" ", end ='')
#                 
#            print ("")   
#            print ("top half: ")
#            for i in range(6,17):
#                print((str)(index_finger[i])+" ", end='')
#                
#            print ("") #new line
#            
#   #middle_finger
#            print ("middle_finger: ")
#            print ("bottom-half:")
#            for i in range(6):
#                print((str)(middle_finger[i])+" ", end='') 
#            print ("")                  
#            print ("top half: ")
#            for i in range(6,17):
#                print((str)(middle_finger[i])+" ", end='') 
#                
#            print ("") #new line
#                
#   #little_finger
#            print ("little_finger: ")
#            print ("bottom-half:")
#            for i in range(6):
#                print((str)(little_finger[i])+" ", end='') 
#            print ("")                  
#            print ("top half: ")
#            for i in range(6,17):
#                print((str)(little_finger[i])+" ", end='')
#                
#            print ("") #new line
#                
#   #thumb_finger
#            print ("thumb_finger: ")
#            print ("bottom-half:")
#            for i in range(6):
#                print((str)(thumb_finger[i])+" ", end='') 
#            print ("")                  
#            print ("top half: ")
#            for i in range(6,17):
#                print((str)(thumb_finger[i])+" ", end='')
#                
#            print ("") #new line
#                
#   #palm
#            print ("palm: ")
#            print ("bottom-half:")
#            for i in range(6):
#                print((str)(palm[i])+" ", end='') 
#            print ("")                  
#            print ("top half: ")
#
#            for i in range(6,17):
#                print((str)(palm[i])+" ", end='') 
#            print ("") 

            
#		if GraspingMonitor.started_task:
#		  GraspingMonitor.started_task = False
#		  GraspingMonitor.fo = msg.
#		  GraspingMonitor.init_y = msg.pose.pose.position.y
#		  print "Grasping monitor got first message."
#		else:  
#		  delta_x = msg.pose.pose.position.x - GraspingMonitor.init_x
#		  delta_y = msg.pose.pose.position.y - GraspingMonitor.init_y
#		  elapsed_time = rospy.get_time() - GraspingMonitor.init_time
#		  
#		  if GraspingMonitor.init_x != -1 and GraspingMonitor.init_y != -1 and elapsed_time > 10 and math.fabs(delta_x) < 1 and math.fabs(delta_y) < 1:
#			  GraspingMonitor.detected_problem = True
	
	def __init__(self, name):
		print ("Initializing grasping monitoring left Node")
		rospy.Subscriber("/sandia_hands/l_hand/tactile_raw", RawTactile, self.callback)
		GraspingMonitor.detected_problem = False
		GraspingMonitor.run_num = 0
		self.resultPublisher = rospy.Publisher("C35/grasp_force_l",String)
		RobilTask.__init__(self, name)
	
	def task(self, name, uid, parameters):
		print ("Start Monitoring grasping left of the robot." )
		#print parameters
		d = rospy.Duration(1,0)
		
		GraspingMonitor.init_time = rospy.get_time()
		GraspingMonitor.started_task = True
		print ("Grasping monitor left received first message.")
		while not GraspingMonitor.detected_problem:	
			#print GraspingMonitor.detected_problem
			if self.isPreepted():
				print ("Preempt grasping left monitoring task")
				return RTResult_PREEPTED()
			rospy.sleep(d)
		print ("detected problem in grasping left module! EXITING...")
		GraspingMonitor.detected_problem = False
		error_code = RobilTask_FAULT + 1
		return RTResult_ABORT(error_code,"Grasping Monitor Left detected a problem. "+str(error_code));
		#return RTResult_SUCCESSED("Finished in Success")
      
if __name__ == '__main__':
	print ("Start GraspingMonitorLeft Node")
	rospy.init_node('GraspingMonitorLeft')
	GraspingMonitor("GraspingMonitorLeft")
	rospy.spin()
	print ("Grasping monitor Left Node Closed") 