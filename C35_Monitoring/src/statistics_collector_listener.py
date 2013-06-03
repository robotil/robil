#!/usr/bin/env python
import roslib; roslib.load_manifest('C35_Monitoring')
import rospy
from std_msgs.msg import String
import time
import sys

class statistics_collector(object):
	_node_start_times_by_id = {}
	_node_execution_times_by_id = {}
	_node_total_execution_times_by_id = {}
	_node_count_success_by_id = {}
	_success_times = {}
	_failur_times = {}
	_count_runs = 0
	_plan_success = 0
	_last_success = False
	_num_of_tests = 0
	_f1=open('./result.txt', 'a+')

	
	def callback(self,data):
#	    parse the data
	    #print "          "+data.data
	    code = data.data[data.data.find("code=")+5]
	    node_name = data.data[data.data.find("node=")+5: data.data.find("(",data.data.find("node=")+5)]
	    node_id = data.data[data.data.find("[id=")+4 : data.data.find("]", data.data.find("[id=")+4)]
	    node_success = data.data.find("OK") > -1
	    
	    #print "        "+node_name
	    #print "         "+(str)(data.data.find(":OK$"))
	    #print "          "+(str)(node_success)	    
	    
	    if "Task" in node_name:
	      statistics_collector._last_success = node_success
	      
	    if "Unknown" in node_name:
	      statistics_collector._count_runs += 1
	      temp = statistics_collector._node_execution_times_by_id
	      run_time = 0
	      
	      
	      for key in temp:
	            run_time += (temp[key])
	      print "Test number: "+(str)(statistics_collector._count_runs)+" succeeded: "+(str)(statistics_collector._last_success)+", test running time: " +(str)(run_time)
	      print >> statistics_collector._f1, "Test number: "+(str)(statistics_collector._count_runs)+" succeeded: "+(str)(statistics_collector._last_success)+", test running time: " +(str)(run_time)+" "
	      if statistics_collector._last_success == True:
		statistics_collector._success_times[statistics_collector._count_runs] = run_time
		statistics_collector._plan_success += 1
	      else:
		statistics_collector._failur_times[statistics_collector._count_runs] = run_time
		
	     # if node_success == True:9
	#	_statistics_collector._plan_success +=1
	      if statistics_collector._count_runs >= (int)(statistics_collector._num_of_tests):
		self.printStatistics()
	      return
	    

	    #f1=open('./testfile', 'w+')rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)

	    if "1" in code: #This means a task is over
              #statistics_collector._node_counter += 1
              statistics_collector._node_execution_times_by_id[node_id] = time.time()-statistics_collector._node_start_times_by_id[node_id]
              a = statistics_collector._node_execution_times_by_id[node_id] 

              statistics_collector._node_total_execution_times_by_id[node_id] +=  a           
              if node_success == True:
		  if node_id not in statistics_collector._node_count_success_by_id:
		    statistics_collector._node_count_success_by_id[node_id] = 1
		  else:
		    statistics_collector._node_count_success_by_id[node_id] += 1
		  
	    else:
              statistics_collector._node_start_times_by_id[node_id] = time.time()
              if node_id not in statistics_collector._node_count_success_by_id:
		statistics_collector._node_total_execution_times_by_id[node_id] = 0
		statistics_collector._node_count_success_by_id[node_id] = 0
             
 
         

	def listener(self):
          rospy.init_node('listener', anonymous=True)
          rospy.spin()
     
     	def printStatistics(self):
	  temp_s = statistics_collector._success_times
	  temp_f = statistics_collector._failur_times
	  sum_avg_time_s = 0
	  sum_avg_time_f = 0
          print " Results:"
          statistics_collector._f1.write("Results:")
          #avg time of failure
          for key in temp_s:
	      sum_avg_time_s += (temp_s[key]/len(temp_s))
              #print "Average time of node :"+ key +" is :"+ (str)(temp[key]/statistics_collector._node_counter)
              #print "Success :" +(str)((statistics_collector._node_count_success_by_id[key]/statistics_collector._node_counter)*100)+ "%"
          #avg time of success
          for key in temp_f:
	      sum_avg_time_f += (temp_f[key]/len(temp_f))
	      
          print " Average time of succesful action: " +(str)(sum_avg_time_s)
          statistics_collector._f1.write(" Average time of succesful action: " +(str)(sum_avg_time_s))
          print " Average time of failure action: " +(str)(sum_avg_time_f)
          statistics_collector._f1.write(" Average time of failure action: " +(str)(sum_avg_time_f))
          #print "##" + (str)(len(temp_s)) + "#### "+(str)(len(temp_f))
          print "Total Success: " +(str)((len(temp_s)*100./(len(temp_s)+len(temp_f))))+ "% "
          print >> statistics_collector._f1, " Total Success: " +(str)((len(temp_s)*100./(len(temp_s)+len(temp_f))))+ "% "
          statistics_collector._f1.close()
          sys.exit()
          
	def __init__(self):
          rospy.Subscriber("/executer/stack_stream", String, self.callback)

            


if __name__ == '__main__':
    
    statistics_collector._num_of_tests = sys.argv[1]
    collector = statistics_collector()
    collector.listener()
    
    
    
    
    
    