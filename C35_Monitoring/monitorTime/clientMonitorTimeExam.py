  #! /usr/bin/env python 
    import roslib; roslib.load_manifest('C35_Monitoring')#my_pkg_name
    import rospy
    import actionlib
    
    from chores.msg import *
    
    if __name__ == '__main__':
       rospy.init_node('monitor_time_client')
       client = actionlib.SimpleActionClient('monitor_time', monitorTimeAction)
       client.wait_for_server()
   
   
       goal = DoMonitorTimeGoal()
       # Fill in the goal here
       client.send_goal(goal)
       client.wait_for_result(rospy.Duration.from_sec(5.0))
       
       #create the goal to send to the server
    def DoMonitorTimeGoal():
        pass
    