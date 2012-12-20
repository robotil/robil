#! /usr/bin/env python

import roslib, yaml; 
roslib.load_manifest('C41_BodyControl')

import rospy, math
import actionlib

import C0_RobilTask.msg
from std_msgs.msg import Float64

TASK_RESULT_REJECT=0
TASK_RESULT_OK=1
TASK_RESULT_PLAN=2

class BodyControlServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C0_RobilTask.msg.RobilTaskFeedback()
  _result   = C0_RobilTask.msg.RobilTaskResult()
  
    
  def __init__(self):
    print "SERVER STARTED"
    self._action_name = "/BodyControl"
    self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
    self._as.start()

  def task(self, goal):
    print "TASK STARTED"
    task_success = True
    task_result = TASK_RESULT_OK
    task_plan = ""

    # start executing the action

    #### GET TASK PARAMETERS ####
    rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
    rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
    rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);

    #### HERE PROCESS TASK PARAMETERS ####
    if goal.parameters == "param=MoveArm" :

      print "PARAM FAUND"

#      _arm = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
      #### DEFINE SLEEP DURATION BETWEEN TASK LOOP ITERATIONS ####
      r = rospy.Rate(100)

      #### SET NUMBER OF TASK LOOP ITERATIONS ####
      for i in xrange(1000): 
	if self._as.is_preempt_requested() or rospy.is_shutdown():
	    #### HERE PROICESS PREEMTION OR INTERAPT #####
   
	    rospy.loginfo('%s: Preempted' % self._action_name)
	    self._as.set_preempted()
	    task_success = False
	    break
		    
    	#### HERE PROCESS TASK ####
	print "TASK PROCESS"
#   	t = 6 * rospy.get_time()
#    	next_pos =  0.4 + 0.4 * math.sin(t)
#    	_arm.publish(next_pos)
	     
	r.sleep()
      
    if goal.parameters == "param=Walk" :
	
      print "working on it"
	
      traj_yaml = yaml.load(file("/home/ariy/robil/Traj_data.yaml", 'r'))
      traj_name = "step_and_fall"
      if not traj_name in traj_yaml:
	print "unable to find trajectory %s in %s" % (traj_name, sys.argv[1])
	sys.exit(1)
      traj_len = len(traj_yaml[traj_name])
      traj_client = actionlib.SimpleActionClient( \
			         '/drc_controller/follow_joint_trajectory', \
			         FollowJointTrajectoryAction)
      traj_client.wait_for_server()
      # now, build the trajectory-action goal message from the YAML snippet
      goal = FollowJointTrajectoryGoal()
      goal.trajectory.joint_names = ['l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy',
			         'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
			         'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy',
			         'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
			         'l_arm_usy', 'l_arm_shx', 'l_arm_ely',
			         'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
			         'r_arm_usy', 'r_arm_shx', 'r_arm_ely',
			         'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx',
			         'neck_ay', 
			         'back_lbz', 'back_mby', 'back_ubx']
      goal.trajectory.points = [ trajectory_msgs.msg.JointTrajectoryPoint() for x in xrange(0, traj_len) ]
      t = 0.0
      for i in xrange(0, traj_len):
	  y = traj_yaml[traj_name][i]
	  goal_pt = goal.trajectory.points[i]
	  t += float(y[0])
	  goal_pt.time_from_start = rospy.Duration.from_sec(t)
	  goal_pt.velocities = [0] * 28
	  goal_pt.positions = [ float(x) for x in y[1].split() ]
      traj_client.send_goal(goal)
      traj_client.wait_for_result(rospy.Duration.from_sec(t + 3))

    if task_success:
      self._result.success = task_result;
      rospy.loginfo("%s: Succeeded", self._action_name);
      if task_result == TASK_RESULT_PLAN:
        ROS_INFO("%s: New plan", self._action_name);
        self._result.plan = task_plan;
      self._as.set_succeeded(self._result);
    else:
      rospy.loginfo("%s: Aborted", self._action_name);
      self._as.set_aborted(self._result);
    print "TASK FINISHED"

