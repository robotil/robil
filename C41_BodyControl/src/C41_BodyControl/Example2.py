#! /usr/bin/env python

import roslib; 
roslib.load_manifest('C45_PostureControl')

import rospy, math
import actionlib

import C0_RobilTask.msg
from std_msgs.msg import Float64

TASK_RESULT_REJECT=0
TASK_RESULT_OK=1
TASK_RESULT_PLAN=2

def parametersParsing(params):
  m={}
  for kv in params.split(','): 
    k,v = kv.strip().split('=')
    m[k]=v
  return m

joint_names = [
  'l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax', 
  'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
  'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
  'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx',
  'neck_ay', 'back_lbz', 'back_mby', 'back_ubx'
]

trajectories = { 
  'step_and_fall' : 
  [ #points := [ time_from_prev, joints_positions ]
    [1.0, [0, -0.1,   0,   0,    0,    0,        0, -0.7, -1.2, 1.4, -1.4, 0,      0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,            0, 0, 0, 0.2 ] ], #point 1
    [0.5, [0,  0.4, 1.2, 0.8, -1.4,  0.2,        0, -0.7, -1.2, 1.4, -1.4, 0,      0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,            0, 0, 0,   0 ] ], #point 2
    [1.5, [0,  0,   0,   0,    0.1,  0,          0,  0,   -0.2, 0.4, -0.4, 0,      0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,            0, 0, 1,   1 ] ], #point 3
    [1.0, [0,  0,   0,   0,    0,    0,          0,  0,    0,   0,    0,   0,      0, 0, 0, 0, 0, 0,           0, 0, 0, 0, 0, 0,            0, 0, 0,   0 ] ]  #point 4
  ],
  'touchdown' : 
  [
    [2.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0, -0.5, 0, 2,   0, 0,      0,  0.5, 0, -2,   0, 0,      0, 0, 0, 0 ] ],
    [3.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0,  1.2, 0, 0.5, 0, 0,      0, -1.2, 0, -0.5, 0, 0,      0, 0, 0, 0 ] ],
    [1.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0, -1.0, 0, 0,   0, 0,      0,  1.0, 0,  0,   0, 0,      0, 0, 0, 0 ] ]
  ],
  'touchdown_exhausted' : 
  [
    [2.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0, -0.5, 0, 2,   0, 0,      0,  0.5, 0, -2,   0, 0,      0, 0, 0, 0 ] ],
    [3.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0,  1.2, 0, 0.5, 0, 0,      0, -1.2, 0, -0.5, 0, 0,      0, 0, 0, 0 ] ],
    [1.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 -1.0, -1.0, 0, 0,0, 0,    1.0,  1.0, 0,  0,   0, 0,      0, 0, 0, 0 ] ],
    [1.0, [0, 0, 0, 0, 0, 0,                   0, 0, 0, 0, 0, 0,                 0, -1.0, 0, 0,   0, 0,      0,  1.0, 0,  0,   0, 0,      0, 0, 0, 0 ] ]
  ]
}


class BodyControlServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C0_RobilTask.msg.RobilTaskFeedback()
  _result   = C0_RobilTask.msg.RobilTaskResult()
  
    
  def __init__(self):

    self.traj_client = actionlib.SimpleActionClient( '/drc_controller/follow_joint_trajectory', FollowJointTrajectoryAction )
    self.traj_client.wait_for_server()

    self._action_name = "/PostureControl"
    self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
    self._as.start()

  def finishTask(task_success, task_result, task_plan):
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

  def moveArm():
    task_success = True
    task_result = TASK_RESULT_OK
    task_plan = ""
    # start executing the action
      
	_arm = rospy.Publisher('/r_arm_ely_position_controller/command', Float64)
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
	   	t = 6 * rospy.get_time()
    	next_pos =  0.4 + 0.4 * math.sin(t)
    	_arm.publish(next_pos)
	     
		r.sleep()

	self.finishTask(task_success, task_result, task_plan)
	
  def procTrajectory(gesture):
    task_success = True
    task_result = TASK_RESULT_OK
    task_plan = ""
    # start executing the action

    drc_goal = FollowJointTrajectoryGoal()
    drc_goal.trajectory.joint_names = joint_names
    drc_goal.trajectory.points = [ ]

    guesture_duration = 0.0
    for i in xrange(0, len(gesture)): # for each point in gesture
      time_from_prev_position, joints_positions = gesture[i]
      guesture_duration += time_from_prev_position
      
      goal_pt = trajectory_msgs.msg.JointTrajectoryPoint()
      goal_pt.time_from_start = rospy.Duration.from_sec(guesture_duration)
      goal_pt.positions = joints_positions
      goal_pt.velocities = [0] * len(joint_names)
      
      drc_goal.trajectory.points.append(goal_pt)
      
    self.traj_client.send_goal(goal)
    guesture_duration = rospy.Duration.from_sec(guesture_duration + 3)

    #### DEFINE SLEEP DURATION BETWEEN TASK LOOP ITERATIONS ####
    r = rospy.Rate(100)

    #### SET NUMBER OF TASK LOOP ITERATIONS ####
    for i in xrange(1000): 
        if self._as.is_preempt_requested() or rospy.is_shutdown():
            #### HERE PROICESS PREEMTION OR INTERAPT #####
   
	    self.traj_client.cancel_goal();

            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            task_success = False
            break
            
    	#### HERE PROCESS TASK ####
   	
		finished = self.traj_client.wait_for_result(guesture_duration)
		if finished:
	  		break;
             
        r.sleep()

	self.finishTask(task_success, task_result, task_plan)

  def task(self, goal):
    #### GET TASK PARAMETERS ####
    rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
    rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
    rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);

    #### HERE PROCESS TASK PARAMETERS ####
    parameters = parametersParsing(goal.parameters)
    gesture = trajectories[parameters['param']

    if gesture == "MoveArm":
		self.moveArm()
	else:
		self.procTrajectory(gesture)

  
    
