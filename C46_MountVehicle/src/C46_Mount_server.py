#! /usr/bin/env python

import roslib; 
roslib.load_manifest('C46_MountVehicle')

import rospy, math
import actionlib
import yaml, sys

import C0_RobilTask.msg
from std_msgs.msg import Float64
#from control_msgs.msg import *

TASK_RESULT_REJECT=0
TASK_RESULT_OK=1
TASK_RESULT_PLAN=2

class MountVehicleServer(object):
  # create messages that are used to publish feedback/result
  _feedback = C0_RobilTask.msg.RobilTaskFeedback()
  _result   = C0_RobilTask.msg.RobilTaskResult()
  
    
  def __init__(self):
    self._action_name = "/MountVehicle"
    self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
    self._as.start()

  def task(self, goalAct):
	print ("Start Mounting Vehicle")
	task_success = True
	task_result = TASK_RESULT_OK
	task_plan = ""

	# start executing the action

	#rospy.init_node('trajectory')
	Traj_data_file = "./Traj_data.yaml"
	Traj_name_to_execute = "Mount"
	traj_yaml = yaml.load(file(Traj_data_file, 'r'))
	traj_name = Traj_name_to_execute
	if not traj_name in traj_yaml:
		print "unable to find trajectory %s in %s" % (traj_name, Traj_data_file)
		task_success = False
	if task_success:
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
		goal.trajectory.points = [ trajectory_msgs.msg.JointTrajectoryPoint() \
					for x in xrange(0, traj_len) ]
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


if __name__ == '__main__':
  rospy.init_node('C46_Mount')
  MountVehicleServer()
  rospy.spin()
