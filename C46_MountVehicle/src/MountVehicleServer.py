#!/usr/bin/env python
import roslib; roslib.load_manifest('C46_MountVehicle')
import rospy, actionlib
import C0_RobilTask.msg
from control_msgs.msg import *

TASK_RESULT_REJECT=1
TASK_RESULT_OK=0
TASK_RESULT_PLAN=-1

traj_yaml = [[2.0, ' 0 0 0 0 0 0     0 0 0 0 0 0     -0.5 -0.3 0 0 0 0     -0.5 0 0 0 0 0     0 -0.3 -0.1 0.5'], [2.0, ' 0 0 0.3 0 0.1 0     -0.5 0 -1.5 0.5 0 0     -0.5 -0.3 0 0 0 0     -0.5 0 0 0 0 0     0 -0.3 0.1 0.5'], [2.0, ' 0.3 0.3 0.3 0 0.3 0     -1 0 -1.5 0.5 0.7 0     -0.5 -0.3 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.3 0.1 0.5'], [2.0, ' 0.3 0.3 0.3 0 0.3 0     -1 0 -1.5 1.5 0.7 0     -0.5 -0.3 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.3 0.1 0.5'], [2.0, ' 0.3 0.3 -1.5 1.5 1 0     -0.5 0 -1.5 1.5 0.7 1     -0.5 -0.3 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.3 0.1 0.5'], [2.0, ' 0.3 0.3 -1.5 1.5 1 0     -0.5 0.5 -1.5 1.5 0.7 1     -0.5 1.5 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.5 0.1 0.5'], [2.0, ' -0.3 0.3 -1.7 0.5 1 0     -0.5 0.5 -1.5 1.5 0.7 0.5     -0.5 1.5 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.5 0.1 0.5'], [2.0, ' -0.3 0.3 -1.3 1.3 1 0     -0.5 0.5 -1.5 1.5 1 0     -0.5 1.5 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.5 0.1 0.5'], [2.0, ' -0.3 0.3 -1.3 1.3 1 -0.5     0 0 -1.5 1.5 0 0     -0.5 1.5 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.5 0.1 0.5'], [2.0, ' 0.7 0.7 -1.3 1.3 1 -0.5     0 0 -1.5 1.5 0 0     -0.5 1.5 0 0 0 0     -0.5 0 -0.5 -0.5 0 0     0 -0.5 0.1 0.5'], [2.0, ' 0 0 -1.5 1.5 0 0     0 0 -1.5 1.5 0 0     -1.5 -1.3 0.7 1 0 0     1.5 1.3 0.7 -1 0 0     0 0 0 0']]

class MountVehicleServer(object):
	# create messages that are used to publish feedback/result
	_feedback = C0_RobilTask.msg.RobilTaskFeedback()
	_result   = C0_RobilTask.msg.RobilTaskResult()

	def __init__(self):
		print "MountVehicle SERVER STARTED"
		self.traj_client = actionlib.SimpleActionClient( '/atlas_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		self.traj_client.wait_for_server()
		
		self._action_name = "MountVehicle"
		self._as = actionlib.SimpleActionServer(self._action_name, C0_RobilTask.msg.RobilTaskAction, execute_cb=self.task)
		self._as.start()
	
	def finishTask(self, task_success, task_result, task_plan):
		print "FINISH TASK : ",task_success, task_result, task_plan
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
	
	def task(self, goal):
		print "START TASK"
		#### GET TASK PARAMETERS ####
		rospy.loginfo("%s: Start: task name = %s",self._action_name, goal.name);
		rospy.loginfo("%s: Start: task id = %s", self._action_name, goal.uid);
		rospy.loginfo("%s: Start: task params = %s", self._action_name, goal.parameters);

		task_success = True
		task_result = TASK_RESULT_OK
		task_plan = ""

		traj_len = len(traj_yaml)
		#rospy.init_node('traj')
		# now, build the trajectory-action goal message from the YAML snippet
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ['l_leg_uhz', 'l_leg_mhx', 'l_leg_lhy', 'l_leg_kny', 'l_leg_uay', 'l_leg_lax',
				                       'r_leg_uhz', 'r_leg_mhx', 'r_leg_lhy', 'r_leg_kny', 'r_leg_uay', 'r_leg_lax',
				                       'l_arm_usy', 'l_arm_shx', 'l_arm_ely', 'l_arm_elx', 'l_arm_uwy', 'l_arm_mwx',
				                       'r_arm_usy', 'r_arm_shx', 'r_arm_ely', 'r_arm_elx', 'r_arm_uwy', 'r_arm_mwx',
				                       'neck_ay', 'back_lbz', 'back_mby', 'back_ubx']
		goal.trajectory.points = [ trajectory_msgs.msg.JointTrajectoryPoint() for x in xrange(0, traj_len) ]
		t = 0.0
		for i in xrange(0, traj_len):
			y = traj_yaml[i]
			goal_pt = goal.trajectory.points[i]
			t += float(y[0])
			goal_pt.time_from_start = rospy.Duration.from_sec(t)
			goal_pt.velocities = [0] * 28
			goal_pt.positions = [ float(x) for x in y[1].split() ]
		self.traj_client.send_goal(goal)
		self.traj_client.wait_for_result(rospy.Duration.from_sec(t + 3))

		self.finishTask(task_success, task_result, task_plan)
