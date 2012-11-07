/*
 * test_action_client.cpp
 *
 *  Created on: Sep 27, 2012
 *      Author: dan
 */


#include <SimpleTask/BTTaskAction.h>
#include <actionlib/client/simple_action_client.h>
#include <SimpleTask.h>

typedef actionlib::SimpleActionClient<SimpleTask::BTTaskAction> Client;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_simpleTask");
	Client client("SimpleTask", true); // true -> don't need ros::spin()
	client.waitForServer();
	SimpleTask::BTTaskGoal goal;
	goal.name = "set_tname";
	goal.uid = "set_tid";
	goal.parameters = "set_tparam";
	client.sendGoal(goal);
	client.waitForResult(ros::Duration(5.0));
	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Yay! The task are now clean");
	ROS_INFO("Current State: %s", client.getState().toString().c_str());
	if (client.getResult()->success == SimpleTask::PLAN)
		ROS_INFO("plan is %s",client.getResult()->plan.c_str());
	return 0;
}

