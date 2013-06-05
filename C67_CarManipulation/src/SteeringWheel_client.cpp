/*
 * SteeringWheel_client.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: gotesdyner
 */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <C67_CarManipulation/SteeringWheelAction.h>

static const float pi = 3.14159265;

int main (int argc, char **argv)
{

	ros::init(argc, argv, "test_SteeringWheel");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<C67_CarManipulation::SteeringWheelAction> ac("SteeringWheel", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	C67_CarManipulation::SteeringWheelGoal goal;
	goal.angle = pi/4;
	ac.sendGoal(goal);
	ros::Duration(1).sleep();

	//wait for the action to return
	std::cout << ac.getState().toString() << std::endl;
	std::cout << ac.getResult() << std::endl;
	bool finished_before_timeout = ac.waitForResult(ros::Duration(7.0));

	if (finished_before_timeout)
	{
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
	{
		ROS_INFO("Action did not finish before the time out.");
		std::cout << ac.getResult() << std::endl;
		std::cout << ac.getState().toString() << std::endl;
		ac.cancelGoal();

	}

	//ac.cancelGoal();


	//exit
	return 0;
}




