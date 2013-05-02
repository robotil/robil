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
static bool gotFeedback = false;
using namespace C67_CarManipulation;
typedef actionlib::SimpleActionClient<SteeringWheelAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const SteeringWheelResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: success:%i", result->success);
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const SteeringWheelFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback angle %f", feedback->angle);
  gotFeedback = true;
}

int main (int argc, char **argv)
{

	ros::init(argc, argv, "test_SteeringWheel2");

	// create the action client
	// true causes the client to spin its own thread
	Client ac("SteeringWheel", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	SteeringWheelGoal goal;
	goal.angle = pi/4;
	ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);


	//ac.cancelGoal();
	int i = 0, goalNum = 2;
	while (ros::ok()&&(!gotFeedback))
	{
		if(i >= 5)
		{
			std::cout<<"send goal #"<<goalNum<<"\n";
			ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
			goalNum++;
			i = 0;
		}
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		i++;
	}

	ros::spin();
	return 0;
}




