#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <move_hand/pelvis_move_hand.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>

class PickUp{
private:
	ros::NodeHandle nh_;
	ros::ServiceClient pelvis_move_hand_cli_;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	std::string action_name_;

public:
	PickUp(std::string name)
	:as_(nh_, name, false), action_name_(name){

	  pelvis_move_hand_cli_ = nh_.serviceClient<move_hand::pelvis_move_hand>("pelvis_move_hand");

		while(!move_hand_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the move_hand server");
		}

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&PickUp::goalCB, this));
//		as_.registerPreemptCallback(boost::bind(&PickUp::preemptCB, this));

		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~PickUp(){}

	void turnToWaypoint(double angle){

	}

	void goalCB(){

		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;


		move_hand::move_hand move_hand;



		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_ERROR("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
		}

		C0_RobilTask::RobilTaskResult _res;
		_res.success = C0_RobilTask::RobilTask::SUCCESS;
		as_.setSucceeded(_res);
		ROS_INFO("End time: %f", ros::Time::now().toSec());
		return;
	}
/*
	void preemptCB()
	{
		ROS_ERROR("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
	}*/
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C66_PickUp");

	PickUp* QuasiStaticWalker = new PickUp("PickUp");
	ROS_INFO("Running PickUp action");
	ros::spin();

	return 0;
}
