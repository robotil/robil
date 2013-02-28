#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <move_hand/move_hand.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <osrf_msgs/JointCommands.h>

class PickUp{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle* rosnode;
	ros::ServiceClient move_hand_cli_;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	ros::Publisher turn_angle;
	std::string action_name_;
	std_msgs::Float64 float64_msg;
	ros::Time clock;
	ros::Publisher pub_joint_commands_;
	ros::Subscriber imu_sub_;
	double yaw_from_imu;

public:
	PickUp(std::string name)
	:as_(nh_, name, false), action_name_(name){

		move_hand_cli_ = nh_.serviceClient<move_hand::move_hand>("move_hand");
		imu_sub_ = nh_.subscribe("/atlas/imu",100,&PickUp::imu_CB,this);

		while(!move_hand_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the move_hand server");
		}
		/*while(!getPath_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the /C31_GlobalPathPlanner/getPath server");
		}*/

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&PickUp::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&PickUp::preemptCB, this));

		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~PickUp(){}


	void imu_CB(const sensor_msgs::Imu::ConstPtr& imu){
		yaw_from_imu = atan2((2*(imu->orientation.w*imu->orientation.z + imu->orientation.y*imu->orientation.x)),
				(1-2*(pow(imu->orientation.y,2)+pow(imu->orientation.z,2))));
		return;
	}

	void turnToWaypoint(double angle){

	}

	void goalCB(){

		clock = ros::Time::now();
		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;
		/*
		C31_PathPlanner::C31_GetPath getpath;
		if(!getPath_cli_.call(getpath)){
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}*/

		//TODO: turn to waypoint

		move_hand::move_hand move_hand;


/*
		move_hand.request.PositionDestination.x = 0.01;
		move_hand.request.PositionDestination.y = 0.0;
		move_hand.request.PositionDestination.z = -0.0;
		move_hand.request.AngleDestination.x = 0.0;
		move_hand.request.AngleDestination.y = 0.0;
		move_hand.request.AngleDestination.z = 0.0;
		move_hand.request.LinkToMove = "l_arm";
		ROS_INFO("Moving left arm up");
		if(!move_hand_cli_.call(move_hand)){
			ROS_ERROR("Could not move left arm up");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}

		move_hand.request.PositionDestination.x = -0.01;
		move_hand.request.PositionDestination.y = 0.0;
		move_hand.request.PositionDestination.z = 0.00;
		move_hand.request.AngleDestination.x = 0.0;
		move_hand.request.AngleDestination.y = 0.0;
		move_hand.request.AngleDestination.z = 0.0;
		move_hand.request.LinkToMove = "l_arm";
		ROS_INFO("Moving left arm forward");
		if(!move_hand_cli_.call(move_hand)){
			ROS_ERROR("Could not move left arm forward");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}

		move_hand.request.PositionDestination.x = 0.01;
		move_hand.request.PositionDestination.y = 0.0;
		move_hand.request.PositionDestination.z = -0.0;
		move_hand.request.AngleDestination.x = 0.0;
		move_hand.request.AngleDestination.y = 0.0;
		move_hand.request.AngleDestination.z = 0.0;
		move_hand.request.LinkToMove = "r_arm";
		ROS_INFO("Moving right arm up");
		if(!move_hand_cli_.call(move_hand)){
			ROS_ERROR("Could not move left arm up");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}

		move_hand.request.PositionDestination.x = 0.01;
		move_hand.request.PositionDestination.y = -0.0;
		move_hand.request.PositionDestination.z = 0.08;
		move_hand.request.AngleDestination.x = 0.0;
		move_hand.request.AngleDestination.y = 0.0;
		move_hand.request.AngleDestination.z = 0.0;
		move_hand.request.LinkToMove = "r_arm";
		ROS_INFO("Moving right arm forward");
		if(!move_hand_cli_.call(move_hand)){
			ROS_ERROR("Could not move left arm forward");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}
*/
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

	void preemptCB()
	{
		ROS_ERROR("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C66_PickUp");

	PickUp* QuasiStaticWalker = new PickUp("PickUp");
	ROS_INFO("Running PickUp action");
	ros::spin();

	return 0;
}
