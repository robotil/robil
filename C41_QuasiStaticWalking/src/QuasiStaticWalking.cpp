#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <C31_PathPlanner/C31_GetPath.h>
#include <C31_PathPlanner/C31_Waypoints.h>
#include <move_pelvis/move_pelvis.h>
#include <pelvis_leg_target/pelvis_leg_target.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <osrf_msgs/JointCommands.h>
#include <tf/transform_listener.h>
#include <FootPlacement/FootPlacement.h>



class QuasiStaticWalking{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle* rosnode;
	ros::ServiceClient move_pelvis_cli_, pelvis_leg_target_cli_, getPath_cli_, walk_legs_cli_, step_down_cli_, make_step_cli_, foot_placement_cli_;
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
	tf::TransformListener listener;

public:
	enum LegEnum{
		RIGHT,
		LEFT
	};

	struct Walk	{
		LegEnum leg;
		double start_pelvis_yaw;
		geometry_msgs::Point position;
		geometry_msgs::Point orientation;
		double end_pelvis_yaw;
	};
	QuasiStaticWalking(std::string name)
	:as_(nh_, name, false), action_name_(name){

		move_pelvis_cli_ = nh_.serviceClient<move_pelvis::move_pelvis>("move_pelvis");
		walk_legs_cli_ = nh_.serviceClient<move_pelvis::move_pelvis>("walk_legs");
		step_down_cli_ = nh_.serviceClient<std_srvs::Empty>("step_down");
		pelvis_leg_target_cli_ = nh_.serviceClient<pelvis_leg_target::pelvis_leg_target>("pelvis_leg_target");
		//foot_placement_cli_ = nh_.serviceClient<FootPlacement::FootPlacement>("foot_placement");
		//getPath_cli_ = nh_.serviceClient<C31_PathPlanner::C31_GetPath>("/getPath");
		imu_sub_ = nh_.subscribe("/atlas/imu",100,&QuasiStaticWalking::imu_CB,this);

		while(!move_pelvis_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the move_pelvis server");
		}

		while(!step_down_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the step_down server");
		}

		while(!walk_legs_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the walk_legs server");
		}

		while(!pelvis_leg_target_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the pelvis_leg_target server");
		}

		/*while(!foot_placement_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the foot_placement server");
		}*/

		make_step_cli_ = nh_.serviceClient<move_pelvis::move_pelvis>("make_step");
		/*while(!getPath_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the /getPath server");
		}*/

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&QuasiStaticWalking::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&QuasiStaticWalking::preemptCB, this));

		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}

	~QuasiStaticWalking(){}


	void imu_CB(const sensor_msgs::Imu::ConstPtr& imu){
		yaw_from_imu = atan2((2*(imu->orientation.w*imu->orientation.z + imu->orientation.y*imu->orientation.x)),
								(1-2*(pow(imu->orientation.y,2)+pow(imu->orientation.z,2))));
		return;
	}

	void turnToWaypoint(double angle){

	}

	void walkForward(std::string leg, double step_size){

	}

	void goalCB(){

/*
		clock = ros::Time::now();
		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;


		//TODO: turn to waypoint

		pelvis_leg_target::pelvis_leg_target leg_target;
		move_pelvis::move_pelvis move_pelvis;

		move_pelvis.request.PositionDestination.x = 0;
		move_pelvis.request.PositionDestination.y = -0.0;
		move_pelvis.request.PositionDestination.z = -0.05;
		move_pelvis.request.AngleDestination.x = 0.0;
		move_pelvis.request.AngleDestination.y = 0.0;
		move_pelvis.request.AngleDestination.z = 0.0;
		move_pelvis.request.LinkToMove = "pelvis";
		ROS_INFO("Lowering pelvis");
		if(!move_pelvis_cli_.call(move_pelvis)){
			ROS_ERROR("Could not lower pelvis");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}

		//Move over to static leg
		leg_target.request.leg = "l_foot";
		ROS_INFO("Moving to %s leg", leg_target.request.leg.c_str());
		if(!pelvis_leg_target_cli_.call(leg_target)){
			ROS_ERROR("Could not move to %s leg", leg_target.request.leg.c_str());
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}


		while(ros::ok()){
			ROS_INFO("Next step");

			ROS_INFO("Getting next step");
			C31_PathPlanner::C31_GetPath path;

			if(!getPath_cli_.call(path)){
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setAborted(_res);
				ROS_INFO("End time: %f", ros::Time::now().toSec());
				return;
			}

			FootPlacement::FootPlacement possible_coords;
			FootPlacement::Pos pos;

			possible_coords.request.dirX = path.response.path.points[0].x;
			possible_coords.request.dirY = path.response.path.points[0].y;
			possible_coords.request.foot = (leg_target.request.leg == "r_foot") ? possible_coords.request.LEFT : possible_coords.request.RIGHT;
			possible_coords.request.useC22 = 0;

			if (!foot_placement_cli_.call(possible_coords)) {

				ROS_INFO("Can't contact foot_placement");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setAborted(_res);
				ROS_INFO("End time: %f", ros::Time::now().toSec());
				return;
			}
			for(unsigned int i = 0; i < possible_coords.response.positions.size(); i++){
				if(sqrt(
						pow(possible_coords.response.positions[i].point.x, 2)
						+ pow(possible_coords.response.positions[i].point.y, 2)
					) < 0.18){
					pos = possible_coords.response.positions[0];
				}
			}
			if(pos.cost == 0.0){
				ROS_INFO("Can't find suitable step");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setAborted(_res);
				ROS_INFO("End time: %f", ros::Time::now().toSec());
				return;
			}

			tf::StampedTransform transform;
			try {
				listener.waitForTransform("/pelvis",(leg_target.request.leg == "l_foot") ? "r_foot" : "l_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/pelvis",(leg_target.request.leg == "l_foot") ? "r_foot" : "l_foot",ros::Time(0),transform);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}

			//Change pelvis orientation
			move_pelvis.request.PositionDestination.x = 0;
			move_pelvis.request.PositionDestination.y = 0;
			move_pelvis.request.PositionDestination.z = 0;
			move_pelvis.request.AngleDestination.x = 0;
			move_pelvis.request.AngleDestination.y = 0;
			move_pelvis.request.AngleDestination.z = 0;
			move_pelvis.request.LinkToMove = "pelvis";
			ROS_INFO("Changing pelvis yaw orientation");
			if(!move_pelvis_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move change pelvis yaw orientation");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Move leg forward
			move_pelvis.request.PositionDestination.x = pos.point.x;
			move_pelvis.request.PositionDestination.y = pos.point.y;
			move_pelvis.request.PositionDestination.z = pos.point.z;
			move_pelvis.request.AngleDestination.x = 0;
			move_pelvis.request.AngleDestination.y = 0;
			move_pelvis.request.AngleDestination.z = (leg_target.request.leg == "l_foot") ? -0.3 : 0.3;
			move_pelvis.request.LinkToMove = (leg_target.request.leg == "l_foot") ? "r_leg" : "l_leg";
			ROS_INFO("Moving %s forward", move_pelvis.request.LinkToMove.c_str());
			if(!make_step_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move %s forward", move_pelvis.request.LinkToMove.c_str());
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Move over to dynamic leg
			leg_target.request.leg = (leg_target.request.leg == "l_foot") ? "r_foot" : "l_foot";
			ROS_INFO("Moving to %s leg", leg_target.request.leg.c_str());
			if(!pelvis_leg_target_cli_.call(leg_target)){
				ROS_ERROR("Could not move to %s leg", leg_target.request.leg.c_str());
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Change pelvis orientation
			move_pelvis.request.PositionDestination.x = 0;
			move_pelvis.request.PositionDestination.y = 0;
			move_pelvis.request.PositionDestination.z = 0;
			move_pelvis.request.AngleDestination.x = 0;
			move_pelvis.request.AngleDestination.y = 0;
			move_pelvis.request.AngleDestination.z = 0;
			move_pelvis.request.LinkToMove = "pelvis";
			ROS_INFO("Changing pelvis yaw orientation");
			if(!move_pelvis_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move change pelvis yaw orientation");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}







		}
*/



		std::vector<Walk> steps;
		Walk step;
		step.start_pelvis_yaw = 0.0;
		step.leg = RIGHT;
		step.position.x = 0.20;
		step.position.y = -0.268;
		step.position.z = -0.779;
		step.orientation.x = 0;
		step.orientation.y = 0;
		step.orientation.z = -0.8;
		step.end_pelvis_yaw = 0;
		steps.push_back(step);

		step.start_pelvis_yaw = -0.0;
		step.leg = LEFT;
		step.position.x = 0.18;
		step.position.y = 0.268;
		step.position.z = -0.779;
		step.orientation.x = 0;
		step.orientation.y = 0;
		step.orientation.z = 0.8;
		step.end_pelvis_yaw = -0.0;
		steps.push_back(step);



		clock = ros::Time::now();
		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;

		//TODO: turn to waypoint

		pelvis_leg_target::pelvis_leg_target leg_target;
		move_pelvis::move_pelvis move_pelvis;

		move_pelvis.request.PositionDestination.x = 0;
		move_pelvis.request.PositionDestination.y = -0.0;
		move_pelvis.request.PositionDestination.z = -0.05;
		move_pelvis.request.AngleDestination.x = 0.0;
		move_pelvis.request.AngleDestination.y = 0.0;
		move_pelvis.request.AngleDestination.z = 0.0;
		move_pelvis.request.LinkToMove = "pelvis";
		ROS_INFO("Lowering pelvis");
		if(!move_pelvis_cli_.call(move_pelvis)){
			ROS_ERROR("Could not lower pelvis");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}

		//Move over to static leg
		leg_target.request.leg = (steps[0].leg == RIGHT) ? "l_foot" : "r_foot";
		ROS_INFO("Moving to %s leg", leg_target.request.leg.c_str());
		if(!pelvis_leg_target_cli_.call(leg_target)){
			ROS_ERROR("Could not move to %s leg", leg_target.request.leg.c_str());
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setSucceeded(_res);
			return;
		}


		for(unsigned int i = 0; i < steps.size(); i++){
			ROS_INFO("Running iteration #%d", i+1);

			//Change pelvis orientation
			move_pelvis.request.PositionDestination.x = 0;
			move_pelvis.request.PositionDestination.y = 0;
			move_pelvis.request.PositionDestination.z = 0;
			move_pelvis.request.AngleDestination.x = 0;
			move_pelvis.request.AngleDestination.y = 0;
			move_pelvis.request.AngleDestination.z = steps[i].start_pelvis_yaw;
			move_pelvis.request.LinkToMove = "pelvis";
			ROS_INFO("Changing pelvis yaw orientation");
			if(!move_pelvis_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move change pelvis yaw orientation");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Move leg forward
			move_pelvis.request.PositionDestination.x = steps[i].position.x;
			move_pelvis.request.PositionDestination.y = steps[i].position.y;
			move_pelvis.request.PositionDestination.z = steps[i].position.z;
			move_pelvis.request.AngleDestination.x = steps[i].orientation.x;
			move_pelvis.request.AngleDestination.y = steps[i].orientation.y;
			move_pelvis.request.AngleDestination.z = steps[i].orientation.z;
			move_pelvis.request.LinkToMove = (steps[i].leg == LEFT) ? "l_leg" : "r_leg";
			ROS_INFO("Moving %s forward", move_pelvis.request.LinkToMove.c_str());
			if(!make_step_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move %s forward", move_pelvis.request.LinkToMove.c_str());
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Move over to dynamic leg
			leg_target.request.leg = (steps[i].leg == LEFT) ? "l_foot" : "r_foot";
			ROS_INFO("Moving to %s leg", leg_target.request.leg.c_str());
			if(!pelvis_leg_target_cli_.call(leg_target)){
				ROS_ERROR("Could not move to %s leg", leg_target.request.leg.c_str());
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}

			//Change pelvis orientation
			move_pelvis.request.PositionDestination.x = 0;
			move_pelvis.request.PositionDestination.y = 0;
			move_pelvis.request.PositionDestination.z = 0;
			move_pelvis.request.AngleDestination.x = 0;
			move_pelvis.request.AngleDestination.y = 0;
			move_pelvis.request.AngleDestination.z = steps[i].end_pelvis_yaw;
			move_pelvis.request.LinkToMove = "pelvis";
			ROS_INFO("Changing pelvis yaw orientation");
			if(!move_pelvis_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move change pelvis yaw orientation");
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}







		}






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


	double QuatToRoll(double x, double y, double z, double w){
		return atan2(2*(w*x + y*z), 1 - 2*(pow(x,2) + pow(y,2)));
	}
	double QuatToRoll(const tf::Quaternion &quat){
		return atan2(2*(quat.w()*quat.x() + quat.y()*quat.z()), 1 - 2*(pow(quat.x(),2) + pow(quat.y(),2)));
	}
	double QuatToRoll(const geometry_msgs::Quaternion &quat){
		return atan2(2*(quat.w*quat.x + quat.y*quat.z), 1 - 2*(pow(quat.x,2) + pow(quat.y,2)));
	}


	double QuatToPitch(double x, double y, double z, double w){
		return asin(2*(w*y - z*x));
	}
	double QuatToPitch(const tf::Quaternion &quat){
		return asin(2*(quat.w()*quat.y() - quat.z()*quat.x()));
	}
	double QuatToPitch(const geometry_msgs::Quaternion &quat){
		return asin(2*(quat.w*quat.y - quat.z*quat.x));
	}


	double QuatToYaw(double x, double y, double z, double w){
		return atan2(2*(w*z + x*y), 1 - 2*(pow(y,2) + pow(z,2)));
	}
	double QuatToYaw(const tf::Quaternion &quat){
		return atan2(2*(quat.w()*quat.z() + quat.x()*quat.y()), 1 - 2*(pow(quat.y(),2) + pow(quat.z(),2)));
	}
	double QuatToYaw(const geometry_msgs::Quaternion &quat){
		return atan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(pow(quat.y,2) + pow(quat.z,2)));
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C41_QuasiStaticWalking");

	QuasiStaticWalking* QuasiStaticWalker = new QuasiStaticWalking("QuasiStaticWalking");
	ROS_INFO("Running QuasiStaticWalking action");
	ros::spin();

	return 0;
}
