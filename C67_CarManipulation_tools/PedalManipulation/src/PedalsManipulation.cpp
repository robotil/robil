#include <ros/ros.h>
#include <ros/common.h>
#include <move_pelvis/move_pelvis.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <PoseController/foot_movement.h>
#include <atlas_msgs/AtlasState.h>
#include "PedalManipulation/PedalsCalibration.h"
#include "PedalManipulation/MovePedals.h"

class pedals_manipulation
{
private:
	ros::NodeHandle n, n2;
	ros::ServiceServer calibrate_service;
	ros::ServiceServer start_service;
	ros::ServiceServer stop_service;
	ros::ServiceClient walk_legs_by_pelvis_client;
	ros::ServiceClient posecontroller_foot_client;
	ros::Subscriber move_right_pedal_subcriber;
	ros::Subscriber move_left_pedal_subcriber;
	geometry_msgs::Point right_pedal_location;
	geometry_msgs::Point left_pedal_location;
	double wanted_right_pedal_location;
	double wanted_left_pedal_location;
	double base_r_leg_kny, base_r_leg_uay;
	double base_l_leg_kny, base_l_leg_uay;
	ros::Subscriber joint_states_sub_;
	std::vector<double> positions;
	bool running;

public:
	pedals_manipulation():running(false){
		calibrate_service = n.advertiseService("/PedalsManipulation/calibrate", &pedals_manipulation::calibrate, this);

		start_service = n.advertiseService("/PedalsManipulation/start", &pedals_manipulation::start, this);
		stop_service = n.advertiseService("/PedalsManipulation/stop", &pedals_manipulation::stop, this);

		move_right_pedal_subcriber = n.subscribe("/PedalsManipulation/move_right_pedal", 1, &pedals_manipulation::move_right_pedal, this);
		move_left_pedal_subcriber = n.subscribe("/PedalsManipulation/move_left_pedal", 1, &pedals_manipulation::move_left_pedal, this);

		joint_states_sub_ = n2.subscribe("/PoseController/joint_states", 100, &pedals_manipulation::joint_states_CB,this);

		walk_legs_by_pelvis_client = n.serviceClient<move_pelvis::move_pelvis>("/move_leg_by_pelvis");
		posecontroller_foot_client = n.serviceClient<PoseController::foot_movement>("/PoseController/delta_foot_movement");

		while(!walk_legs_by_pelvis_client.exists()){
			ROS_INFO("Waiting for %s service", walk_legs_by_pelvis_client.getService().c_str());
			ros::Duration(0.1).sleep();
		}

		ROS_INFO("Running PedalsManipulation services.");
		move_pedals();
	}

	void move_pedals(){
		double r_dist, l_dist;
		while(ros::ok()){
			ros::Duration(0.01).sleep();
			ros::spinOnce();
			if(!running){
				continue;
			}
			PoseController::foot_movement r_foot, l_foot;
			r_dist = positions[atlas_msgs::AtlasState::r_leg_kny] + wanted_right_pedal_location * (0.13) - (base_r_leg_kny);
			//ROS_INFO("r_leg_kny = %f, r_dist = %f", positions[atlas_msgs::AtlasState::r_leg_kny], r_dist);
			if(fabs(r_dist) > 0.00199){
				r_foot.request.r_leg_kny = (r_dist < 0) ? 0.001 : -0.001;
				r_foot.request.r_leg_uay = (r_dist < 0) ? -0.001 : 0.001;
				posecontroller_foot_client.call(r_foot);
			}
			l_dist = positions[atlas_msgs::AtlasState::l_leg_kny] + wanted_left_pedal_location * (0.13) - (base_l_leg_kny);
			//ROS_INFO("l_leg_kny = %f, l_dist = %f", positions[atlas_msgs::AtlasState::l_leg_kny], l_dist);
			if(fabs(l_dist) > 0.00199){
				l_foot.request.l_leg_kny = (l_dist < 0) ? 0.001 : -0.001;
				l_foot.request.l_leg_uay = (l_dist < 0) ? -0.001 : 0.001;
				posecontroller_foot_client.call(l_foot);
			}
		}
	}

	bool calibrate(PedalManipulation::PedalsCalibration::Request &req, PedalManipulation::PedalsCalibration::Response &res){
		this->right_pedal_location.x = req.right_pedal_location.x;
		this->right_pedal_location.y = req.right_pedal_location.y;
		this->right_pedal_location.z = req.right_pedal_location.z;
		this->left_pedal_location.x = req.left_pedal_location.x;
		this->left_pedal_location.y = req.left_pedal_location.y;
		this->left_pedal_location.z = req.left_pedal_location.z;

		move_pelvis::move_pelvis move, move2;
		move.request.LinkToMove = "r_leg";
		move.request.PositionDestination.x = right_pedal_location.x;
		move.request.PositionDestination.y = right_pedal_location.y;
		move.request.PositionDestination.z = right_pedal_location.z;
		move.request.AngleDestination.x = 0.0;
		move.request.AngleDestination.y = 0.0;
		move.request.AngleDestination.z = 0.0;
		if(!walk_legs_by_pelvis_client.call(move)){
			res.success = false;
			return false;
		}

		ROS_INFO("Moving left leg to (%f ,%f ,%f)", left_pedal_location.x, left_pedal_location.y, left_pedal_location.z);
		move2.request.LinkToMove = "l_leg";
		move2.request.PositionDestination.x = left_pedal_location.x;
		move2.request.PositionDestination.y = left_pedal_location.y;
		move2.request.PositionDestination.z = left_pedal_location.z;
		move2.request.AngleDestination.x = 0.0;
		move2.request.AngleDestination.y = 0.0;
		move2.request.AngleDestination.z = 0.0;
		if(!walk_legs_by_pelvis_client.call(move2)){
			res.success = false;
			return false;
		}
		res.success = true;
		return true;
	}

	void move_right_pedal(const std_msgs::Float64ConstPtr& percentage){
		wanted_right_pedal_location = percentage->data / 100.0;
	}

	void move_left_pedal(const std_msgs::Float64ConstPtr& percentage){
		wanted_left_pedal_location = percentage->data / 100.0;
	}

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		positions = state->position;
	}

	bool start(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		base_r_leg_kny = positions[atlas_msgs::AtlasState::r_leg_kny];
		base_r_leg_uay = positions[atlas_msgs::AtlasState::r_leg_uay];
		base_l_leg_kny = positions[atlas_msgs::AtlasState::l_leg_kny];
		base_l_leg_uay = positions[atlas_msgs::AtlasState::l_leg_uay];
		running = true;
		return true;
	}

	bool stop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
		running = false;
		return true;
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pedals_manipulation");
	pedals_manipulation *s = new pedals_manipulation();
	ros::spin();

	return 0;
}
