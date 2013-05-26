#include <ros/ros.h>
#include <ros/common.h>
#include <move_pelvis/move_pelvis.h>
#include <geometry_msgs/Point.h>
#include "PedalManipulation/PedalsCalibration.h"
#include "PedalManipulation/MovePedals.h"

class pedals_manipulation
{
private:
	ros::NodeHandle n;
	ros::ServiceServer calibrate_service;
	ros::ServiceServer move_right_pedal_service;
	ros::ServiceServer move_left_pedal_service;
	ros::ServiceClient walk_legs_by_pelvis_client;
	geometry_msgs::Point right_pedal_location;
	geometry_msgs::Point left_pedal_location;

public:
	pedals_manipulation(){
		calibrate_service = n.advertiseService("/PedalsManipulation/calibrate", &pedals_manipulation::calibrate, this);
		move_right_pedal_service = n.advertiseService("/PedalsManipulation/move_right_pedal", &pedals_manipulation::move_right_pedal, this);
		move_left_pedal_service = n.advertiseService("/PedalsManipulation/move_left_pedal", &pedals_manipulation::move_left_pedal, this);
		walk_legs_by_pelvis_client = n.serviceClient<move_pelvis::move_pelvis>("/walk_legs_by_pelvis");

		while(!walk_legs_by_pelvis_client.exists()){
			ROS_INFO("Waiting for %s service", walk_legs_by_pelvis_client.getService().c_str());
			ros::Duration(0.1).sleep();
		}

		ROS_INFO("Running PedalsManipulation services.");
	}

	bool calibrate(PedalManipulation::PedalsCalibration::Request &req, PedalManipulation::PedalsCalibration::Response &res){
		this->right_pedal_location.x = req.right_pedal_location.x;
		this->right_pedal_location.y = req.right_pedal_location.y;
		this->right_pedal_location.z = req.right_pedal_location.z;
		this->left_pedal_location.x = req.left_pedal_location.x;
		this->left_pedal_location.y = req.left_pedal_location.y;
		this->left_pedal_location.z = req.left_pedal_location.z;
		return true;
	}

	bool move_right_pedal(PedalManipulation::MovePedals::Request &req, PedalManipulation::MovePedals::Response &res){
		move_pelvis::move_pelvis move;
		move.request.LinkToMove = "r_leg";
		move.request.PositionDestination.x = right_pedal_location.x + 0.05 * req.percentage;
		move.request.PositionDestination.y = right_pedal_location.y;
		move.request.PositionDestination.z = right_pedal_location.z - 0.05 * req.percentage;
		move.request.AngleDestination.x = 0;
		move.request.AngleDestination.y = 0.2;
		move.request.AngleDestination.z = 0;
		if(walk_legs_by_pelvis_client.call(move)){
			res.success = true;
			return true;
		}
		return false;
	}

	bool move_left_pedal(PedalManipulation::MovePedals::Request &req, PedalManipulation::MovePedals::Response &res){
		move_pelvis::move_pelvis move;
		move.request.LinkToMove = "l_leg";
		move.request.PositionDestination.x = left_pedal_location.x + 0.05 * req.percentage;
		move.request.PositionDestination.y = left_pedal_location.y;
		move.request.PositionDestination.z = left_pedal_location.z - 0.05 * req.percentage;
		move.request.AngleDestination.x = 0;
		move.request.AngleDestination.y = 0.2;
		move.request.AngleDestination.z = 0;
		if(walk_legs_by_pelvis_client.call(move)){
			res.success = true;
			return true;
		}
		return false;
	}


};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pedals_manipulation");
	pedals_manipulation *s = new pedals_manipulation();
	ros::spin();

	return 0;
}
