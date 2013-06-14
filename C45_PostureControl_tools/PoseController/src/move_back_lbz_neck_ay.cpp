#include <ros/ros.h>
#include <atlas_msgs/AtlasCommand.h>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasSimInterfaceState.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>
#include <std_srvs/Empty.h>
#include <PoseController/back_movement.h>
#include <PoseController/neck_movement.h>
#include <PoseController/back_lbz_neck_ay.h>

class back_neck{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle *rosnode;
	std::string action_name_;
	std::map <std::string, int> joints;
	std::vector<double> joint_positions, wanted_positions, kp_positions;
	ros::Publisher pub_atlas_commands_,pub_joint_states_;
	ros::Subscriber joint_states_sub_, atlas_sim_int_state_sub_;
	ros::ServiceServer com_service;
	ros::ServiceClient back_service, neck_service;
	ros::ServiceClient backz_cli, start_posecontroller_cli_, stop_posecontroller_cli_;
	std::vector<double> positions;
	bool up, waitForJointStates;
public:
	back_neck(std::string name):action_name_(name),up(false){

		rosnode = new ros::NodeHandle();

		com_service = nh_.advertiseService("/PoseController/back_lbz_neck_ay", &back_neck::com_service_CB, this);

		back_service = nh2_.serviceClient<PoseController::back_movement>("/PoseController/delta_back_movement");
		neck_service = nh2_.serviceClient<PoseController::neck_movement>("/PoseController/neck_movement");

		start_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/start");
		stop_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/stop");

		pub_atlas_commands_ = nh2_.advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1, true);

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&back_neck::joint_states_CB,this);

	}

	~back_neck(){}

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		positions = state->position;
	}




	/////////////////////////////////////////////////////////////////////////////////////////////////////
	bool com_service_CB(PoseController::back_lbz_neck_ay::Request &req, PoseController::back_lbz_neck_ay::Response &res){

		std_srvs::Empty em;
		atlas_msgs::AtlasSimInterfaceCommand cm;


		//To switch to manipulate, change STAND to MANIPULATE.
		cm.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;

		//Publish STAND behavior
		pub_atlas_commands_.publish(cm);

		ros::Duration(0.1).sleep();

		//Send 'start' command to PoseController service
		start_posecontroller_cli_.call(em);


		//Change neck and back joints
		PoseController::neck_movement neck;
		neck.request.neck_ay = req.neck_ay;
		neck_service.call(neck);

		PoseController::back_movement b;
		double total_time = 1.0;
		int segments = 50;
		double velocity = (req.back_lbz - positions[joints["back_lbz"]])/total_time;
		for(int i = 0; i < segments; i++){
			ros::spinOnce();
			b.request.back_lbz = velocity/segments;
			back_service.call(b);
			ros::Duration(total_time/segments).sleep();
		}
		b.request.back_lbz = req.back_lbz;
		b.request.back_mby = -100; //Send 'Dont change' to back_mby joint
		b.request.back_ubx = -100; //Send 'Dont change' to back_ubx joint

		back_service.call(b);


		//Send 'stop' command to PoseController service
		stop_posecontroller_cli_.call(em);

		ros::Duration(0.1).sleep();


		//Switch behavior
		cm.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;

		//pub_atlas_commands_.publish(cm);

		res.success = true;
		return true;

	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "back_neck");

	back_neck* PoseControl = new back_neck("back_neck");
	ROS_INFO("Running back_neck service");
	ros::spin();

	return 0;
}
