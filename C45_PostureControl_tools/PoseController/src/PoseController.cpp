#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <std_msgs/Float64.h>
#include <boost/thread.hpp>
#include <PoseController/back_movement.h>
#include <PoseController/foot_movement.h>
#include <PoseController/hand_movement.h>

class PoseControllerClass{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle *rosnode;
	std::string action_name_;
	std::map <std::string, int> joints;
	std::vector<double> joint_positions, wanted_positions;
	ros::Publisher pub_joint_commands_;
	ros::Subscriber joint_states_sub_;
	ros::Subscriber hand_service, foot_service, back_service;
public:
	PoseControllerClass(std::string name):action_name_(name){

		rosnode = new ros::NodeHandle();
		wanted_positions.resize(28);
		joint_positions.resize(28);
		hand_service = nh_.subscribe("/PoseController/hand_movement", 1000, &PoseControllerClass::hand_movement, this);
		foot_service = nh_.subscribe("/PoseController/foot_movement", 1000, &PoseControllerClass::foot_movement, this);
		back_service = nh_.subscribe("/PoseController/back_movement", 1000, &PoseControllerClass::back_movement, this);

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&PoseControllerClass::joint_states_CB,this);
		pub_joint_commands_ = nh2_.advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);

		//this->ControlPose();
		ROS_INFO("Running pose controller");
	}

	~PoseControllerClass(){}

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		joint_positions = state->position;
	}

	void hand_movement(const PoseController::hand_movement::ConstPtr &req){
		ROS_INFO("Got hand movement");
		wanted_positions[joints["l_arm_usy"]] = joint_positions[joints["l_arm_usy"]] + req->l_arm_usy;
		wanted_positions[joints["l_arm_shx"]] = joint_positions[joints["l_arm_shx"]] + req->l_arm_shx;
		wanted_positions[joints["l_arm_ely"]] = joint_positions[joints["l_arm_ely"]] + req->l_arm_ely;
		wanted_positions[joints["l_arm_elx"]] = joint_positions[joints["l_arm_elx"]] + req->l_arm_elx;
		wanted_positions[joints["l_arm_uwy"]] = joint_positions[joints["l_arm_uwy"]] + req->l_arm_uwy;
		wanted_positions[joints["l_arm_mwx"]] = joint_positions[joints["l_arm_mwx"]] + req->l_arm_mwx;
		wanted_positions[joints["r_arm_usy"]] = joint_positions[joints["r_arm_usy"]] + req->r_arm_usy;
		wanted_positions[joints["r_arm_shx"]] = joint_positions[joints["r_arm_shx"]] + req->r_arm_shx;
		wanted_positions[joints["r_arm_ely"]] = joint_positions[joints["r_arm_ely"]] + req->r_arm_ely;
		wanted_positions[joints["r_arm_elx"]] = joint_positions[joints["r_arm_elx"]] + req->r_arm_elx;
		wanted_positions[joints["r_arm_uwy"]] = joint_positions[joints["r_arm_uwy"]] + req->r_arm_uwy;
		wanted_positions[joints["r_arm_mwx"]] = joint_positions[joints["r_arm_mwx"]] + req->r_arm_mwx;
		this->ControlPose();
	}

	void foot_movement(const PoseController::foot_movement::ConstPtr &req){
		ROS_INFO("Got foot movement");
		wanted_positions[joints["l_leg_uhz"]] = joint_positions[joints["l_leg_uhz"]] + req->l_leg_uhz;
		wanted_positions[joints["l_leg_mhx"]] = joint_positions[joints["l_leg_mhx"]] + req->l_leg_mhx;
		wanted_positions[joints["l_leg_lhy"]] = joint_positions[joints["l_leg_lhy"]] + req->l_leg_lhy;
		wanted_positions[joints["l_leg_kny"]] = joint_positions[joints["l_leg_kny"]] + req->l_leg_kny;
		wanted_positions[joints["l_leg_uay"]] = joint_positions[joints["l_leg_uay"]] + req->l_leg_uay;
		wanted_positions[joints["l_leg_lax"]] = joint_positions[joints["l_leg_lax"]] + req->l_leg_lax;
		wanted_positions[joints["r_leg_uhz"]] = joint_positions[joints["r_leg_uhz"]] + req->r_leg_uhz;
		wanted_positions[joints["r_leg_mhx"]] = joint_positions[joints["r_leg_mhx"]] + req->r_leg_mhx;
		wanted_positions[joints["r_leg_lhy"]] = joint_positions[joints["r_leg_lhy"]] + req->r_leg_lhy;
		wanted_positions[joints["r_leg_kny"]] = joint_positions[joints["r_leg_kny"]] + req->r_leg_kny;
		wanted_positions[joints["r_leg_uay"]] = joint_positions[joints["r_leg_uay"]] + req->r_leg_uay;
		wanted_positions[joints["r_leg_lax"]] = joint_positions[joints["r_leg_lax"]] + req->r_leg_lax;

		this->ControlPose();
	}

	void back_movement(const PoseController::back_movement::ConstPtr &req){
		ROS_INFO("Got back movement");
		wanted_positions[joints["back_lbz"]] = joint_positions[joints["back_lbz"]] + req->back_lbz;
		wanted_positions[joints["back_mby"]] = joint_positions[joints["back_mby"]] + req->back_mby;
		wanted_positions[joints["back_ubx"]] = joint_positions[joints["back_ubx"]] + req->back_ubx;

		this->ControlPose();
	}

	void ControlPose(){
		osrf_msgs::JointCommands jointcommands;
		jointcommands.name.push_back("atlas::back_lbz");
		jointcommands.name.push_back("atlas::back_mby");
		jointcommands.name.push_back("atlas::back_ubx");
		jointcommands.name.push_back("atlas::neck_ay");
		jointcommands.name.push_back("atlas::l_leg_uhz");
		jointcommands.name.push_back("atlas::l_leg_mhx");
		jointcommands.name.push_back("atlas::l_leg_lhy");
		jointcommands.name.push_back("atlas::l_leg_kny");
		jointcommands.name.push_back("atlas::l_leg_uay");
		jointcommands.name.push_back("atlas::l_leg_lax");
		jointcommands.name.push_back("atlas::r_leg_uhz");
		jointcommands.name.push_back("atlas::r_leg_mhx");
		jointcommands.name.push_back("atlas::r_leg_lhy");
		jointcommands.name.push_back("atlas::r_leg_kny");
		jointcommands.name.push_back("atlas::r_leg_uay");
		jointcommands.name.push_back("atlas::r_leg_lax");
		jointcommands.name.push_back("atlas::l_arm_usy");
		jointcommands.name.push_back("atlas::l_arm_shx");
		jointcommands.name.push_back("atlas::l_arm_ely");
		jointcommands.name.push_back("atlas::l_arm_elx");
		jointcommands.name.push_back("atlas::l_arm_uwy");
		jointcommands.name.push_back("atlas::l_arm_mwx");
		jointcommands.name.push_back("atlas::r_arm_usy");
		jointcommands.name.push_back("atlas::r_arm_shx");
		jointcommands.name.push_back("atlas::r_arm_ely");
		jointcommands.name.push_back("atlas::r_arm_elx");
		jointcommands.name.push_back("atlas::r_arm_uwy");
		jointcommands.name.push_back("atlas::r_arm_mwx");

		unsigned int n = jointcommands.name.size();
		jointcommands.position.resize(n);
		jointcommands.velocity.resize(n);
		jointcommands.effort.resize(n);
		jointcommands.kp_position.resize(n);
		jointcommands.ki_position.resize(n);
		jointcommands.kd_position.resize(n);
		jointcommands.kp_velocity.resize(n);
		jointcommands.i_effort_min.resize(n);
		jointcommands.i_effort_max.resize(n);
		for (unsigned int i = 0; i < n; i++)
		{
			std::vector<std::string> pieces;
			boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
					jointcommands.kp_position[i]);

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
					jointcommands.ki_position[i]);

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
					jointcommands.kd_position[i]);

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
					jointcommands.i_effort_min[i]);
			jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
					jointcommands.i_effort_max[i]);

			jointcommands.velocity[i]     = 0;
			jointcommands.effort[i]       = 0;
			jointcommands.kp_velocity[i]  = 0;
		}


		jointcommands.position[joints["l_leg_uhz"]] = wanted_positions[joints["l_leg_uhz"]];
		jointcommands.position[joints["l_leg_mhx"]] = wanted_positions[joints["l_leg_mhx"]];
		jointcommands.position[joints["l_leg_lhy"]] = wanted_positions[joints["l_leg_lhy"]];
		jointcommands.position[joints["l_leg_kny"]] = wanted_positions[joints["l_leg_kny"]];
		jointcommands.position[joints["l_leg_uay"]] = wanted_positions[joints["l_leg_uay"]];
		jointcommands.position[joints["l_leg_lax"]] = wanted_positions[joints["l_leg_lax"]];
		jointcommands.position[joints["r_leg_uhz"]] = wanted_positions[joints["r_leg_uhz"]];
		jointcommands.position[joints["r_leg_mhx"]] = wanted_positions[joints["r_leg_mhx"]];
		jointcommands.position[joints["r_leg_lhy"]] = wanted_positions[joints["r_leg_lhy"]];
		jointcommands.position[joints["r_leg_kny"]] = wanted_positions[joints["r_leg_kny"]];
		jointcommands.position[joints["r_leg_uay"]] = wanted_positions[joints["r_leg_uay"]];
		jointcommands.position[joints["r_leg_lax"]] = wanted_positions[joints["r_leg_lax"]];
		jointcommands.position[joints["l_arm_usy"]] = wanted_positions[joints["l_arm_usy"]];
		jointcommands.position[joints["l_arm_shx"]] = wanted_positions[joints["l_arm_shx"]];
		jointcommands.position[joints["l_arm_ely"]] = wanted_positions[joints["l_arm_ely"]];
		jointcommands.position[joints["l_arm_elx"]] = wanted_positions[joints["l_arm_elx"]];
		jointcommands.position[joints["l_arm_uwy"]] = wanted_positions[joints["l_arm_uwy"]];
		jointcommands.position[joints["l_arm_mwx"]] = wanted_positions[joints["l_arm_mwx"]];
		jointcommands.position[joints["r_arm_usy"]] = wanted_positions[joints["r_arm_usy"]];
		jointcommands.position[joints["r_arm_shx"]] = wanted_positions[joints["r_arm_shx"]];
		jointcommands.position[joints["r_arm_ely"]] = wanted_positions[joints["r_arm_ely"]];
		jointcommands.position[joints["r_arm_elx"]] = wanted_positions[joints["r_arm_elx"]];
		jointcommands.position[joints["r_arm_uwy"]] = wanted_positions[joints["r_arm_uwy"]];
		jointcommands.position[joints["r_arm_mwx"]] = wanted_positions[joints["r_arm_mwx"]];
		jointcommands.position[joints["neck_ay"]] = wanted_positions[joints["neck_ay"]];
		jointcommands.position[joints["back_lbz"]] = wanted_positions[joints["back_lbz"]];
		jointcommands.position[joints["back_mby"]] = wanted_positions[joints["back_mby"]];
		jointcommands.position[joints["back_ubx"]] = wanted_positions[joints["back_ubx"]];

		wanted_positions[joints["l_leg_uhz"]] = joint_positions[joints["l_leg_uhz"]];
		wanted_positions[joints["l_leg_mhx"]] = joint_positions[joints["l_leg_mhx"]];
		wanted_positions[joints["l_leg_lhy"]] = joint_positions[joints["l_leg_lhy"]];
		wanted_positions[joints["l_leg_kny"]] = joint_positions[joints["l_leg_kny"]];
		wanted_positions[joints["l_leg_uay"]] = joint_positions[joints["l_leg_uay"]];
		wanted_positions[joints["l_leg_lax"]] = joint_positions[joints["l_leg_lax"]];
		wanted_positions[joints["r_leg_uhz"]] = joint_positions[joints["r_leg_uhz"]];
		wanted_positions[joints["r_leg_mhx"]] = joint_positions[joints["r_leg_mhx"]];
		wanted_positions[joints["r_leg_lhy"]] = joint_positions[joints["r_leg_lhy"]];
		wanted_positions[joints["r_leg_kny"]] = joint_positions[joints["r_leg_kny"]];
		wanted_positions[joints["r_leg_uay"]] = joint_positions[joints["r_leg_uay"]];
		wanted_positions[joints["r_leg_lax"]] = joint_positions[joints["r_leg_lax"]];
		wanted_positions[joints["l_arm_usy"]] = joint_positions[joints["l_arm_usy"]];
		wanted_positions[joints["l_arm_shx"]] = joint_positions[joints["l_arm_shx"]];
		wanted_positions[joints["l_arm_ely"]] = joint_positions[joints["l_arm_ely"]];
		wanted_positions[joints["l_arm_elx"]] = joint_positions[joints["l_arm_elx"]];
		wanted_positions[joints["l_arm_uwy"]] = joint_positions[joints["l_arm_uwy"]];
		wanted_positions[joints["l_arm_mwx"]] = joint_positions[joints["l_arm_mwx"]];
		wanted_positions[joints["r_arm_usy"]] = joint_positions[joints["r_arm_usy"]];
		wanted_positions[joints["r_arm_shx"]] = joint_positions[joints["r_arm_shx"]];
		wanted_positions[joints["r_arm_ely"]] = joint_positions[joints["r_arm_ely"]];
		wanted_positions[joints["r_arm_elx"]] = joint_positions[joints["r_arm_elx"]];
		wanted_positions[joints["r_arm_uwy"]] = joint_positions[joints["r_arm_uwy"]];
		wanted_positions[joints["r_arm_mwx"]] = joint_positions[joints["r_arm_mwx"]];
		wanted_positions[joints["neck_ay"]] = joint_positions[joints["neck_ay"]];
		wanted_positions[joints["back_lbz"]] = joint_positions[joints["back_lbz"]];
		wanted_positions[joints["back_mby"]] = joint_positions[joints["back_mby"]];
		wanted_positions[joints["back_ubx"]] = joint_positions[joints["back_ubx"]];

		pub_joint_commands_.publish(jointcommands);
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "C45_PostureControl_PoseController");

	PoseControllerClass* PoseControl = new PoseControllerClass("PoseController");
	ROS_INFO("Running PoseController service");
	ros::spin();

	return 0;
}
