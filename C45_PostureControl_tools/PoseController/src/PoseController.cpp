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
#include <PoseController/foot_movement.h>
#include <PoseController/hand_movement.h>
#include <PoseController/neck_movement.h>

class PoseControllerClass{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle *rosnode;
	std::string action_name_;
	std::map <std::string, int> joints;
	std::vector<double> joint_positions, wanted_positions, kp_positions;
	ros::Publisher pub_atlas_commands_,pub_joint_states_;
	ros::Subscriber joint_states_sub_, atlas_sim_int_state_sub_;
	ros::ServiceServer hand_service, foot_service, back_service, neck_service, reset_service, start_service, stop_service;
	ros::ServiceServer delta_hand_service, delta_foot_service, delta_back_service, delta_neck_service;
	osrf_msgs::JointCommands jointcommands;
	atlas_msgs::AtlasCommand atlascommand;
	atlas_msgs::AtlasSimInterfaceState atlassimintstate;
	bool up, waitForJointStates;
public:
	PoseControllerClass(std::string name):action_name_(name),up(false){

		rosnode = new ros::NodeHandle();
		wanted_positions.resize(28);
		joint_positions.resize(28);
		kp_positions.resize(28);

		hand_service = nh_.advertiseService("/PoseController/hand_movement", &PoseControllerClass::hand_movement, this);
		delta_hand_service = nh_.advertiseService("/PoseController/delta_hand_movement", &PoseControllerClass::delta_hand_movement, this);

		foot_service = nh_.advertiseService("/PoseController/foot_movement", &PoseControllerClass::foot_movement, this);
		delta_foot_service = nh_.advertiseService("/PoseController/delta_foot_movement", &PoseControllerClass::delta_foot_movement, this);

		back_service = nh_.advertiseService("/PoseController/back_movement", &PoseControllerClass::back_movement, this);
		delta_back_service = nh_.advertiseService("/PoseController/delta_back_movement", &PoseControllerClass::delta_back_movement, this);

		neck_service = nh_.advertiseService("/PoseController/neck_movement", &PoseControllerClass::neck_movement, this);
		delta_neck_service = nh_.advertiseService("/PoseController/delta_neck_movement", &PoseControllerClass::delta_neck_movement, this);

		reset_service = nh_.advertiseService("/PoseController/reset_joints", &PoseControllerClass::reset_joints_CB, this);

		start_service = nh_.advertiseService("/PoseController/start", &PoseControllerClass::start_CB, this);
		stop_service = nh_.advertiseService("/PoseController/stop", &PoseControllerClass::stop_CB, this);

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&PoseControllerClass::joint_states_CB,this);
		pub_atlas_commands_ = nh2_.advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1, true);
		pub_joint_states_ = nh2_.advertise<sensor_msgs::JointState>("/PoseController/joint_states", 1, true);

		while(!waitForJointStates){
			ros::spinOnce();
			ros::Duration(0.05).sleep();
		}

		atlas_sim_int_state_sub_ = nh_.subscribe("/atlas/atlas_sim_interface_state", 100, &PoseControllerClass::atlas_sim_int_state_CB,this);

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
		atlascommand.position.resize(n);
		atlascommand.velocity.resize(n);
		atlascommand.effort.resize(n);
		atlascommand.kp_position.resize(n);
		atlascommand.ki_position.resize(n);
		atlascommand.kd_position.resize(n);
		atlascommand.kp_velocity.resize(n);
		atlascommand.i_effort_min.resize(n);
		atlascommand.i_effort_max.resize(n);
		atlascommand.k_effort.resize(n);
		for (unsigned int i = 0; i < n; i++)
		{
			std::vector<std::string> pieces;
			boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

			double f;
			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
					f);
			atlascommand.kp_position[i] = (float) f;

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
					f);
			atlascommand.ki_position[i] = (float) f;

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
					f);
			atlascommand.kd_position[i] = (float) f;

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
					f);
			atlascommand.i_effort_min[i] = (float) f;
			atlascommand.i_effort_min[i] = -atlascommand.i_effort_min[i];

			rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
					f);
			atlascommand.i_effort_max[i] = (float) f;

			atlascommand.velocity[i]     = 0;
			atlascommand.effort[i]       = 0;
			atlascommand.kp_velocity[i]  = 0;
			atlascommand.k_effort[i] 	 = 255;
			kp_positions[i] = atlascommand.kp_position[i];
		}

		std_srvs::Empty e;
		this->reset_joints_CB(e.request, e.response);

		this->ControlPose();
		ROS_INFO("Running pose controller");
	}

	~PoseControllerClass(){}

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		joint_positions = state->position;
		if(!waitForJointStates) {
			std_srvs::Empty e;
			this->reset_joints_CB(e.request, e.response);
			waitForJointStates = true;
		}
	}

	void atlas_sim_int_state_CB(const atlas_msgs::AtlasSimInterfaceStateConstPtr& state){
		atlassimintstate.desired_behavior = state->desired_behavior;
	}

	bool reset_joints_CB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
		ROS_INFO("Got reset joints");
		for(unsigned int i = 0; i < joint_positions.size(); i++){
			wanted_positions[i] = joint_positions[i];
		}
		return true;
	}

	bool start_CB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
		ROS_INFO("Starting PoseController");
		this->reset_joints_CB(req, res);
		this->up = true;
		return true;
	}

	bool stop_CB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
		ROS_INFO("Stopping PoseController");
		this->up = false;
		return true;
	}

	bool hand_movement(PoseController::hand_movement::Request &req, PoseController::hand_movement::Response &res){
		//ROS_INFO("Got hand movement");
		wanted_positions[atlas_msgs::AtlasState::l_arm_usy] = req.l_arm_usy;
		wanted_positions[atlas_msgs::AtlasState::l_arm_shx] = req.l_arm_shx;
		wanted_positions[atlas_msgs::AtlasState::l_arm_ely] = req.l_arm_ely;
		wanted_positions[atlas_msgs::AtlasState::l_arm_elx] = req.l_arm_elx;
		wanted_positions[atlas_msgs::AtlasState::l_arm_uwy] = req.l_arm_uwy;
		wanted_positions[atlas_msgs::AtlasState::l_arm_mwx] = req.l_arm_mwx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_usy] = req.r_arm_usy;
		wanted_positions[atlas_msgs::AtlasState::r_arm_shx] = req.r_arm_shx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_ely] = req.r_arm_ely;
		wanted_positions[atlas_msgs::AtlasState::r_arm_elx] = req.r_arm_elx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_uwy] = req.r_arm_uwy;
		wanted_positions[atlas_msgs::AtlasState::r_arm_mwx] = req.r_arm_mwx;
		return true;
	}

	bool foot_movement(PoseController::foot_movement::Request &req, PoseController::foot_movement::Response &res){
		//ROS_INFO("Got foot movement");
		wanted_positions[atlas_msgs::AtlasState::l_leg_uhz] = req.l_leg_uhz;
		wanted_positions[atlas_msgs::AtlasState::l_leg_mhx] = req.l_leg_mhx;
		wanted_positions[atlas_msgs::AtlasState::l_leg_lhy] = req.l_leg_lhy;
		wanted_positions[atlas_msgs::AtlasState::l_leg_kny] = req.l_leg_kny;
		wanted_positions[atlas_msgs::AtlasState::l_leg_uay] = req.l_leg_uay;
		wanted_positions[atlas_msgs::AtlasState::l_leg_lax] = req.l_leg_lax;
		wanted_positions[atlas_msgs::AtlasState::r_leg_uhz] = req.r_leg_uhz;
		wanted_positions[atlas_msgs::AtlasState::r_leg_mhx] = req.r_leg_mhx;
		wanted_positions[atlas_msgs::AtlasState::r_leg_lhy] = req.r_leg_lhy;
		wanted_positions[atlas_msgs::AtlasState::r_leg_kny] = req.r_leg_kny;
		wanted_positions[atlas_msgs::AtlasState::r_leg_uay] = req.r_leg_uay;
		wanted_positions[atlas_msgs::AtlasState::r_leg_lax] = req.r_leg_lax;
		if(req.l_leg_uhz_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_uhz] = req.l_leg_uhz_kp_position;
		if(req.l_leg_mhx_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_mhx] = req.l_leg_mhx_kp_position;
		if(req.l_leg_lhy_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_lhy] = req.l_leg_lhy_kp_position;
		if(req.l_leg_kny_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_kny] = req.l_leg_kny_kp_position;
		if(req.l_leg_uay_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_uay] = req.l_leg_uay_kp_position;
		if(req.l_leg_lax_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_lax] = req.l_leg_lax_kp_position;
		if(req.r_leg_uhz_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_uhz] = req.r_leg_uhz_kp_position;
		if(req.r_leg_mhx_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_mhx] = req.r_leg_mhx_kp_position;
		if(req.r_leg_lhy_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_lhy] = req.r_leg_lhy_kp_position;
		if(req.r_leg_kny_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_kny] = req.r_leg_kny_kp_position;
		if(req.r_leg_uay_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_uay] = req.r_leg_uay_kp_position;
		if(req.r_leg_lax_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_lax] = req.r_leg_lax_kp_position;
		return true;
	}

	bool back_movement(PoseController::back_movement::Request &req, PoseController::back_movement::Response &res){
		//ROS_INFO("Got back movement");
		if(req.back_lbz != -100) wanted_positions[atlas_msgs::AtlasState::back_lbz] = req.back_lbz;
		wanted_positions[atlas_msgs::AtlasState::back_mby] = req.back_mby;
		wanted_positions[atlas_msgs::AtlasState::back_ubx] = req.back_ubx;
		return true;
	}

	bool neck_movement(PoseController::neck_movement::Request &req, PoseController::neck_movement::Response &res){
		//ROS_INFO("Got neck movement");
		wanted_positions[atlas_msgs::AtlasState::neck_ay] = req.neck_ay;
		return true;
	}



	bool delta_hand_movement(PoseController::hand_movement::Request &req, PoseController::hand_movement::Response &res){
		//ROS_INFO("Got delta hand movement");
		wanted_positions[atlas_msgs::AtlasState::l_arm_usy] += req.l_arm_usy;
		wanted_positions[atlas_msgs::AtlasState::l_arm_shx] += req.l_arm_shx;
		wanted_positions[atlas_msgs::AtlasState::l_arm_ely] += req.l_arm_ely;
		wanted_positions[atlas_msgs::AtlasState::l_arm_elx] += req.l_arm_elx;
		wanted_positions[atlas_msgs::AtlasState::l_arm_uwy] += req.l_arm_uwy;
		wanted_positions[atlas_msgs::AtlasState::l_arm_mwx] += req.l_arm_mwx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_usy] += req.r_arm_usy;
		wanted_positions[atlas_msgs::AtlasState::r_arm_shx] += req.r_arm_shx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_ely] += req.r_arm_ely;
		wanted_positions[atlas_msgs::AtlasState::r_arm_elx] += req.r_arm_elx;
		wanted_positions[atlas_msgs::AtlasState::r_arm_uwy] += req.r_arm_uwy;
		wanted_positions[atlas_msgs::AtlasState::r_arm_mwx] += req.r_arm_mwx;
		return true;
	}

	bool delta_foot_movement(PoseController::foot_movement::Request &req, PoseController::foot_movement::Response &res){
		//ROS_INFO("Got delta foot movement");
		wanted_positions[atlas_msgs::AtlasState::l_leg_uhz] += req.l_leg_uhz;
		wanted_positions[atlas_msgs::AtlasState::l_leg_mhx] += req.l_leg_mhx;
		wanted_positions[atlas_msgs::AtlasState::l_leg_lhy] += req.l_leg_lhy;
		wanted_positions[atlas_msgs::AtlasState::l_leg_kny] += req.l_leg_kny;
		wanted_positions[atlas_msgs::AtlasState::l_leg_uay] += req.l_leg_uay;
		wanted_positions[atlas_msgs::AtlasState::l_leg_lax] += req.l_leg_lax;
		wanted_positions[atlas_msgs::AtlasState::r_leg_uhz] += req.r_leg_uhz;
		wanted_positions[atlas_msgs::AtlasState::r_leg_mhx] += req.r_leg_mhx;
		wanted_positions[atlas_msgs::AtlasState::r_leg_lhy] += req.r_leg_lhy;
		wanted_positions[atlas_msgs::AtlasState::r_leg_kny] += req.r_leg_kny;
		wanted_positions[atlas_msgs::AtlasState::r_leg_uay] += req.r_leg_uay;
		wanted_positions[atlas_msgs::AtlasState::r_leg_lax] += req.r_leg_lax;
		if(req.l_leg_uhz_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_uhz] = req.l_leg_uhz_kp_position;
		if(req.l_leg_mhx_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_mhx] = req.l_leg_mhx_kp_position;
		if(req.l_leg_lhy_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_lhy] = req.l_leg_lhy_kp_position;
		if(req.l_leg_kny_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_kny] = req.l_leg_kny_kp_position;
		if(req.l_leg_uay_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_uay] = req.l_leg_uay_kp_position;
		if(req.l_leg_lax_kp_position) kp_positions[atlas_msgs::AtlasState::l_leg_lax] = req.l_leg_lax_kp_position;
		if(req.r_leg_uhz_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_uhz] = req.r_leg_uhz_kp_position;
		if(req.r_leg_mhx_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_mhx] = req.r_leg_mhx_kp_position;
		if(req.r_leg_lhy_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_lhy] = req.r_leg_lhy_kp_position;
		if(req.r_leg_kny_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_kny] = req.r_leg_kny_kp_position;
		if(req.r_leg_uay_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_uay] = req.r_leg_uay_kp_position;
		if(req.r_leg_lax_kp_position) kp_positions[atlas_msgs::AtlasState::r_leg_lax] = req.r_leg_lax_kp_position;
		return true;
	}

	bool delta_back_movement(PoseController::back_movement::Request &req, PoseController::back_movement::Response &res){
		//ROS_INFO("Got delta back movement %f", req.back_lbz);
		if(req.back_lbz != -100) wanted_positions[atlas_msgs::AtlasState::back_lbz] += req.back_lbz;
		wanted_positions[atlas_msgs::AtlasState::back_mby] += req.back_mby;
		wanted_positions[atlas_msgs::AtlasState::back_ubx] += req.back_ubx;
		return true;
	}

	bool delta_neck_movement(PoseController::neck_movement::Request &req, PoseController::neck_movement::Response &res){
		//ROS_INFO("Got neck movement");
		wanted_positions[atlas_msgs::AtlasState::neck_ay] += req.neck_ay;
		return true;
	}
	void ControlPose(){

		/*jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_uhz] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_uhz] = 0;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_uhz] = 0;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_uhz] = -30;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_uhz] = 30;
		jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_mhx] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_mhx] = 150;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_mhx] = 0;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_mhx] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_mhx] = 50;
		jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_uhz] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_uhz] = 0;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_uhz] = 50;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_uhz] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_uhz] = 50;
		jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_kny] = 6000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_kny] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_kny] = 50;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_kny] = 150;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_kny] = -150;
		/*jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_uay] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_uay] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_uay] = 10;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_uay] = -150;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_uay] = 150;
		jointcommands.kp_position[atlas_msgs::AtlasState::l_leg_lax] = 8000;
		jointcommands.ki_position[atlas_msgs::AtlasState::l_leg_lax] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::l_leg_lax] = 1.5;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::l_leg_lax] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::l_leg_lax] = 50;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_uhz] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_uhz] = 0;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_uhz] = 0;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_uhz] = -30;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_uhz] = 30;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_mhx] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_mhx] = 150;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_mhx] = 0;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_mhx] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_mhx] = 50;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_uhz] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_uhz] = 0;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_uhz] = 50;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_uhz] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_uhz] = 50;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_kny] = 6000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_kny] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_kny] = 50;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_kny] = -150;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_kny] = 150;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_uay] = 5000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_uay] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_uay] = 10;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_uay] = -150;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_uay] = 150;
		jointcommands.kp_position[atlas_msgs::AtlasState::r_leg_lax] = 8000;
		jointcommands.ki_position[atlas_msgs::AtlasState::r_leg_lax] = 500;
		jointcommands.kd_position[atlas_msgs::AtlasState::r_leg_lax] = 1.5;
		jointcommands.i_effort_min[atlas_msgs::AtlasState::r_leg_lax] = -50;
		jointcommands.i_effort_max[atlas_msgs::AtlasState::r_leg_lax] = 50;*/

		double dt = 0.005;
		while(nh2_.ok()){
			ros::spinOnce();
			atlascommand.position[atlas_msgs::AtlasState::l_leg_uhz] = wanted_positions[atlas_msgs::AtlasState::l_leg_uhz];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_uhz] = kp_positions[atlas_msgs::AtlasState::l_leg_uhz];
			atlascommand.position[atlas_msgs::AtlasState::l_leg_mhx] = wanted_positions[atlas_msgs::AtlasState::l_leg_mhx];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_mhx] = kp_positions[atlas_msgs::AtlasState::l_leg_mhx];
			atlascommand.position[atlas_msgs::AtlasState::l_leg_lhy] = wanted_positions[atlas_msgs::AtlasState::l_leg_lhy];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_lhy] = kp_positions[atlas_msgs::AtlasState::l_leg_lhy];
			atlascommand.position[atlas_msgs::AtlasState::l_leg_kny] = wanted_positions[atlas_msgs::AtlasState::l_leg_kny];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_kny] = kp_positions[atlas_msgs::AtlasState::l_leg_kny];
			atlascommand.position[atlas_msgs::AtlasState::l_leg_uay] = wanted_positions[atlas_msgs::AtlasState::l_leg_uay];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_uay] = kp_positions[atlas_msgs::AtlasState::l_leg_uay];
			atlascommand.position[atlas_msgs::AtlasState::l_leg_lax] = wanted_positions[atlas_msgs::AtlasState::l_leg_lax];
			atlascommand.kp_position[atlas_msgs::AtlasState::l_leg_lax] = kp_positions[atlas_msgs::AtlasState::l_leg_lax];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_uhz] = wanted_positions[atlas_msgs::AtlasState::r_leg_uhz];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_uhz] = kp_positions[atlas_msgs::AtlasState::r_leg_uhz];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_mhx] = wanted_positions[atlas_msgs::AtlasState::r_leg_mhx];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_mhx] = kp_positions[atlas_msgs::AtlasState::r_leg_mhx];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_lhy] = wanted_positions[atlas_msgs::AtlasState::r_leg_lhy];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_lhy] = kp_positions[atlas_msgs::AtlasState::r_leg_lhy];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_kny] = wanted_positions[atlas_msgs::AtlasState::r_leg_kny];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_kny] = kp_positions[atlas_msgs::AtlasState::r_leg_kny];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_uay] = wanted_positions[atlas_msgs::AtlasState::r_leg_uay];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_uay] = kp_positions[atlas_msgs::AtlasState::r_leg_uay];
			atlascommand.position[atlas_msgs::AtlasState::r_leg_lax] = wanted_positions[atlas_msgs::AtlasState::r_leg_lax];
			atlascommand.kp_position[atlas_msgs::AtlasState::r_leg_lax] = kp_positions[atlas_msgs::AtlasState::r_leg_lax];







			atlascommand.position[atlas_msgs::AtlasState::l_arm_usy] = wanted_positions[atlas_msgs::AtlasState::l_arm_usy];
			atlascommand.position[atlas_msgs::AtlasState::l_arm_shx] = wanted_positions[atlas_msgs::AtlasState::l_arm_shx];
			atlascommand.position[atlas_msgs::AtlasState::l_arm_ely] = wanted_positions[atlas_msgs::AtlasState::l_arm_ely];
			atlascommand.position[atlas_msgs::AtlasState::l_arm_elx] = wanted_positions[atlas_msgs::AtlasState::l_arm_elx];
			atlascommand.position[atlas_msgs::AtlasState::l_arm_uwy] = wanted_positions[atlas_msgs::AtlasState::l_arm_uwy];
			atlascommand.position[atlas_msgs::AtlasState::l_arm_mwx] = wanted_positions[atlas_msgs::AtlasState::l_arm_mwx];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_usy] = wanted_positions[atlas_msgs::AtlasState::r_arm_usy];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_shx] = wanted_positions[atlas_msgs::AtlasState::r_arm_shx];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_ely] = wanted_positions[atlas_msgs::AtlasState::r_arm_ely];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_elx] = wanted_positions[atlas_msgs::AtlasState::r_arm_elx];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_uwy] = wanted_positions[atlas_msgs::AtlasState::r_arm_uwy];
			atlascommand.position[atlas_msgs::AtlasState::r_arm_mwx] = wanted_positions[atlas_msgs::AtlasState::r_arm_mwx];






			atlascommand.position[atlas_msgs::AtlasState::neck_ay] = wanted_positions[atlas_msgs::AtlasState::neck_ay];





			atlascommand.position[atlas_msgs::AtlasState::back_lbz] = wanted_positions[atlas_msgs::AtlasState::back_lbz];
			atlascommand.kp_position[atlas_msgs::AtlasState::back_lbz] = 1000;
			atlascommand.ki_position[atlas_msgs::AtlasState::back_lbz] = 10;
			atlascommand.kd_position[atlas_msgs::AtlasState::back_lbz] = 100;
			atlascommand.position[atlas_msgs::AtlasState::back_mby] = wanted_positions[atlas_msgs::AtlasState::back_mby];
			atlascommand.kp_position[atlas_msgs::AtlasState::back_mby] = 1000;
			atlascommand.ki_position[atlas_msgs::AtlasState::back_mby] = 10;
			atlascommand.kd_position[atlas_msgs::AtlasState::back_mby] = 100;
			atlascommand.position[atlas_msgs::AtlasState::back_ubx] = wanted_positions[atlas_msgs::AtlasState::back_ubx];
			atlascommand.kp_position[atlas_msgs::AtlasState::back_ubx] = 1000;
			atlascommand.ki_position[atlas_msgs::AtlasState::back_ubx] = 10;
			atlascommand.kd_position[atlas_msgs::AtlasState::back_ubx] = 50;

			switch(atlassimintstate.desired_behavior){
			case atlas_msgs::AtlasSimInterfaceCommand::USER:
				for(int i = 0; i < (int) atlascommand.position.size(); i++){
					atlascommand.k_effort[i] = 255;
				}
				break;

			case atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE:
				atlascommand.k_effort[atlas_msgs::AtlasState::back_lbz] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::back_mby] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::back_ubx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::neck_ay] = 255;

				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_usy] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_shx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_ely] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_elx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_uwy] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_arm_mwx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_usy] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_shx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_ely] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_elx] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_uwy] = 255;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_arm_mwx] = 255;

				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_uhz] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_mhx] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_lhy] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_kny] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_uay] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::l_leg_lax] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_uhz] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_mhx] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_lhy] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_kny] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_uay] = 0;
				atlascommand.k_effort[atlas_msgs::AtlasState::r_leg_lax] = 0;
				break;

			default:
				for(int i = 0; i < (int) atlascommand.position.size(); i++){
					atlascommand.k_effort[i] = 0;
				}
				break;
			}


			if(up){
				pub_atlas_commands_.publish(atlascommand);
			}



			sensor_msgs::JointState jointstates;
			jointstates.position.resize(28);
			jointstates.position = atlascommand.position;
			pub_joint_states_.publish(jointstates);
			ros::Duration(dt).sleep();
		}
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "C45_PostureControl_PoseController");

	PoseControllerClass* PoseControl = new PoseControllerClass("PoseController");
	ROS_INFO("Running PoseController service");
	ros::spin();

	return 0;
}
