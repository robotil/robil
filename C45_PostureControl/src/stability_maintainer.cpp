/*
 * This class maintains the posture stability.
 * Dependencies:
 * 		com_error service
 * 		/back_mby_position_controller/command topic
 * 		/back_ubx_position_controller/command topic
 *
 * Advertises:
 * 		stability_maintainer_action action
 */


#include <ros/ros.h>
//#include <C45_PostureControl/C45_PostureControlAction.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <C45_PostureControl/back_lbz_poller_service.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <osrf_msgs/JointCommands.h>
#include <geometry_msgs/Point.h>
#include <hrl_kinematics/TestStability.h>


class stability_maintainer{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle* rosnode;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	control_toolbox::Pid back_ubx_stab_pid,back_mby_stab_pid; // PIDs for stability
	std::string action_name_;
	std_msgs::Float64 float64_msg;
	C45_PostureControl::back_lbz_poller_service back_;
	ros::Time clock;
	ros::Subscriber back_lbz_state_sub_;
	double back_lbz_state_;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;
	ros::Subscriber joint_states_sub_, imu_sub_, pcom_sub_;
	geometry_msgs::Point imu;
	geometry_msgs::Point com;

public:
	stability_maintainer(std::string name)
	:as_(nh_, name, false), action_name_(name){
		/*back_mby_stab_pid.initPid(p,i,d,M_PI,-M_PI);
		back_ubx_stab_pid.initPid(p,i,d,M_PI,-M_PI);*/

		rosnode = new ros::NodeHandle();
		positions.resize(28);
		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&stability_maintainer::joint_states_CB,this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100,&stability_maintainer::imu_CB,this);
		pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);
		pcom_sub_ = nh_.subscribe("projected_com", 1, &stability_maintainer::get_com_from_hrl_kinematics, this);

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&stability_maintainer::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&stability_maintainer::preemptCB, this));

		//back_lbz_state_sub_ = nh_.subscribe("/back_lbz_position_controller/state",100,&stability_maintainer::back_lbz_state_CB,this);

		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~stability_maintainer(){}
	/*void back_lbz_state_CB(const pr2_controllers_msgs::JointControllerStateConstPtr& back_lbz_state){
		ROS_ERROR("process value %f", back_lbz_state->process_value);
		back_lbz_state_=back_lbz_state->process_value;
	}*/
	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		positions = state->position;
	}

	void get_com_from_hrl_kinematics(const visualization_msgs::MarkerConstPtr& comMarker){
		//ROS_INFO("# of points: %d", (int) comMarker->points.size());

		com = comMarker->pose.position;
		//this->_CoM.x = com.x();
		//this->_CoM.y = com.y();
	}

	void imu_CB(const sensor_msgs::ImuConstPtr& imuC){

	/*Roll*/	imu.x = atan2(2*(imuC->orientation.w*imuC->orientation.x + imuC->orientation.y*imuC->orientation.z), 1 - 2*(pow(imuC->orientation.x,2) + pow(imuC->orientation.y,2)));
	/*Pitch*/	imu.y = asin(2*(imuC->orientation.w*imuC->orientation.y - imuC->orientation.z*imuC->orientation.x));
	/*Yaw*/		imu.z = atan2(2*(imuC->orientation.w*imuC->orientation.z + imuC->orientation.x*imuC->orientation.y), 1 - 2*(pow(imuC->orientation.y,2) + pow(imuC->orientation.z,2)));

	}

	void goalCB(){
		int segments = 1000;
		double total_time = 1;


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

		clock = ros::Time::now();
		ROS_INFO("Start time: %f", ros::Time::now().toSec());
		std::string goal_params = as_.acceptNewGoal()->parameters;

		double start_back_mby = positions[joints["back_mby"]];
		double start_back_ubx = positions[joints["back_ubx"]];
		double part_roll = (imu.x-start_back_ubx) / segments;
		double part_pitch = (imu.y-start_back_mby) / segments;
		double dt = 0.01;

		double kproll = 1;
		double kppitch = 1;
		//for(int i=0; i<segments; i++){
		while(1){
			jointcommands.position[joints["l_leg_uhz"]] = positions[joints["l_leg_uhz"]];
			jointcommands.position[joints["l_leg_mhx"]] = positions[joints["l_leg_mhx"]];
			jointcommands.position[joints["l_leg_lhy"]] = positions[joints["l_leg_lhy"]];
			jointcommands.position[joints["l_leg_kny"]] = positions[joints["l_leg_kny"]];
			jointcommands.position[joints["l_leg_uay"]] = positions[joints["l_leg_uay"]];
			jointcommands.position[joints["l_leg_lax"]] = positions[joints["l_leg_lax"]];
			jointcommands.position[joints["r_leg_uhz"]] = positions[joints["r_leg_uhz"]];
			jointcommands.position[joints["r_leg_mhx"]] = positions[joints["r_leg_mhx"]];
			jointcommands.position[joints["r_leg_lhy"]] = positions[joints["r_leg_lhy"]];
			jointcommands.position[joints["r_leg_kny"]] = positions[joints["r_leg_kny"]];
			jointcommands.position[joints["r_leg_uay"]] = positions[joints["r_leg_uay"]];
			jointcommands.position[joints["r_leg_lax"]] = positions[joints["r_leg_lax"]];
			jointcommands.position[joints["l_arm_usy"]] = positions[joints["l_arm_usy"]];
			jointcommands.position[joints["l_arm_shx"]] = positions[joints["l_arm_shx"]];
			jointcommands.position[joints["l_arm_ely"]] = positions[joints["l_arm_ely"]];
			jointcommands.position[joints["l_arm_elx"]] = positions[joints["l_arm_elx"]];
			jointcommands.position[joints["l_arm_uwy"]] = positions[joints["l_arm_uwy"]];
			jointcommands.position[joints["l_arm_mwx"]] = positions[joints["l_arm_mwx"]];
			jointcommands.position[joints["r_arm_usy"]] = positions[joints["r_arm_usy"]];
			jointcommands.position[joints["r_arm_shx"]] = positions[joints["r_arm_shx"]];
			jointcommands.position[joints["r_arm_ely"]] = positions[joints["r_arm_ely"]];
			jointcommands.position[joints["r_arm_elx"]] = positions[joints["r_arm_elx"]];
			jointcommands.position[joints["r_arm_uwy"]] = positions[joints["r_arm_uwy"]];
			jointcommands.position[joints["r_arm_mwx"]] = positions[joints["r_arm_mwx"]];
			jointcommands.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
			jointcommands.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
			jointcommands.position[joints["back_mby"]] = positions[joints["back_mby"]];
			jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];
			jointcommands.effort[joints["back_ubx"]]       = com.y * kproll;
			jointcommands.effort[joints["back_mby"]]       = com.x * kppitch;

			// To be reached 1+ind second after the start of the trajectory
			//current_dt += traj_vec_srv.response.dt[ind];
			pub_joint_commands_.publish(jointcommands);
			ros::Duration(dt).sleep();
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
	ros::init(argc, argv, "C45_PostureControl_maintain_stability");

	stability_maintainer* stab_maintainer = new stability_maintainer("MaintainStability");
	ROS_INFO("Running stability maintainer action");
	ros::spin();

	return 0;
}
