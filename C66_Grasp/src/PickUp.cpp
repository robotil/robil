#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <move_hand/matrix.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <dFind/perceptionTransform.h>
#include <boost/bind/bind.hpp>
#include <hand_grasp/grasp.h>
#include <move_hand/pelvis_move_hand.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sensor_msgs/JointState.h>
class PickUp{
private:
	ros::NodeHandle nh_;
	ros::ServiceClient pelvis_move_hand_matrix_cli_,perception_transform_cli_,pelvis_move_hand_cli_;

	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	std::string action_name_;

	ros::NodeHandle* rosnode;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_,pub_hand_grasp_;
	ros::Subscriber joint_states_sub_;
        double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
        double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
public:
	PickUp(std::string name)
	:as_(nh_, name, false), action_name_(name){
	  rosnode = new ros::NodeHandle();
	  pelvis_move_hand_matrix_cli_ = nh_.serviceClient<move_hand::matrix>("C66_matrix");
	  pelvis_move_hand_cli_ = nh_.serviceClient<move_hand::pelvis_move_hand>("pelvis_move_hand");
	  perception_transform_cli_ = nh_.serviceClient<dFind::perceptionTransform>("perceptionTransform");
/*		while(!move_hand_cli_.waitForExistence(ros::Duration(0.1))){
			ROS_INFO("Waiting for the move_hand server");
		}
*/
		//Set callback functions
		as_.registerGoalCallback(boost::bind(&PickUp::goalCB, this));
//		as_.registerPreemptCallback(boost::bind(&PickUp::preemptCB, this));
		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&PickUp::joint_states_CB,this);
		pub_joint_commands_ = rosnode->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 100, true);
		pub_hand_grasp_ = rosnode->advertise<hand_grasp::grasp>("/hand_grasp_right", 100, true);
		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~PickUp(){}

	void goalCB(){

		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;
/*

		ROS_INFO("moving to position 0");
		position0();
		ros::Duration(0.5).sleep();
		ROS_INFO("done position 0");
*/
		ROS_INFO("opening right hand");
		hand_grasp::grasp grasp_msg;
		grasp_msg.strength = 0;
		pub_hand_grasp_.publish(grasp_msg);
		dFind::perceptionTransform perception_srv_msg;
		perception_srv_msg.request.command = g;
		ROS_INFO("requesting transformation from perception");
		if (perception_transform_cli_.call(perception_srv_msg)){
		  move_hand::matrix move_hand_matrix_msg;
		  move_hand_matrix_msg.request.matrix = perception_srv_msg.response.transMat;
		  if (as_.isPreemptRequested() || !ros::ok())
		                  {
		                          ROS_ERROR("%s: Preempted", action_name_.c_str());
		                          // set the action state to preempted
		                          as_.setPreempted();
		                          return;
		                  }
		  ROS_INFO("moving hand to requested object");
		  if(pelvis_move_hand_matrix_cli_.call(move_hand_matrix_msg)){
		    if (!move_hand_matrix_msg.response.success){
		      as_.setAborted();
		    return;}
		  }
		else{
	                ROS_INFO("ERROR in move_hand service");
	                  as_.setAborted();
	                  return;
		}}
		else {
		  ROS_INFO("ERROR in requesting transformation from perception");
		  as_.setAborted();
		}
                ROS_INFO("closing right hand");
                grasp_msg.strength = 1000;
                pub_hand_grasp_.publish(grasp_msg);
                ros::Duration(1.2).sleep();
                move_hand::pelvis_move_hand pel_move_msg;
                pel_move_msg.request.PositionDestination_right.x = 0.684;
                pel_move_msg.request.PositionDestination_right.y = 0.095;
                pel_move_msg.request.PositionDestination_right.z = 0.361;
                pel_move_msg.request.AngleDestination_right.x = -1.790;
                pel_move_msg.request.AngleDestination_right.y = -0.066;
                pel_move_msg.request.AngleDestination_right.z = -0.066;
                ROS_INFO("moving to target");
                pelvis_move_hand_cli_.call(pel_move_msg);
                ROS_INFO("dropping object");
                grasp_msg.strength = 0;
                pub_hand_grasp_.publish(grasp_msg);
                ros::Duration(1.2).sleep();
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

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
	                for(unsigned int i=0; i < state->name.size(); i++){
	                        joints[state->name[i]] = i;
	                }
	                q0_l=state->position[joints["l_arm_usy"]];
	                q1_l=state->position[joints["l_arm_shx"]];
	                q2_l=state->position[joints["l_arm_ely"]];
	                q3_l=state->position[joints["l_arm_elx"]];
	                q4_l=state->position[joints["l_arm_uwy"]];
	                q5_l=state->position[joints["l_arm_mwx"]];
	                q0_r=state->position[joints["r_arm_usy"]];
	                q1_r=state->position[joints["r_arm_shx"]];
	                q2_r=state->position[joints["r_arm_ely"]];
	                q3_r=state->position[joints["r_arm_elx"]];
	                q4_r=state->position[joints["r_arm_uwy"]];
	                q5_r=state->position[joints["r_arm_mwx"]];
	                positions = state->position;
	        }

	void position0(){
          osrf_msgs::JointCommands jointcommands;
          atlas_msgs::AtlasCommand atlas_command;
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

                          atlas_command.position.resize(n);
                          atlas_command.velocity.resize(n);
                          atlas_command.effort.resize(n);
                          atlas_command.kp_position.resize(n);
                          atlas_command.ki_position.resize(n);
                          atlas_command.kd_position.resize(n);
                          atlas_command.kp_velocity.resize(n);
                          atlas_command.i_effort_min.resize(n);
                          atlas_command.i_effort_max.resize(n);
                          atlas_command.k_effort.resize(n);
                          for (unsigned int i = 0; i < n; i++)
                          {
                                  std::vector<std::string> pieces;
                                  boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

                                  rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
                                                    jointcommands.kp_position[i]);
                                  atlas_command.kp_position[i] = jointcommands.kp_position[i];
                                  rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
                                                    jointcommands.ki_position[i]);
                                  atlas_command.ki_position[i] = jointcommands.ki_position[i];
                                  rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
                                                    jointcommands.kd_position[i]);
                                  atlas_command.kd_position[i] = jointcommands.kd_position[i];
                                  rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
                                                    jointcommands.i_effort_min[i]);
                                  atlas_command.i_effort_min[i] = -jointcommands.i_effort_min[i];

                                  rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
                                                    jointcommands.i_effort_max[i]);
                                  atlas_command.i_effort_max[i] = jointcommands.i_effort_max[i];

                                  atlas_command.kp_position[i] = jointcommands.kp_position[i];
                                  atlas_command.velocity[i]     = 0;
                                  atlas_command.effort[i]       = 0;
                                  atlas_command.kp_velocity[i]  = 0;
                                  atlas_command.k_effort[i] = 255;
                          }
for(unsigned int i = 0; i < 101; i++){
                          atlas_command.position[joints["l_arm_usy"]] = -0.3*(i/100.0);
                          atlas_command.position[joints["l_arm_shx"]] = -0.9*(i/100.0);
                          atlas_command.position[joints["l_arm_ely"]] = 1.2*(i/100.0);
                          atlas_command.position[joints["l_arm_elx"]] = 1.5*(i/100.0);
                          atlas_command.position[joints["l_arm_uwy"]] = 0.6*(i/100.0);
                          atlas_command.position[joints["l_arm_mwx"]] = 0.1*(i/100.0);

                          atlas_command.position[joints["r_arm_usy"]] = -0.3*(i/100.0);
                          atlas_command.position[joints["r_arm_shx"]] = 0.9*(i/100.0);
                          atlas_command.position[joints["r_arm_ely"]] = 1.2*(i/100.0);
                          atlas_command.position[joints["r_arm_elx"]] = -1.5*(i/100.0);
                          atlas_command.position[joints["r_arm_uwy"]] = 0.6*(i/100.0);
                          atlas_command.position[joints["r_arm_mwx"]] = 0.1*(i/100.0);



                          atlas_command.position[joints["l_leg_uhz"]] = positions[joints["l_leg_uhz"]];
                          atlas_command.position[joints["l_leg_mhx"]] = positions[joints["l_leg_mhx"]];
                          atlas_command.position[joints["l_leg_lhy"]] = positions[joints["l_leg_lhy"]];
                          atlas_command.position[joints["l_leg_kny"]] = positions[joints["l_leg_kny"]];
                          atlas_command.position[joints["l_leg_uay"]] = positions[joints["l_leg_uay"]];
                          atlas_command.position[joints["l_leg_lax"]] = positions[joints["l_leg_lax"]];

                          atlas_command.position[joints["r_leg_uhz"]] = positions[joints["r_leg_uhz"]];
                          atlas_command.position[joints["r_leg_mhx"]] = positions[joints["r_leg_mhx"]];
                          atlas_command.position[joints["r_leg_lhy"]] = positions[joints["r_leg_lhy"]];
                          atlas_command.position[joints["r_leg_kny"]] = positions[joints["r_leg_kny"]];
                          atlas_command.position[joints["r_leg_uay"]] = positions[joints["r_leg_uay"]];
                          atlas_command.position[joints["r_leg_lax"]] = positions[joints["r_leg_lax"]];

                          atlas_command.position[joints["neck_ay"]] = 0.8*(i/100.0);
                          atlas_command.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
                          atlas_command.position[joints["back_mby"]] = positions[joints["back_mby"]];
                          atlas_command.position[joints["back_ubx"]] = positions[joints["back_ubx"]];

                          pub_joint_commands_.publish(atlas_command);
                          ros::Duration(0.02).sleep();
}
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C66_PickUp");

	PickUp* QuasiStaticWalker = new PickUp("PickUp");
	ROS_INFO("Running PickUp action");
	ros::spin();

	return 0;
}
