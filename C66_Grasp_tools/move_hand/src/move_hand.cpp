#include "ros/ros.h"
#include <ros/subscribe_options.h>
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "move_hand/move_hand.h"
#include "geometry_msgs/Vector3.h"
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <arms_val_calc/arms_val_calc.h>
#include <osrf_msgs/JointCommands.h>
#include <string>
#include <map>


class move_hand_service{

protected:
	ros::NodeHandle nh_, nh2_;
	ros::NodeHandle* rosnode;
	ros::ServiceServer move_hand_srv_;
	ros::ServiceClient traj_vector_cli_;
	ros::ServiceClient arms_val_calc_cli_;
	ros::Publisher traj_action_pub_;
	ros::Subscriber joint_states_sub_;
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_client_;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;

private:
	int _n;

public:
	move_hand_service():
		traj_client_(nh2_,"atlas_controller/follow_joint_trajectory", true)
	{
		rosnode = new ros::NodeHandle();
		ros::NodeHandle nh_private("~");
		traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("traj_vector_server");

		while(!traj_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the traj_vector_server server");
		}

		arms_val_calc_cli_ = nh_.serviceClient<arms_val_calc::arms_val_calc>("arms_val_calc_srv");

		while(!arms_val_calc_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the arms_val_calc_srv server");
		}

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&move_hand_service::joint_states_CB,this);

		// wait for action server to come up
		/*while(!traj_client_.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}*/
		pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);

		move_hand_srv_ = nh_.advertiseService("move_hand", &move_hand_service::gen_traj, this);
		ROS_INFO("running move hand service");
	}
	~move_hand_service(){
		//delete traj_client_;
	}
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

	bool gen_traj(move_hand::move_hand::Request &req, move_hand::move_hand::Response &res){
		traj_splitter_to_vector::trajectory_vector traj_vec_left_srv;
		traj_splitter_to_vector::trajectory_vector traj_vec_right_srv;
		//control_msgs::FollowJointTrajectoryGoal goal;
		osrf_msgs::JointCommands jointcommands;
		//ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_left_srv.request.Position = req.PositionDestination_left;
		traj_vec_left_srv.request.Angle = req.AngleDestination_left;
		traj_vec_left_srv.request.segments_number = 1000 ;
		traj_vec_left_srv.request.total_time = 5 ;

                traj_vec_right_srv.request.Position = req.PositionDestination_right;
                traj_vec_right_srv.request.Angle = req.AngleDestination_right;
                traj_vec_right_srv.request.segments_number = 1000 ;
                traj_vec_right_srv.request.total_time = 5 ;
		/*
	trajectory_msgs::JointTrajectory jt;
	jt.header.stamp = ros::Time::now();
    jt.header.frame_id = "atlas::hand";
		 */

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

		if(traj_vector_cli_.call(traj_vec_left_srv)&&traj_vector_cli_.call(traj_vec_right_srv)){
			double p0_l,p1_l,p2_l,p3_l,p4_l,p5_l;
			double p0_r,p1_r,p2_r,p3_r,p4_r,p5_r;
			p0_l = q0_l;
			p1_l = q1_l;
			p2_l = q2_l;
			p3_l = q3_l;
			p4_l = q4_l;
			p5_l = q5_l;
			p0_r = q0_r;
			p1_r = q1_r;
			p2_r = q2_r;
			p3_r = q3_r;
			p4_r = q4_r;
			p5_r = q5_r;

			//ROS_INFO("Response from trajectory vector service: Position size %d", (int) traj_vec_srv.response.PositionArray.size());
			//ROS_INFO("Response from trajectory vector service: Angle size %d", (int) traj_vec_srv.response.AngleArray.size());
			double current_dt = 0;
			if (traj_vec_left_srv.response.PositionArray.size() != traj_vec_right_srv.response.PositionArray.size()) return false;
			for(unsigned int ind = 0; ind < traj_vec_left_srv.response.PositionArray.size(); ind++){
				arms_val_calc::arms_val_calc v_left,v_right;
				//ROS_INFO("Response from trajectory vector service example: %f", traj_vec_srv.response.PositionArray[ind].x);
				v_left.request.x_dot = traj_vec_left_srv.response.PositionArray[ind].x;
				v_left.request.y_dot = traj_vec_left_srv.response.PositionArray[ind].y;
				v_left.request.z_dot = traj_vec_left_srv.response.PositionArray[ind].z;
				v_left.request.roll_dot = traj_vec_left_srv.response.AngleArray[ind].x;
				v_left.request.pitch_dot = traj_vec_left_srv.response.AngleArray[ind].y;
				v_left.request.yaw_dot = traj_vec_left_srv.response.AngleArray[ind].z;

                                v_right.request.x_dot = traj_vec_right_srv.response.PositionArray[ind].x;
                                v_right.request.y_dot = traj_vec_right_srv.response.PositionArray[ind].y;
                                v_right.request.z_dot = traj_vec_right_srv.response.PositionArray[ind].z;
                                v_right.request.roll_dot = traj_vec_right_srv.response.AngleArray[ind].x;
                                v_right.request.pitch_dot = traj_vec_right_srv.response.AngleArray[ind].y;
                                v_right.request.yaw_dot = traj_vec_right_srv.response.AngleArray[ind].z;
				/*ROS_INFO("Pose of dt %f: (%f,%f,%f)",traj_vec_srv.response.dt[ind], traj_vec_srv.response.PositionArray[ind].x,
						traj_vec_srv.response.PositionArray[ind].y, traj_vec_srv.response.PositionArray[ind].z);*/

				if(arms_val_calc_cli_.call(v_left)&&arms_val_calc_cli_.call(v_right)){

							jointcommands.position[joints["l_arm_usy"]] = p0_l + v_left.response.q_left_dot[0]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_usy"]] = v_left.response.q_left_dot[0];
							p0_l = jointcommands.position[joints["l_arm_usy"]];
							jointcommands.position[joints["l_arm_shx"]] = p1_l + v_left.response.q_left_dot[1]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_shx"]] = v_left.response.q_left_dot[1];
							p1_l = jointcommands.position[joints["l_arm_shx"]];
							jointcommands.position[joints["l_arm_ely"]] = p2_l + v_left.response.q_left_dot[2]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_ely"]] = v_left.response.q_left_dot[2];
							p2_l = jointcommands.position[joints["l_arm_ely"]];
							jointcommands.position[joints["l_arm_elx"]] = p3_l + v_left.response.q_left_dot[3]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_elx"]] = v_left.response.q_left_dot[3];
							p3_l = jointcommands.position[joints["l_arm_elx"]];
							jointcommands.position[joints["l_arm_uwy"]] = p4_l + v_left.response.q_left_dot[4]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_uwy"]] = v_left.response.q_left_dot[4];
							p4_l = jointcommands.position[joints["l_arm_uwy"]];
							jointcommands.position[joints["l_arm_mwx"]] = p5_l + v_left.response.q_left_dot[5]*traj_vec_left_srv.response.dt[ind];
							jointcommands.velocity[joints["l_arm_mwx"]] = v_left.response.q_left_dot[5];
							p5_l = jointcommands.position[joints["l_arm_mwx"]];


								jointcommands.position[joints["r_arm_usy"]] = p0_r + v_right.response.q_right_dot[0]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_usy"]] = v_right.response.q_right_dot[0];
								p0_r = jointcommands.position[joints["r_arm_usy"]];
								jointcommands.position[joints["r_arm_shx"]] = p1_r + v_right.response.q_right_dot[1]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_shx"]] = v_right.response.q_right_dot[1];
								p1_r = jointcommands.position[joints["r_arm_shx"]];
								jointcommands.position[joints["r_arm_ely"]] = p2_r + v_right.response.q_right_dot[2]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_ely"]] = v_right.response.q_right_dot[2];
								p2_r = jointcommands.position[joints["r_arm_ely"]];
								jointcommands.position[joints["r_arm_elx"]] = p3_r + v_right.response.q_right_dot[3]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_elx"]] = v_right.response.q_right_dot[3];
								p3_r = jointcommands.position[joints["r_arm_elx"]];
								jointcommands.position[joints["r_arm_uwy"]] = p4_r + v_right.response.q_right_dot[4]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_uwy"]] = v_right.response.q_right_dot[4];
								p4_r = jointcommands.position[joints["r_arm_uwy"]];
								jointcommands.position[joints["r_arm_mwx"]] = p5_r + v_right.response.q_right_dot[5]*traj_vec_right_srv.response.dt[ind];
								//jointcommands.velocity[joints["r_arm_mwx"]] = v_right.response.q_right_dot[5];
								p5_r = jointcommands.position[joints["r_arm_mwx"]];

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

					jointcommands.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
					jointcommands.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
					jointcommands.position[joints["back_mby"]] = positions[joints["back_mby"]];
					jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];

					// To be reached 1+ind second after the start of the trajectory
					pub_joint_commands_.publish(jointcommands);
					ros::Duration(traj_vec_left_srv.response.dt[ind]).sleep();
				}else{
					ROS_ERROR("Could not reach gen arms velocity vector server");
					res.success = false;
					return false;
				}
			}

			//ROS_INFO("starting trajectory");


			/*
    	while(!traj_client_.waitForServer(ros::Duration(1.0))){
	    	ROS_INFO("Waiting for the joint_trajectory_action server");
	    }*/
			// When to start the trajectory: 0.5s from now
			/*goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
			ROS_INFO("sending trajectory");
			traj_client_.sendGoal(goal);*/
			res.success = true;
			return true;
		}else{
			ROS_ERROR("Could not reach trajectory vector server");
			res.success = false;
			return false;
		}
		res.success = true;
		return true;
	}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_hand_service");
	ROS_INFO("starting move hand service");
	move_hand_service* c = new move_hand_service();
	ROS_INFO("started move hand service");

	ros::spin();
	return 0;
}



