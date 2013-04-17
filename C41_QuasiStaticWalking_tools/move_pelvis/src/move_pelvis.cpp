#include "ros/ros.h"
#include <ros/subscribe_options.h>
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "move_pelvis/move_pelvis.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTolerance.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <legs_val_calc/legs_val_calc.h>
#include <osrf_msgs/JointCommands.h>
#include <string>
#include <map>


class move_pelvis_service{

protected:
	ros::NodeHandle nh_, nh2_;
	ros::NodeHandle* rosnode;
	ros::ServiceServer move_pelvis_srv_;
	ros::ServiceClient traj_vector_cli_,walking_vector_cli_;
	ros::ServiceClient legs_val_calc_cli_;
	ros::Publisher traj_action_pub_;
	//ros::Subscriber l_leg_uhz_sub_,l_leg_mhx_sub_,l_leg_lhy_sub_,l_leg_kny_sub_,l_leg_uay_sub_,l_leg_lax_sub_;
	//ros::Subscriber r_leg_uhz_sub_,r_leg_mhx_sub_,r_leg_lhy_sub_,r_leg_kny_sub_,r_leg_uay_sub_,r_leg_lax_sub_;
	ros::Subscriber joint_states_sub_, imu_sub_;
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_client_;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;
	geometry_msgs::Point imu;

private:
	int _n;

public:
	move_pelvis_service():
		traj_client_(nh2_,"atlas_controller/follow_joint_trajectory", true)
	{
		rosnode = new ros::NodeHandle();
		ros::NodeHandle nh_private("~");
		traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("traj_vector_server");
		walking_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("walking_trajectory_vector");

		while(!traj_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the traj_vector_server server");
		}

		legs_val_calc_cli_ = nh_.serviceClient<legs_val_calc::legs_val_calc>("legs_val_calc_srv");

		while(!legs_val_calc_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the legs_val_calc_srv server");
		}
		//traj_vector_cli_ = nh_.subscribe("trajectory_vector", 1, &com_error_node::get_com_from_hrl_kinematics, this);
		//support_polygon_sub_ = nh_.subscribe("support_polygon", 1, &com_error_node::get_support_polygon_from_hrl_kinematics, this);

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&move_pelvis_service::joint_states_CB,this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100,&move_pelvis_service::imu_CB,this);

		// wait for action server to come up
		/*while(!traj_client_.waitForServer(ros::Duration(1.0))){
			ROS_INFO("Waiting for the joint_trajectory_action server");
		}*/
		pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);

		move_pelvis_srv_ = nh_.advertiseService("move_pelvis", &move_pelvis_service::gen_traj, this);
		ROS_INFO("running move pelvis service");
	}
	~move_pelvis_service(){
		//delete traj_client_;
	}
	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		q0_l=state->position[joints["l_leg_uhz"]];
		q1_l=state->position[joints["l_leg_mhx"]];
		q2_l=state->position[joints["l_leg_lhy"]];
		q3_l=state->position[joints["l_leg_kny"]];
		q4_l=state->position[joints["l_leg_uay"]];
		q5_l=state->position[joints["l_leg_lax"]];
		q0_r=state->position[joints["r_leg_uhz"]];
		q1_r=state->position[joints["r_leg_mhx"]];
		q2_r=state->position[joints["r_leg_lhy"]];
		q3_r=state->position[joints["r_leg_kny"]];
		q4_r=state->position[joints["r_leg_uay"]];
		q5_r=state->position[joints["r_leg_lax"]];
		positions = state->position;
	}
	void imu_CB(const sensor_msgs::ImuConstPtr& imuC){
		imu.x = atan2(2*(imuC->orientation.w*imuC->orientation.x + imuC->orientation.y*imuC->orientation.z), 1 - 2*(pow(imuC->orientation.x,2) + pow(imuC->orientation.y,2)));
		imu.y = asin(2*(imuC->orientation.w*imuC->orientation.y - imuC->orientation.z*imuC->orientation.x));
		imu.z = atan2(2*(imuC->orientation.w*imuC->orientation.z + imuC->orientation.x*imuC->orientation.y), 1 - 2*(pow(imuC->orientation.y,2) + pow(imuC->orientation.z,2)));
	}

	bool gen_traj(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		traj_splitter_to_vector::trajectory_vector traj_vec_srv;
		//control_msgs::FollowJointTrajectoryGoal goal;
		osrf_msgs::JointCommands jointcommands;
		ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_srv.request.Position = req.PositionDestination;
		traj_vec_srv.request.Angle = req.AngleDestination;
		traj_vec_srv.request.segments_number = 1000 ;
		traj_vec_srv.request.total_time = 5 ;
		double kp = 10;
		/*
	trajectory_msgs::JointTrajectory jt;
	jt.header.stamp = ros::Time::now();
    jt.header.frame_id = "atlas::pelvis";
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

		if(!req.LinkToMove.compare("pelvis")){

			traj_vec_srv.request.Position.x *= 1.10;
			traj_vec_srv.request.Position.y *= 1.10;
			traj_vec_srv.request.Position.z *= 1.10;
			if(traj_vector_cli_.call(traj_vec_srv)){
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
				for(unsigned int ind = 0; ind < traj_vec_srv.response.PositionArray.size(); ind++){
					legs_val_calc::legs_val_calc v;
					//ROS_INFO("Response from trajectory vector service example: %f", traj_vec_srv.response.PositionArray[ind].x);
					v.request.x_dot = traj_vec_srv.response.PositionArray[ind].x;
					v.request.y_dot = traj_vec_srv.response.PositionArray[ind].y;
					v.request.z_dot = traj_vec_srv.response.PositionArray[ind].z;
					v.request.roll_dot = traj_vec_srv.response.AngleArray[ind].x;// - imu.x;
					v.request.pitch_dot = traj_vec_srv.response.AngleArray[ind].y;// - imu.y;
					v.request.yaw_dot = traj_vec_srv.response.AngleArray[ind].z;// - imu.z;
					/*ROS_INFO("Pose of dt %f: (%f,%f,%f)",traj_vec_srv.response.dt[ind], traj_vec_srv.response.PositionArray[ind].x,
						traj_vec_srv.response.PositionArray[ind].y, traj_vec_srv.response.PositionArray[ind].z);*/

					if(legs_val_calc_cli_.call(v)){
						//trajectory_msgs::JointTrajectoryPoint p;

						/*jointcommands.name.push_back("l_leg_uhz");
        	    jointcommands.name.push_back("l_leg_mhx");
        	    jointcommands.name.push_back("l_leg_lhy");
        	    jointcommands.name.push_back("l_leg_kny");
        	    jointcommands.name.push_back("l_leg_uay");
        	    jointcommands.name.push_back("l_leg_lax");*/
						jointcommands.position[joints["l_leg_uhz"]] = p0_l + v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_uhz"]] = p0_l + v.response.q_left_dot[0];
						p0_l = jointcommands.position[joints["l_leg_uhz"]];
						jointcommands.position[joints["l_leg_mhx"]] = p1_l + v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_mhx"]] = p0_l + v.response.q_left_dot[1];
						p1_l = jointcommands.position[joints["l_leg_mhx"]];
						jointcommands.position[joints["l_leg_lhy"]] = p2_l + v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_lhy"]] = p0_l + v.response.q_left_dot[2];
						p2_l = jointcommands.position[joints["l_leg_lhy"]];
						jointcommands.position[joints["l_leg_kny"]] = p3_l + v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_kny"]] = p0_l + v.response.q_left_dot[3];
						p3_l = jointcommands.position[joints["l_leg_kny"]];
						jointcommands.position[joints["l_leg_uay"]] = p4_l + v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_uay"]] = p0_l + v.response.q_left_dot[4];
						p4_l = jointcommands.position[joints["l_leg_uay"]];
						jointcommands.position[joints["l_leg_lax"]] = p5_l + v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["l_leg_lax"]] = p0_l + v.response.q_left_dot[5];
						p5_l = jointcommands.position[joints["l_leg_lax"]];
						//ROS_INFO("Sent to left leg (%f, %f, %f, %f, %f, %f)", p0_l, p1_l, p2_l, p3_l, p4_l, p5_l);
						/*goal.trajectory.points[ind].velocities[0] = v.response.q_left_dot[0];
					goal.trajectory.points[ind].velocities[1] = v.response.q_left_dot[1];
					goal.trajectory.points[ind].velocities[2] = v.response.q_left_dot[2];
					goal.trajectory.points[ind].velocities[3] = v.response.q_left_dot[3];
					goal.trajectory.points[ind].velocities[4] = v.response.q_left_dot[4];
					goal.trajectory.points[ind].velocities[5] = v.response.q_left_dot[5];*/
						/*
        	    jointcommands.name.push_back("r_leg_uhz");
        	    jointcommands.name.push_back("r_leg_mhx");
        	    jointcommands.name.push_back("r_leg_lhy");
        	    jointcommands.name.push_back("r_leg_kny");
        	    jointcommands.name.push_back("r_leg_uay");
        	    jointcommands.name.push_back("r_leg_lax");
						 */
						jointcommands.position[joints["r_leg_uhz"]] = p0_r + v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_uhz"]] = p0_l + v.response.q_right_dot[0];
						p0_r = jointcommands.position[joints["r_leg_uhz"]];
						jointcommands.position[joints["r_leg_mhx"]] = p1_r + v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_mhx"]] = p0_l + v.response.q_right_dot[1];
						p1_r = jointcommands.position[joints["r_leg_mhx"]];
						jointcommands.position[joints["r_leg_lhy"]] = p2_r + v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_lhy"]] = p0_l + v.response.q_right_dot[2];
						p2_r = jointcommands.position[joints["r_leg_lhy"]];
						jointcommands.position[joints["r_leg_kny"]] = p3_r + v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_kny"]] = p0_l + v.response.q_right_dot[3];
						p3_r = jointcommands.position[joints["r_leg_kny"]];
						jointcommands.position[joints["r_leg_uay"]] = p4_r + v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_uay"]] = p0_l + v.response.q_right_dot[4];
						p4_r = jointcommands.position[joints["r_leg_uay"]];
						jointcommands.position[joints["r_leg_lax"]] = p5_r + v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
						jointcommands.velocity[joints["r_leg_lax"]] = p0_l + v.response.q_right_dot[5];
						p5_r = jointcommands.position[joints["r_leg_lax"]];
						/*goal.trajectory.points[ind].velocities[6] = v.response.q_right_dot[0];
					goal.trajectory.points[ind].velocities[7] = v.response.q_right_dot[1];
					goal.trajectory.points[ind].velocities[8] = v.response.q_right_dot[2];
					goal.trajectory.points[ind].velocities[9] = v.response.q_right_dot[3];
					goal.trajectory.points[ind].velocities[10] = v.response.q_right_dot[4];
					goal.trajectory.points[ind].velocities[11] = v.response.q_right_dot[5];*/
						/*
        	    jointcommands.name.push_back("l_arm_usy");
        	    jointcommands.name.push_back("l_arm_shx");
        	    jointcommands.name.push_back("l_arm_ely");
        	    jointcommands.name.push_back("l_arm_elx");
        	    jointcommands.name.push_back("l_arm_uwy");
        	    jointcommands.name.push_back("l_arm_mwx");
						 */
						jointcommands.position[joints["l_arm_usy"]] = positions[joints["l_arm_usy"]];
						jointcommands.position[joints["l_arm_shx"]] = positions[joints["l_arm_shx"]];
						jointcommands.position[joints["l_arm_ely"]] = positions[joints["l_arm_ely"]];
						jointcommands.position[joints["l_arm_elx"]] = positions[joints["l_arm_elx"]];
						jointcommands.position[joints["l_arm_uwy"]] = positions[joints["l_arm_uwy"]];
						jointcommands.position[joints["l_arm_mwx"]] = positions[joints["l_arm_mwx"]];
						//ROS_INFO("%f, %f, %f, %f, %f, %f", jointcommands.position[12], jointcommands.position[13], jointcommands.position[14], jointcommands.position[15], jointcommands.position[16], jointcommands.position[17]);
						/*
        	    jointcommands.name.push_back("r_arm_usy");
        	    jointcommands.name.push_back("r_arm_shx");
        	    jointcommands.name.push_back("r_arm_ely");
        	    jointcommands.name.push_back("r_arm_elx");
        	    jointcommands.name.push_back("r_arm_uwy");
        	    jointcommands.name.push_back("r_arm_mwx");
						 */
						jointcommands.position[joints["r_arm_usy"]] = positions[joints["r_arm_usy"]];
						jointcommands.position[joints["r_arm_shx"]] = positions[joints["r_arm_shx"]];
						jointcommands.position[joints["r_arm_ely"]] = positions[joints["r_arm_ely"]];
						jointcommands.position[joints["r_arm_elx"]] = positions[joints["r_arm_elx"]];
						jointcommands.position[joints["r_arm_uwy"]] = positions[joints["r_arm_uwy"]];
						jointcommands.position[joints["r_arm_mwx"]] = positions[joints["r_arm_mwx"]];
						//ROS_INFO("%f, %f, %f, %f, %f, %f", jointcommands.position[18], jointcommands.position[19], jointcommands.position[20], jointcommands.position[21], jointcommands.position[22], jointcommands.position[23]);
						/*
        	    jointcommands.name.push_back("neck_ay"  );
        	    jointcommands.name.push_back("back_lbz" );
        	    jointcommands.name.push_back("back_mby" );
        	    jointcommands.name.push_back("back_ubx" );
						 */
						jointcommands.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
						jointcommands.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
						jointcommands.position[joints["back_mby"]] = positions[joints["back_mby"]];
						jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];//kp*imu.x;
						//ROS_INFO("%f, %f, %f, %f", jointcommands.position[24], jointcommands.position[25], jointcommands.position[26], jointcommands.position[27]);

						// To be reached 1+ind second after the start of the trajectory
						current_dt += traj_vec_srv.response.dt[ind];
						pub_joint_commands_.publish(jointcommands);
						ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
					}else{
						ROS_ERROR("Could not reach gen legs velocity vector server");
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
		}else{

			traj_vec_srv.request.total_time = 10 ;
			if(walking_vector_cli_.call(traj_vec_srv)){
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

				ROS_INFO("Response from trajectory vector service: Position size %d", (int) traj_vec_srv.response.PositionArray.size());
				ROS_INFO("Response from trajectory vector service: Angle size %d", (int) traj_vec_srv.response.AngleArray.size());
				double current_dt = 0;
				for(unsigned int ind = 0; ind < traj_vec_srv.response.PositionArray.size(); ind++){
					legs_val_calc::legs_val_calc v;
					//ROS_INFO("Response from trajectory vector service example: %f", traj_vec_srv.response.PositionArray[ind].x);
					v.request.x_dot = traj_vec_srv.response.PositionArray[ind].x;
					v.request.y_dot = traj_vec_srv.response.PositionArray[ind].y;
					v.request.z_dot = traj_vec_srv.response.PositionArray[ind].z;
					v.request.roll_dot = traj_vec_srv.response.AngleArray[ind].x;// - imu.x;
					v.request.pitch_dot = traj_vec_srv.response.AngleArray[ind].y;// - imu.y;
					v.request.yaw_dot = traj_vec_srv.response.AngleArray[ind].z;// - imu.z;
					/*ROS_INFO("Pose of dt %f: (%f,%f,%f)",traj_vec_srv.response.dt[ind], traj_vec_srv.response.PositionArray[ind].x,
						traj_vec_srv.response.PositionArray[ind].y, traj_vec_srv.response.PositionArray[ind].z);*/

					if(legs_val_calc_cli_.call(v)){
						if(!req.LinkToMove.compare("l_leg")){
							jointcommands.position[joints["l_leg_uhz"]] = p0_l + v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_uhz"]] = p0_l + v.response.q_left_dot[0];
							p0_l = jointcommands.position[joints["l_leg_uhz"]];
							jointcommands.position[joints["l_leg_mhx"]] = p1_l + v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_mhx"]] = p0_l + v.response.q_left_dot[1];
							p1_l = jointcommands.position[joints["l_leg_mhx"]];
							jointcommands.position[joints["l_leg_lhy"]] = p2_l + v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_lhy"]] = p0_l + v.response.q_left_dot[2];
							p2_l = jointcommands.position[joints["l_leg_lhy"]];
							jointcommands.position[joints["l_leg_kny"]] = p3_l + v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_kny"]] = p0_l + v.response.q_left_dot[3];
							p3_l = jointcommands.position[joints["l_leg_kny"]];
							jointcommands.position[joints["l_leg_uay"]] = p4_l + v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_uay"]] = p0_l + v.response.q_left_dot[4];
							p4_l = jointcommands.position[joints["l_leg_uay"]];
							jointcommands.position[joints["l_leg_lax"]] = p5_l + v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
							jointcommands.velocity[joints["l_leg_lax"]] = p0_l + v.response.q_left_dot[5];
							p5_l = jointcommands.position[joints["l_leg_lax"]];
							//ROS_INFO("Sent to left leg (%f, %f, %f, %f, %f, %f)", p0_l, p1_l, p2_l, p3_l, p4_l, p5_l);
							jointcommands.position[joints["r_leg_uhz"]] = positions[joints["r_leg_uhz"]];
							jointcommands.position[joints["r_leg_mhx"]] = positions[joints["r_leg_mhx"]];
							jointcommands.position[joints["r_leg_lhy"]] = positions[joints["r_leg_lhy"]];
							jointcommands.position[joints["r_leg_kny"]] = positions[joints["r_leg_kny"]];
							jointcommands.position[joints["r_leg_uay"]] = positions[joints["r_leg_uay"]];
							jointcommands.position[joints["r_leg_lax"]] = positions[joints["r_leg_lax"]];
							jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];//0.1*(traj_vec_srv.response.PositionArray.size()/2.0-ind*1.0)/traj_vec_srv.response.PositionArray.size();
						}else{
							if(!req.LinkToMove.compare("r_leg")){
								jointcommands.position[joints["r_leg_uhz"]] = p0_r + v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_uhz"]] = p0_l + v.response.q_right_dot[0];
								p0_r = jointcommands.position[joints["r_leg_uhz"]];
								jointcommands.position[joints["r_leg_mhx"]] = p1_r + v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_mhx"]] = p0_l + v.response.q_right_dot[1];
								p1_r = jointcommands.position[joints["r_leg_mhx"]];
								jointcommands.position[joints["r_leg_lhy"]] = p2_r + v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_lhy"]] = p0_l + v.response.q_right_dot[2];
								p2_r = jointcommands.position[joints["r_leg_lhy"]];
								jointcommands.position[joints["r_leg_kny"]] = p3_r + v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_kny"]] = p0_l + v.response.q_right_dot[3];
								p3_r = jointcommands.position[joints["r_leg_kny"]];
								jointcommands.position[joints["r_leg_uay"]] = p4_r + v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_uay"]] = p0_l + v.response.q_right_dot[4];
								p4_r = jointcommands.position[joints["r_leg_uay"]];
								jointcommands.position[joints["r_leg_lax"]] = p5_r + v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
								jointcommands.velocity[joints["r_leg_lax"]] = p0_l + v.response.q_right_dot[5];
								p5_r = jointcommands.position[joints["r_leg_lax"]];
								//ROS_INFO("Sent to left leg (%f, %f, %f, %f, %f, %f)", p0_r, p1_r, p2_r, p3_r, p4_r, p5_r);
								jointcommands.position[joints["l_leg_uhz"]] = positions[joints["l_leg_uhz"]];
								jointcommands.position[joints["l_leg_mhx"]] = positions[joints["l_leg_mhx"]];
								jointcommands.position[joints["l_leg_lhy"]] = positions[joints["l_leg_lhy"]];
								jointcommands.position[joints["l_leg_kny"]] = positions[joints["l_leg_kny"]];
								jointcommands.position[joints["l_leg_uay"]] = positions[joints["l_leg_uay"]];
								jointcommands.position[joints["l_leg_lax"]] = positions[joints["l_leg_lax"]];
								jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];//-0.1*(traj_vec_srv.response.PositionArray.size()/2.0-ind*1.0)/traj_vec_srv.response.PositionArray.size();
							}else{
								ROS_ERROR("Wrong link name");
								res.success = false;
								return false;
							}
						}
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
						current_dt += traj_vec_srv.response.dt[ind];
						pub_joint_commands_.publish(jointcommands);
						ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
					}else{
						ROS_ERROR("Could not reach gen legs velocity vector server");
						res.success = false;
						return false;
					}
				}
				res.success = true;
				return true;
			}else{
				ROS_ERROR("Could not reach walking vector server");
				res.success = false;
				return false;
			}
		}
		res.success = true;
		return true;
	}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_pelvis_service");
	ROS_INFO("starting move pelvis service");
	move_pelvis_service* c = new move_pelvis_service();
	ROS_INFO("started move pelvis service");

	ros::spin();
	return 0;
}



