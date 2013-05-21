#include "ros/ros.h"
#include "transformations.h"
#include <C43_LocalBodyCOM/Kinematics.h>
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "move_pelvis/move_pelvis.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <legs_val_calc/legs_val_calc.h>
#include <osrf_msgs/JointCommands.h>
#include <PoseController/foot_movement.h>
#include <PoseController/back_movement.h>
#include <tf/transform_listener.h>
#include <string>
#include <map>


class move_pelvis_service{

protected:
	ros::NodeHandle nh_, nh2_;
	ros::NodeHandle* rosnode;
	ros::ServiceServer move_pelvis_srv_;
	ros::ServiceClient traj_vector_cli_,walking_vector_cli_,walking_position_vector_cli_;
	ros::ServiceClient left_leg_val_calc_cli_,right_leg_val_calc_cli_;
	ros::Publisher traj_action_pub_;
	ros::Subscriber joint_states_sub_, imu_sub_, posecontroller_joint_states_sub_;
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	std::map <std::string, int> joints;
	std::vector<double> positions, posecontroller_positions;
	ros::Publisher pub_joint_commands_;
	geometry_msgs::Point imu;
	ros::ServiceClient posecontroller_cli;
	ros::ServiceClient delta_posecontroller_cli;
	ros::ServiceClient delta_back_posecontroller_cli;
	tf::TransformListener listener;

private:
	int _n;

public:
	move_pelvis_service()
	{
		rosnode = new ros::NodeHandle();
		ros::NodeHandle nh_private("~");
		traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("traj_vector_server");
		walking_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("walking_trajectory_vector");
		walking_position_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("walking_position_vector");

		while(!traj_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the traj_vector_server server");
		}

		while(!walking_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the walking_trajectory_vector server");
		}
		while(!walking_position_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the walking_trajectory_vector server");
		}

		posecontroller_cli = nh_.serviceClient<PoseController::foot_movement>("/PoseController/foot_movement");

		while(!posecontroller_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /PoseController/foot_movement server");
		}

		delta_posecontroller_cli = nh_.serviceClient<PoseController::foot_movement>("/PoseController/delta_foot_movement");

		while(!delta_posecontroller_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /PoseController/delta_foot_movement server");
		}

		delta_back_posecontroller_cli = nh_.serviceClient<PoseController::back_movement>("/PoseController/delta_back_movement");

		while(!delta_back_posecontroller_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /PoseController/delta_back_movement server");
		}

		left_leg_val_calc_cli_ = nh_.serviceClient<legs_val_calc::legs_val_calc>("LeftFoot2Pelvis_jacobian_srv");

		while(!left_leg_val_calc_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the LeftFoot2Pelvis_jacobian_srv server");
		}
		right_leg_val_calc_cli_ = nh_.serviceClient<legs_val_calc::legs_val_calc>("RightFoot2Pelvis_jacobian_srv");

		while(!right_leg_val_calc_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the RightFoot2Pelvis_jacobian_srv server");
		}
		//traj_vector_cli_ = nh_.subscribe("trajectory_vector", 1, &com_error_node::get_com_from_hrl_kinematics, this);
		//support_polygon_sub_ = nh_.subscribe("support_polygon", 1, &com_error_node::get_support_polygon_from_hrl_kinematics, this);

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&move_pelvis_service::joint_states_CB,this);
		posecontroller_joint_states_sub_ = nh_.subscribe("/PoseController/joint_states",100,&move_pelvis_service::posecontroller_joint_states_CB,this);
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
		imu.x = QuatToRoll(imuC->orientation.x,imuC->orientation.y,imuC->orientation.z,imuC->orientation.w);
		imu.y = QuatToPitch(imuC->orientation.x,imuC->orientation.y,imuC->orientation.z,imuC->orientation.w);
		imu.z = QuatToYaw(imuC->orientation.x,imuC->orientation.y,imuC->orientation.z,imuC->orientation.w);
	}

	//Get joint states from PoseController
	void posecontroller_joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		posecontroller_positions = state->position;
	}

	bool gen_traj(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		traj_splitter_to_vector::trajectory_vector traj_vec_srv;
		//control_msgs::FollowJointTrajectoryGoal goal;
		osrf_msgs::JointCommands jointcommands;
		std::vector<double> initial_positions = positions;
		ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_srv.request.Position = req.PositionDestination;
		traj_vec_srv.request.Angle = req.AngleDestination;
		traj_vec_srv.request.segments_number = 150;
		traj_vec_srv.request.total_time = 1.5 ;
		double kp = 10;

		tf::StampedTransform original_r_foot_transform;
		tf::StampedTransform original_l_foot_transform;
		try {
			listener.waitForTransform("/r_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/r_foot","/pelvis",ros::Time(0),original_r_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		try {
			listener.waitForTransform("/l_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/l_foot","/pelvis",ros::Time(0),original_l_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}


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
			traj_vec_srv.request.total_time = 5 ;
			if(walking_position_vector_cli_.call(traj_vec_srv)){
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
				int lookahead = 3;


				traj_splitter_to_vector::trajectory_vector cpy = traj_vec_srv;
				traj_splitter_to_vector::trajectory_vector cpy2 = traj_vec_srv;

				for(unsigned int ind = 0; ind < traj_vec_srv.response.PositionArray.size(); ind++){
					XYZRPY transform = VectorTranformation(	traj_vec_srv.response.PositionArray[ind].x, traj_vec_srv.response.PositionArray[ind].y, traj_vec_srv.response.PositionArray[ind].z,
															traj_vec_srv.response.AngleArray[ind].x, traj_vec_srv.response.AngleArray[ind].y, traj_vec_srv.response.AngleArray[ind].z,
															original_l_foot_transform.getOrigin().x(), original_l_foot_transform.getOrigin().y(), original_l_foot_transform.getOrigin().z(),
															QuatToRoll(original_l_foot_transform.getRotation()), QuatToPitch(original_l_foot_transform.getRotation()), QuatToYaw(original_l_foot_transform.getRotation()));

					cpy.response.PositionArray[ind].x = transform.x;
					cpy.response.PositionArray[ind].y = transform.y;
					cpy.response.PositionArray[ind].z = transform.z;
					cpy.response.AngleArray[ind].x = transform.roll;
					cpy.response.AngleArray[ind].y = transform.pitch;
					cpy.response.AngleArray[ind].z = transform.yaw;
					XYZRPY transform2 = VectorTranformation(	traj_vec_srv.response.PositionArray[ind].x, traj_vec_srv.response.PositionArray[ind].y, traj_vec_srv.response.PositionArray[ind].z,
																traj_vec_srv.response.AngleArray[ind].x, traj_vec_srv.response.AngleArray[ind].y, traj_vec_srv.response.AngleArray[ind].z,
																original_r_foot_transform.getOrigin().x(), original_r_foot_transform.getOrigin().y(), original_r_foot_transform.getOrigin().z(),
																QuatToRoll(original_r_foot_transform.getRotation()), QuatToPitch(original_r_foot_transform.getRotation()), QuatToYaw(original_r_foot_transform.getRotation()));

					cpy2.response.PositionArray[ind].x = transform2.x;
					cpy2.response.PositionArray[ind].y = transform2.y;
					cpy2.response.PositionArray[ind].z = transform2.z;
					cpy2.response.AngleArray[ind].x = transform2.roll;
					cpy2.response.AngleArray[ind].y = transform2.pitch;
					cpy2.response.AngleArray[ind].z = transform2.yaw;
				}






				for(unsigned int ind = 2; ind < traj_vec_srv.response.PositionArray.size()-2; ind++){
					PoseController::foot_movement move;
					ros::spinOnce();
					legs_val_calc::legs_val_calc v;
					//ROS_INFO("Response from trajectory vector service example: %f", traj_vec_srv.response.PositionArray[ind].x);
					/*XYZRPY transform = VectorTranformation(	traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
															traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
															original_l_foot_transform.getOrigin().x(), original_l_foot_transform.getOrigin().y(), original_l_foot_transform.getOrigin().z(),
															QuatToRoll(original_l_foot_transform.getRotation()), QuatToPitch(original_l_foot_transform.getRotation()), QuatToYaw(original_l_foot_transform.getRotation()));
*/
					tf::StampedTransform l_foot_transform;
					try {
						listener.waitForTransform("/l_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/l_foot","/pelvis",ros::Time(0),l_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}
/*
					v.request.x_dot = (transform.x - l_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.y_dot = (transform.y - l_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.z_dot = (transform.z - l_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.roll_dot = (transform.roll - QuatToRoll(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);// - imu.x;
					v.request.pitch_dot = (transform.pitch - QuatToPitch(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.yaw_dot = (transform.yaw - QuatToYaw(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);*/
					v.request.x_dot = (-cpy.response.PositionArray[ind+2].x + 8*cpy.response.PositionArray[ind+1].x - 8*cpy.response.PositionArray[ind-1].x + cpy.response.PositionArray[ind-2].x)/(12*cpy.response.dt[ind]);
					v.request.y_dot = (-cpy.response.PositionArray[ind+2].y + 8*cpy.response.PositionArray[ind+1].y - 8*cpy.response.PositionArray[ind-1].y + cpy.response.PositionArray[ind-2].y)/(12*cpy.response.dt[ind]);
					v.request.z_dot = (-cpy.response.PositionArray[ind+2].z + 8*cpy.response.PositionArray[ind+1].z - 8*cpy.response.PositionArray[ind-1].z + cpy.response.PositionArray[ind-2].z)/(12*cpy.response.dt[ind]);
					v.request.roll_dot = 	(-cpy.response.AngleArray[ind+2].x + 8*cpy.response.AngleArray[ind+1].x - 8*cpy.response.AngleArray[ind-1].x + cpy.response.AngleArray[ind-2].x)/(12*cpy.response.dt[ind]);
					v.request.pitch_dot = 	(-cpy.response.AngleArray[ind+2].y + 8*cpy.response.AngleArray[ind+1].y - 8*cpy.response.AngleArray[ind-1].y + cpy.response.AngleArray[ind-2].y)/(12*cpy.response.dt[ind]);
					v.request.yaw_dot = 	(-cpy.response.AngleArray[ind+2].z + 8*cpy.response.AngleArray[ind+1].z - 8*cpy.response.AngleArray[ind-1].z + cpy.response.AngleArray[ind-2].z)/(12*cpy.response.dt[ind]);

					if(left_leg_val_calc_cli_.call(v)){
						//trajectory_msgs::JointTrajectoryPoint p;

						/*jointcommands.name.push_back("l_leg_uhz");
        	    jointcommands.name.push_back("l_leg_mhx");
        	    jointcommands.name.push_back("l_leg_lhy");
        	    jointcommands.name.push_back("l_leg_kny");
        	    jointcommands.name.push_back("l_leg_uay");
        	    jointcommands.name.push_back("l_leg_lax");*/
						jointcommands.position[joints["l_leg_uhz"]] = posecontroller_positions[joints["l_leg_uhz"]] + v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
						p0_l = jointcommands.position[joints["l_leg_uhz"]];
						jointcommands.position[joints["l_leg_mhx"]] = posecontroller_positions[joints["l_leg_mhx"]] + v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
						p1_l = jointcommands.position[joints["l_leg_mhx"]];
						jointcommands.position[joints["l_leg_lhy"]] = posecontroller_positions[joints["l_leg_lhy"]] + v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
						p2_l = jointcommands.position[joints["l_leg_lhy"]];
						jointcommands.position[joints["l_leg_kny"]] = posecontroller_positions[joints["l_leg_kny"]] + v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
						p3_l = jointcommands.position[joints["l_leg_kny"]];
						jointcommands.position[joints["l_leg_uay"]] = posecontroller_positions[joints["l_leg_uay"]] + v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
						p4_l = jointcommands.position[joints["l_leg_uay"]];
						jointcommands.position[joints["l_leg_lax"]] = posecontroller_positions[joints["l_leg_lax"]] + v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
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

					}else{
						ROS_ERROR("Could not reach gen legs velocity vector server");
						res.success = false;
						return false;
					}

					/*transform = VectorTranformation(traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
													traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
													original_r_foot_transform.getOrigin().x(), original_r_foot_transform.getOrigin().y(), original_r_foot_transform.getOrigin().z(),
													QuatToRoll(original_r_foot_transform.getRotation()), QuatToPitch(original_r_foot_transform.getRotation()), QuatToYaw(original_r_foot_transform.getRotation()));
*/
					tf::StampedTransform r_foot_transform;
					try {
						listener.waitForTransform("/r_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/r_foot","/pelvis",ros::Time(0),r_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}
/*
					v.request.x_dot = (transform.x - r_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.y_dot = (transform.y - r_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.z_dot = (transform.z - r_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.roll_dot = (transform.roll - QuatToRoll(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);// - imu.x;
					v.request.pitch_dot = (transform.pitch - QuatToPitch(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.yaw_dot = (transform.yaw - QuatToYaw(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);*/
					v.request.x_dot = (-cpy2.response.PositionArray[ind+2].x + 8*cpy2.response.PositionArray[ind+1].x - 8*cpy2.response.PositionArray[ind-1].x + cpy2.response.PositionArray[ind-2].x)/(12*cpy2.response.dt[ind]);
					v.request.y_dot = (-cpy2.response.PositionArray[ind+2].y + 8*cpy2.response.PositionArray[ind+1].y - 8*cpy2.response.PositionArray[ind-1].y + cpy2.response.PositionArray[ind-2].y)/(12*cpy2.response.dt[ind]);
					v.request.z_dot = (-cpy2.response.PositionArray[ind+2].z + 8*cpy2.response.PositionArray[ind+1].z - 8*cpy2.response.PositionArray[ind-1].z + cpy2.response.PositionArray[ind-2].z)/(12*cpy2.response.dt[ind]);
					v.request.roll_dot = 	(-cpy2.response.AngleArray[ind+2].x + 8*cpy2.response.AngleArray[ind+1].x - 8*cpy2.response.AngleArray[ind-1].x + cpy2.response.AngleArray[ind-2].x)/(12*cpy2.response.dt[ind]);
					v.request.pitch_dot = 	(-cpy2.response.AngleArray[ind+2].y + 8*cpy2.response.AngleArray[ind+1].y - 8*cpy2.response.AngleArray[ind-1].y + cpy2.response.AngleArray[ind-2].y)/(12*cpy2.response.dt[ind]);
					v.request.yaw_dot = 	(-cpy2.response.AngleArray[ind+2].z + 8*cpy2.response.AngleArray[ind+1].z - 8*cpy2.response.AngleArray[ind-1].z + cpy2.response.AngleArray[ind-2].z)/(12*cpy2.response.dt[ind]);


					if(right_leg_val_calc_cli_.call(v)){
						jointcommands.position[joints["r_leg_uhz"]] = posecontroller_positions[joints["r_leg_uhz"]] + v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
						p0_r = jointcommands.position[joints["r_leg_uhz"]];
						jointcommands.position[joints["r_leg_mhx"]] = posecontroller_positions[joints["r_leg_mhx"]] + v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
						p1_r = jointcommands.position[joints["r_leg_mhx"]];
						jointcommands.position[joints["r_leg_lhy"]] = posecontroller_positions[joints["r_leg_lhy"]] + v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
						p2_r = jointcommands.position[joints["r_leg_lhy"]];
						jointcommands.position[joints["r_leg_kny"]] = posecontroller_positions[joints["r_leg_kny"]] + v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
						p3_r = jointcommands.position[joints["r_leg_kny"]];
						jointcommands.position[joints["r_leg_uay"]] = posecontroller_positions[joints["r_leg_uay"]] + v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
						p4_r = jointcommands.position[joints["r_leg_uay"]];
						jointcommands.position[joints["r_leg_lax"]] = posecontroller_positions[joints["r_leg_lax"]] + v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
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
						/*jointcommands.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
						jointcommands.position[joints["back_lbz"]] = positions[joints["back_lbz"]];*/
						//jointcommands.position[joints["back_mby"]] =  -imu.y;
						//jointcommands.position[joints["back_mby"]] =  -imu.y;
						jointcommands.kd_position[joints["back_mby"]] = 50;
						//jointcommands.position[joints["back_ubx"]] =  -imu.x;
						jointcommands.kd_position[joints["back_ubx"]] = 50;
						//ROS_INFO("%f, %f, %f, %f", jointcommands.position[24], jointcommands.position[25], jointcommands.position[26], jointcommands.position[27]);

						// To be reached 1+ind second after the start of the trajectory
						current_dt += traj_vec_srv.response.dt[ind];
						//pub_joint_commands_.publish(jointcommands);

						move.request.l_leg_uhz =  jointcommands.position[joints["l_leg_uhz"]];
						//move.request.l_leg_uhz_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.l_leg_mhx =  jointcommands.position[joints["l_leg_mhx"]];
						//move.request.l_leg_mhx_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.l_leg_lhy =  jointcommands.position[joints["l_leg_lhy"]];
						//move.request.l_leg_lhy_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.l_leg_kny =  jointcommands.position[joints["l_leg_kny"]];
						//move.request.l_leg_kny_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?3000:6000;
						move.request.l_leg_uay =  jointcommands.position[joints["l_leg_uay"]];
						//move.request.l_leg_uay_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.l_leg_lax =  jointcommands.position[joints["l_leg_lax"]];
						//move.request.l_leg_lax_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?4000:8000;
						move.request.r_leg_uhz =  jointcommands.position[joints["r_leg_uhz"]];
						//move.request.r_leg_uhz_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.r_leg_mhx =  jointcommands.position[joints["r_leg_mhx"]];
						//move.request.r_leg_mhx_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.r_leg_lhy =  jointcommands.position[joints["r_leg_lhy"]];
						//move.request.r_leg_lhy_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.r_leg_kny =  jointcommands.position[joints["r_leg_kny"]];
						//move.request.r_leg_kny_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?3000:6000;
						move.request.r_leg_uay =  jointcommands.position[joints["r_leg_uay"]];
						//move.request.r_leg_uay_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
						move.request.r_leg_lax =  jointcommands.position[joints["r_leg_lax"]];
						//move.request.r_leg_lax_kp_position =  1.0*(ind > traj_vec_srv.response.PositionArray.size()*0.9)?4000:8000;

						if(!posecontroller_cli.call(move)){
							ROS_ERROR("Could not reach PoseController server");
							res.success = false;
							return false;
						}
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
				ROS_ERROR("Could not reach trajectory vector server");
				res.success = false;
				return false;
			}
		}else{
			/*traj_vec_srv.request.total_time = 1.5 ;
			if(walking_position_vector_cli_.call(traj_vec_srv)){

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
					tf::StampedTransform r_foot_to_l;
					tf::StampedTransform l_foot_to_r;
					try {
						listener.waitForTransform("/l_foot","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/l_foot","/r_foot",ros::Time(0),r_foot_to_l);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}
					try {
						listener.waitForTransform("/r_foot","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/r_foot","/l_foot",ros::Time(0),l_foot_to_r);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}






					PoseController::foot_movement move;
					ros::spinOnce();
					legs_val_calc::legs_val_calc v;
					v.request.x_dot = traj_vec_srv.response.PositionArray[ind].x;
					v.request.y_dot = traj_vec_srv.response.PositionArray[ind].y;
					v.request.z_dot = traj_vec_srv.response.PositionArray[ind].z;
					v.request.roll_dot = traj_vec_srv.response.AngleArray[ind].x;// - imu.x;
					v.request.pitch_dot = traj_vec_srv.response.AngleArray[ind].y;// - imu.y;
					v.request.yaw_dot = traj_vec_srv.response.AngleArray[ind].z;// - imu.z;

					PoseController::back_movement back;
					if(!req.LinkToMove.compare("l_leg")){


						if(left_leg_val_calc_cli_.call(v)){
							jointcommands.position[joints["l_leg_uhz"]] = v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
							p0_l = jointcommands.position[joints["l_leg_uhz"]];
							jointcommands.position[joints["l_leg_mhx"]] = v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
							p1_l = jointcommands.position[joints["l_leg_mhx"]];
							jointcommands.position[joints["l_leg_lhy"]] = v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
							p2_l = jointcommands.position[joints["l_leg_lhy"]];
							jointcommands.position[joints["l_leg_kny"]] = v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
							p3_l = jointcommands.position[joints["l_leg_kny"]];
							jointcommands.position[joints["l_leg_uay"]] = v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
							p4_l = jointcommands.position[joints["l_leg_uay"]];
							jointcommands.position[joints["l_leg_lax"]] = v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
							p5_l = jointcommands.position[joints["l_leg_lax"]];
							//ROS_INFO("Sent to left leg (%f, %f, %f, %f, %f, %f)", p0_l, p1_l, p2_l, p3_l, p4_l, p5_l);
							jointcommands.position[joints["r_leg_uhz"]] = v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
							p0_r = jointcommands.position[joints["r_leg_uhz"]];
							jointcommands.position[joints["r_leg_mhx"]] = v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
							p1_r = jointcommands.position[joints["r_leg_mhx"]];
							jointcommands.position[joints["r_leg_lhy"]] = v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
							p2_r = jointcommands.position[joints["r_leg_lhy"]];
							jointcommands.position[joints["r_leg_kny"]] = v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
							p3_r = jointcommands.position[joints["r_leg_kny"]];
							jointcommands.position[joints["r_leg_uay"]] = v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
							p4_r = jointcommands.position[joints["r_leg_uay"]];
							jointcommands.position[joints["r_leg_lax"]] = v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
							p5_r = jointcommands.position[joints["r_leg_lax"]];
						}else{
							ROS_ERROR("Could not reach gen left leg velocity vector server");
							res.success = false;
							return false;
						}
					}else{
						if(!req.LinkToMove.compare("r_leg")){

							tf::StampedTransform l_foot_transform;
							try {
								listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
								listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
							} catch (tf::TransformException &ex) {
								ROS_ERROR("%s",ex.what());
							}

							legs_val_calc::legs_val_calc tfv;
							tfv.request.x_dot = original_l_foot_transform.getOrigin().x() - l_foot_transform.getOrigin().x();
							tfv.request.y_dot = original_l_foot_transform.getOrigin().x() - l_foot_transform.getOrigin().x();
							tfv.request.z_dot = original_l_foot_transform.getOrigin().x() - l_foot_transform.getOrigin().x();
							tfv.request.roll_dot = QuatToRoll(original_l_foot_transform.getRotation().x(),original_l_foot_transform.getRotation().y(),original_l_foot_transform.getRotation().z(),original_l_foot_transform.getRotation().w()) - QuatToRoll(l_foot_transform.getRotation().x(),l_foot_transform.getRotation().y(),l_foot_transform.getRotation().z(),l_foot_transform.getRotation().w());// - imu.x;
							tfv.request.pitch_dot = QuatToPitch(original_l_foot_transform.getRotation().x(),original_l_foot_transform.getRotation().y(),original_l_foot_transform.getRotation().z(),original_l_foot_transform.getRotation().w()) - QuatToPitch(l_foot_transform.getRotation().x(),l_foot_transform.getRotation().y(),l_foot_transform.getRotation().z(),l_foot_transform.getRotation().w());// - imu.y;
							tfv.request.yaw_dot =QuatToYaw(original_l_foot_transform.getRotation().x(),original_l_foot_transform.getRotation().y(),original_l_foot_transform.getRotation().z(),original_l_foot_transform.getRotation().w()) - QuatToYaw(l_foot_transform.getRotation().x(),l_foot_transform.getRotation().y(),l_foot_transform.getRotation().z(),l_foot_transform.getRotation().w());// - imu.z;

							if(!legs_val_calc_cli_.call(tfv)){
								ROS_ERROR("Could not reach gen leg velocity vector server");
								res.success = false;
								return false;
							}

							if(legs_val_calc_cli_.call(v)){
								jointcommands.position[joints["r_leg_uhz"]] = v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
								p0_r = jointcommands.position[joints["r_leg_uhz"]];
								jointcommands.position[joints["r_leg_mhx"]] = v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
								p1_r = jointcommands.position[joints["r_leg_mhx"]];
								jointcommands.position[joints["r_leg_lhy"]] = v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
								p2_r = jointcommands.position[joints["r_leg_lhy"]];
								jointcommands.position[joints["r_leg_kny"]] = v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
								p3_r = jointcommands.position[joints["r_leg_kny"]];
								jointcommands.position[joints["r_leg_uay"]] = v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
								p4_r = jointcommands.position[joints["r_leg_uay"]];
								jointcommands.position[joints["r_leg_lax"]] = v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
								p5_r = jointcommands.position[joints["r_leg_lax"]];
								jointcommands.position[joints["l_leg_uhz"]] = tfv.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_uhz"]];
								jointcommands.position[joints["l_leg_mhx"]] = tfv.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_mhx"]];
								jointcommands.position[joints["l_leg_lhy"]] = tfv.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_lhy"]];
								jointcommands.position[joints["l_leg_kny"]] = tfv.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_kny"]];
								jointcommands.position[joints["l_leg_uay"]] = tfv.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_uay"]];
								jointcommands.position[joints["l_leg_lax"]] = tfv.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];;//initial_positions[joints["l_leg_lax"]];

							}else{
								ROS_ERROR("Could not reach gen right leg velocity vector server");
								res.success = false;
								return false;
							}
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
					//jointcommands.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
					//jointcommands.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
					jointcommands.position[joints["back_mby"]] =  -imu.y;
					jointcommands.kd_position[joints["back_mby"]] = 50;
					jointcommands.position[joints["back_ubx"]] =  -imu.x;
					jointcommands.kd_position[joints["back_ubx"]] = 50;
					current_dt += traj_vec_srv.response.dt[ind];
					//pub_joint_commands_.publish(jointcommands);

					move.request.l_leg_uhz =  jointcommands.position[joints["l_leg_uhz"]];
					//move.request.l_leg_uhz_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.l_leg_mhx =  jointcommands.position[joints["l_leg_mhx"]];
					//move.request.l_leg_mhx_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.l_leg_lhy =  jointcommands.position[joints["l_leg_lhy"]];
					//move.request.l_leg_lhy_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.l_leg_kny =  jointcommands.position[joints["l_leg_kny"]];
					//move.request.l_leg_kny_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?3000:6000;
					move.request.l_leg_uay =  jointcommands.position[joints["l_leg_uay"]];
					//move.request.l_leg_uay_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.l_leg_lax =  jointcommands.position[joints["l_leg_lax"]];
					//move.request.l_leg_lax_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2000:4000;
					move.request.r_leg_uhz =  jointcommands.position[joints["r_leg_uhz"]];
					//move.request.r_leg_uhz_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.r_leg_mhx =  jointcommands.position[joints["r_leg_mhx"]];
					//move.request.r_leg_mhx_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.r_leg_lhy =  jointcommands.position[joints["r_leg_lhy"]];
					//move.request.r_leg_lhy_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.r_leg_kny =  jointcommands.position[joints["r_leg_kny"]];
					//move.request.r_leg_kny_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?3000:6000;
					move.request.r_leg_uay =  jointcommands.position[joints["r_leg_uay"]];
					//move.request.r_leg_uay_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2500:5000;
					move.request.r_leg_lax =  jointcommands.position[joints["r_leg_lax"]];
					//move.request.r_leg_lax_kp_position =  (ind > traj_vec_srv.response.PositionArray.size()*0.9)?2000:4000;

					if(!delta_posecontroller_cli.call(move)){
						ROS_ERROR("Could not reach PoseController server");
						res.success = false;
						return false;
					}
					if(!delta_back_posecontroller_cli.call(back)){
						ROS_ERROR("Could not reach PoseController server");
						res.success = false;
						return false;
					}
					ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
				}
				res.success = true;
				return true;
			}else{
				ROS_ERROR("Could not reach walking vector server");
				res.success = false;
				return false;
			}*/
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



