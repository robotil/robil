#include "ros/ros.h"
#include "transformations.h"
#include <atlas_msgs/ForceTorqueSensors.h>
#include <ros/subscribe_options.h>
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "move_pelvis/move_pelvis.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <legs_val_calc/legs_val_calc.h>
#include <PoseController/foot_movement.h>
#include <PoseController/back_movement.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <string>
#include <map>
#include <cmath>
#include <C43_LocalBodyCOM/Kinematics.h>

class walk_legs_service{

protected:
	ros::NodeHandle nh_, nh2_;
	ros::NodeHandle* rosnode;
	ros::ServiceServer walk_legs_srv_, step_down_srv_, make_step_srv_, walk_legs_by_pelvis_srv_, move_leg_by_pelvis_srv_;
	ros::ServiceClient traj_vector_cli_,walking_vector_cli_,walking_position_vector_cli_;
	ros::ServiceClient legs_val_calc_cli_,left_leg_val_calc_cli_,right_leg_val_calc_cli_;
	ros::Publisher traj_action_pub_;
	//ros::Subscriber l_leg_uhz_sub_,l_leg_mhx_sub_,l_leg_lhy_sub_,l_leg_kny_sub_,l_leg_uay_sub_,l_leg_lax_sub_;
	//ros::Subscriber r_leg_uhz_sub_,r_leg_mhx_sub_,r_leg_lhy_sub_,r_leg_kny_sub_,r_leg_uay_sub_,r_leg_lax_sub_;
	ros::Subscriber joint_states_sub_, imu_sub_,posecontroller_joint_states_sub_;
	ros::Subscriber force_sub_;
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	std::map <std::string, int> joints;
	std::vector<double> positions, posecontroller_positions;
	ros::Publisher pub_joint_commands_;
	geometry_msgs::Point imu;
	ros::ServiceClient posecontroller_cli, reset_posecontroller_cli;
	ros::ServiceClient delta_posecontroller_cli;
	ros::ServiceClient delta_back_posecontroller_cli;
	atlas_msgs::ForceTorqueSensors forcesensors;

	tf::TransformListener listener;

private:
	int _n;

public:
	walk_legs_service()
	{
		rosnode = new ros::NodeHandle();
		ros::NodeHandle nh_private("~");
		traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("traj_vector_server");
		walking_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("walking_trajectory_vector");
		walking_position_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("walking_position_vector");
		force_sub_ = nh_.subscribe(/*"/atlas/force_torque_sensors"*/"/filtered_contacts",100,&walk_legs_service::force_CB,this);

		while(force_sub_.getNumPublishers() == 0){
			std::string c = std::string("Waiting for the ")+ force_sub_.getTopic() + " topic";
			ROS_INFO(c.c_str());
			ros::Duration(0.1).sleep();
		}

		while(!traj_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the traj_vector_server server");
		}

		while(!walking_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the walking_trajectory_vector server");
		}
		while(!walking_position_vector_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the walking_trajectory_vector server");
		}


		legs_val_calc_cli_ = nh_.serviceClient<legs_val_calc::legs_val_calc>("walk_legs_val_calc_srv");

		while(!legs_val_calc_cli_.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the legs_val_calc_srv server");
		}

		posecontroller_cli = nh_.serviceClient<PoseController::foot_movement>("/PoseController/foot_movement");

		while(!posecontroller_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /PoseController/foot_movement server");
		}

		reset_posecontroller_cli = nh_.serviceClient<std_srvs::Empty>("/PoseController/reset_joints");

		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&walk_legs_service::joint_states_CB,this);
		posecontroller_joint_states_sub_ = nh_.subscribe("/PoseController/joint_states",100,&walk_legs_service::posecontroller_joint_states_CB,this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100,&walk_legs_service::imu_CB,this);


		walk_legs_srv_ = nh_.advertiseService("walk_legs", &walk_legs_service::gen_traj, this);
		walk_legs_by_pelvis_srv_ = nh_.advertiseService("walk_legs_by_pelvis", &walk_legs_service::walk_legs_by_pelvis, this);
		move_leg_by_pelvis_srv_ = nh_.advertiseService("move_leg_by_pelvis", &walk_legs_service::move_leg_by_pelvis, this);
		step_down_srv_ = nh2_.advertiseService("step_down", &walk_legs_service::step_down, this);
		make_step_srv_ = nh2_.advertiseService("make_step", &walk_legs_service::make_step, this);
		ROS_INFO("running walk_legs service");
	}
	~walk_legs_service(){
		//delete traj_client_;
	}

	//Get foot contact
	void force_CB(const atlas_msgs::ForceTorqueSensorsConstPtr& contact){
		forcesensors.l_foot = contact->l_foot;
		forcesensors.r_foot = contact->r_foot;
	}

	//Get joint states
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

	//Get joint states from PoseController
	void posecontroller_joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		posecontroller_positions = state->position;
	}


	void imu_CB(const sensor_msgs::ImuConstPtr& imuC){
		imu.x = QuatToRoll(imuC->orientation);
		imu.y = QuatToPitch(imuC->orientation);
		imu.z = QuatToYaw(imuC->orientation);
	}

	bool gen_traj(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		traj_splitter_to_vector::trajectory_vector traj_vec_srv;
		ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_srv.request.Position = req.PositionDestination;
		traj_vec_srv.request.Angle = req.AngleDestination;
		traj_vec_srv.request.segments_number = 50 ;
		traj_vec_srv.request.total_time = 2 ;

		tf::StampedTransform original_r_foot_transform;
		tf::StampedTransform original_l_foot_transform;
		try {
			listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),original_r_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		try {
			listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),original_l_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}


		if(!req.LinkToMove.compare("l_leg")){


			traj_vec_srv.request.Position.x = req.PositionDestination.x;
			traj_vec_srv.request.Position.y = req.PositionDestination.y;
			traj_vec_srv.request.Position.z = req.PositionDestination.z;
			traj_vec_srv.request.Angle.x = req.AngleDestination.x;
			traj_vec_srv.request.Angle.y = req.AngleDestination.y;
			traj_vec_srv.request.Angle.z = req.AngleDestination.z;

			if(walking_position_vector_cli_.call(traj_vec_srv)){

				tf::StampedTransform original_r_foot_to_l;
				try {
					listener.waitForTransform("/r_foot","/l_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/r_foot","/l_foot",ros::Time(0),original_r_foot_to_l);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}
				for(unsigned i=0; i < traj_vec_srv.request.segments_number; i++){
					XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[i].x, traj_vec_srv.response.PositionArray[i].y, traj_vec_srv.response.PositionArray[i].z,
							traj_vec_srv.response.AngleArray[i].x, traj_vec_srv.response.AngleArray[i].y, traj_vec_srv.response.AngleArray[i].z,
							original_r_foot_to_l.getOrigin().x(), original_r_foot_to_l.getOrigin().y(), original_r_foot_to_l.getOrigin().z(),
							QuatToRoll(original_r_foot_to_l.getRotation()), QuatToPitch(original_r_foot_to_l.getRotation()), QuatToYaw(original_r_foot_to_l.getRotation()));
					traj_vec_srv.response.PositionArray[i].x = tranform.x;
					traj_vec_srv.response.PositionArray[i].y = tranform.y;
					traj_vec_srv.response.PositionArray[i].z = tranform.z;
					traj_vec_srv.response.AngleArray[i].x = tranform.roll;
					traj_vec_srv.response.AngleArray[i].y = tranform.pitch;
					traj_vec_srv.response.AngleArray[i].z = tranform.yaw;
				}

				int lookahead = 3;


				tf::StampedTransform first_r_foot_transform;
				try {
					listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),first_r_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				for(unsigned int ind = 1; ind < traj_vec_srv.response.PositionArray.size()-lookahead; ind++){
					ros::spinOnce();

					tf::StampedTransform l_foot_to_r;
					try {
						listener.waitForTransform("/l_foot","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/l_foot","/r_foot",ros::Time(0),l_foot_to_r);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					tf::StampedTransform r_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}
					XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
							traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
							first_r_foot_transform.getOrigin().x(), first_r_foot_transform.getOrigin().y(), first_r_foot_transform.getOrigin().z(),
							QuatToRoll(first_r_foot_transform.getRotation()), QuatToPitch(first_r_foot_transform.getRotation()), QuatToYaw(first_r_foot_transform.getRotation()));

					double newx = tranform.x;
					double newy = tranform.y;
					double newz = tranform.z;
					double newroll = tranform.roll;
					double newpitch = tranform.pitch;
					double newyaw = tranform.yaw;


					tf::StampedTransform l_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					PoseController::foot_movement move;
					ros::spinOnce();
					legs_val_calc::legs_val_calc v;
					v.request.x_dot = (newx - l_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.y_dot = (newy - l_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.z_dot = (newz - l_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.roll_dot = (newroll - QuatToRoll(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.pitch_dot = (newpitch - QuatToPitch(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.yaw_dot = (newyaw - QuatToYaw(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);

					legs_val_calc::legs_val_calc tfv;
					tfv.request.x_dot = original_r_foot_transform.getOrigin().x() - r_foot_transform.getOrigin().x();
					tfv.request.y_dot = original_r_foot_transform.getOrigin().y() - r_foot_transform.getOrigin().y();
					tfv.request.z_dot = original_r_foot_transform.getOrigin().z() - r_foot_transform.getOrigin().z();
					tfv.request.roll_dot = QuatToRoll(original_r_foot_transform.getRotation()) - QuatToRoll(r_foot_transform.getRotation());
					tfv.request.pitch_dot = QuatToPitch(original_r_foot_transform.getRotation()) - QuatToPitch(r_foot_transform.getRotation());
					tfv.request.yaw_dot = QuatToYaw(original_r_foot_transform.getRotation()) - QuatToYaw(r_foot_transform.getRotation());

					/*if(!legs_val_calc_cli_.call(tfv)){
						ROS_ERROR("Could not reach gen leg velocity vector server");
						res.success = false;
						return false;
					}*/

					if(legs_val_calc_cli_.call(v)){

						move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]] + v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]] + v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]] + v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]] + v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]] + v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]] + v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]] + tfv.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]] + tfv.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]] + tfv.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]] + tfv.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]] + tfv.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]] + tfv.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];

						if(!posecontroller_cli.call(move)){
							ROS_ERROR("Could not reach PoseController server");
							res.success = false;
							return false;
						}
					}else{
						ROS_ERROR("Could not reach gen leg velocity vector server");
						res.success = false;
						return false;
					}
					ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
				}
			}else{
				ROS_ERROR("Could not reach walking_position_vector server");
				res.success = false;
				return false;
			}
		}else{
			if(!req.LinkToMove.compare("r_leg")){
				tf::StampedTransform original_l_foot_to_r;
				traj_vec_srv.request.Position.x = req.PositionDestination.x;
				traj_vec_srv.request.Position.y = req.PositionDestination.y;
				traj_vec_srv.request.Position.z = req.PositionDestination.z;
				traj_vec_srv.request.Angle.x = req.AngleDestination.x;
				traj_vec_srv.request.Angle.y = req.AngleDestination.y;
				traj_vec_srv.request.Angle.z = req.AngleDestination.z;

				if(walking_position_vector_cli_.call(traj_vec_srv)){

					try {
						listener.waitForTransform("/l_foot","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/l_foot","/r_foot",ros::Time(0),original_l_foot_to_r);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					for(unsigned i=0; i < traj_vec_srv.request.segments_number; i++){
						XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[i].x, traj_vec_srv.response.PositionArray[i].y, traj_vec_srv.response.PositionArray[i].z,
								traj_vec_srv.response.AngleArray[i].x, traj_vec_srv.response.AngleArray[i].y, traj_vec_srv.response.AngleArray[i].z,
								original_l_foot_to_r.getOrigin().x(), original_l_foot_to_r.getOrigin().y(), original_l_foot_to_r.getOrigin().z(),
								QuatToRoll(original_l_foot_to_r.getRotation()), QuatToPitch(original_l_foot_to_r.getRotation()), QuatToYaw(original_l_foot_to_r.getRotation()));
						traj_vec_srv.response.PositionArray[i].x = tranform.x;
						traj_vec_srv.response.PositionArray[i].y = tranform.y;
						traj_vec_srv.response.PositionArray[i].z = tranform.z;
						traj_vec_srv.response.AngleArray[i].x = tranform.roll;
						traj_vec_srv.response.AngleArray[i].y = tranform.pitch;
						traj_vec_srv.response.AngleArray[i].z = tranform.yaw;
					}

					int lookahead = 3;


					tf::StampedTransform first_l_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),first_l_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					for(unsigned int ind = 1; ind < traj_vec_srv.response.PositionArray.size()-lookahead; ind++){
						ros::spinOnce();
						tf::StampedTransform r_foot_to_l;
						try {
							listener.waitForTransform("/r_foot","/l_foot",ros::Time(0),ros::Duration(0.2));
							listener.lookupTransform("/r_foot","/l_foot",ros::Time(0),r_foot_to_l);
						} catch (tf::TransformException &ex) {
							ROS_ERROR("%s",ex.what());
						}
						tf::StampedTransform l_foot_transform;
						try {
							listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
							listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
						} catch (tf::TransformException &ex) {
							ROS_ERROR("%s",ex.what());
						}

						XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
								traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
								first_l_foot_transform.getOrigin().x(), first_l_foot_transform.getOrigin().y(), first_l_foot_transform.getOrigin().z(),
								QuatToRoll(first_l_foot_transform.getRotation()), QuatToPitch(first_l_foot_transform.getRotation()), QuatToYaw(first_l_foot_transform.getRotation()));

						double newx = tranform.x;
						double newy = tranform.y;
						double newz = tranform.z;
						double newroll = tranform.roll;
						double newpitch = tranform.pitch;
						double newyaw = tranform.yaw;

						tf::StampedTransform r_foot_transform;
						try {
							listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
							listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
						} catch (tf::TransformException &ex) {
							ROS_ERROR("%s",ex.what());
						}

						PoseController::foot_movement move;
						ros::spinOnce();
						legs_val_calc::legs_val_calc v;
						v.request.x_dot = (newx - r_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.PositionArray[ind].x + original_l_foot_to_r.getOrigin().x() - l_foot_to_r.getOrigin().x())/traj_vec_srv.response.dt[ind];
						v.request.y_dot = (newy - r_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.PositionArray[ind].y + original_l_foot_to_r.getOrigin().y() - l_foot_to_r.getOrigin().y())/traj_vec_srv.response.dt[ind];
						v.request.z_dot = (newz - r_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.PositionArray[ind].z + original_l_foot_to_r.getOrigin().z() - l_foot_to_r.getOrigin().z())/traj_vec_srv.response.dt[ind];
						v.request.roll_dot = (newroll - QuatToRoll(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.AngleArray[ind].x + QuatToRoll(original_l_foot_to_r.getRotation()) - QuatToRoll(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.x;
						v.request.pitch_dot = (newpitch - QuatToPitch(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.AngleArray[ind].y + QuatToPitch(original_l_foot_to_r.getRotation()) - QuatToPitch(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.y;
						v.request.yaw_dot = (newyaw - QuatToYaw(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);//(traj_vec_srv.response.AngleArray[ind].z + QuatToYaw(original_l_foot_to_r.getRotation()) - QuatToYaw(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.z;

						legs_val_calc::legs_val_calc tfv;
						tfv.request.x_dot = original_l_foot_transform.getOrigin().x() - l_foot_transform.getOrigin().x();
						tfv.request.y_dot = original_l_foot_transform.getOrigin().y() - l_foot_transform.getOrigin().y();
						tfv.request.z_dot = original_l_foot_transform.getOrigin().z() - l_foot_transform.getOrigin().z();
						tfv.request.roll_dot = QuatToRoll(original_l_foot_transform.getRotation()) - QuatToRoll(l_foot_transform.getRotation());// - imu.x;
						tfv.request.pitch_dot = QuatToPitch(original_l_foot_transform.getRotation()) - QuatToPitch(l_foot_transform.getRotation());// - imu.y;
						tfv.request.yaw_dot = QuatToYaw(original_l_foot_transform.getRotation()) - QuatToYaw(l_foot_transform.getRotation());// - imu.z;

						/*if(!legs_val_calc_cli_.call(tfv)){
							ROS_ERROR("Could not reach gen leg velocity vector server");
							res.success = false;
							return false;
						}*/

						if(legs_val_calc_cli_.call(v)){
							move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]] + v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]] + v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]] + v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]] + v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]] + v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]] + v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]] + tfv.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]] + tfv.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]] + tfv.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]] + tfv.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]] + tfv.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]] + tfv.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];

							if(!posecontroller_cli.call(move)){
								ROS_ERROR("Could not reach PoseController server");
								res.success = false;
								return false;
							}
						}else{
							ROS_ERROR("Could not reach gen leg velocity vector server");
							res.success = false;
							return false;
						}
						ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
					}
				}else{
					ROS_ERROR("Could not reach walking_position_vector server");
					res.success = false;
					return false;
				}
			}else{
				ROS_ERROR("Wrong link name");
				res.success = false;
				return false;
			}
		}

		res.success = true;
		return true;

	}

	bool move_leg_by_pelvis(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		traj_splitter_to_vector::trajectory_vector traj_vec_srv;
		ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_srv.request.Position = req.PositionDestination;
		traj_vec_srv.request.Angle = req.AngleDestination;
		traj_vec_srv.request.segments_number = 50 ;
		traj_vec_srv.request.total_time = 2 ;

		tf::StampedTransform original_r_foot_transform;
		tf::StampedTransform original_l_foot_transform;
		try {
			listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),original_r_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		try {
			listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),original_l_foot_transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}


		if(!req.LinkToMove.compare("l_leg")){
			traj_vec_srv.request.Position.x = req.PositionDestination.x;
			traj_vec_srv.request.Position.y = req.PositionDestination.y;
			traj_vec_srv.request.Position.z = req.PositionDestination.z;
			traj_vec_srv.request.Angle.x = req.AngleDestination.x;
			traj_vec_srv.request.Angle.y = req.AngleDestination.y;
			traj_vec_srv.request.Angle.z = req.AngleDestination.z;

			XYZRPY tranform = VectorTransposeTranformation(	traj_vec_srv.request.Position.x, traj_vec_srv.request.Position.y, traj_vec_srv.request.Position.z,
					traj_vec_srv.request.Angle.x, traj_vec_srv.request.Angle.y, traj_vec_srv.request.Angle.z,
					original_l_foot_transform.getOrigin().x(), original_l_foot_transform.getOrigin().y(), original_l_foot_transform.getOrigin().z(),
					QuatToRoll(original_l_foot_transform.getRotation()), QuatToPitch(original_l_foot_transform.getRotation()), QuatToYaw(original_l_foot_transform.getRotation()));

			traj_vec_srv.request.Position.x = tranform.x;
			traj_vec_srv.request.Position.y = tranform.y;
			traj_vec_srv.request.Position.z = tranform.z;
			traj_vec_srv.request.Angle.x = tranform.roll;
			traj_vec_srv.request.Angle.y = tranform.pitch;
			traj_vec_srv.request.Angle.z = tranform.yaw;

			if(walking_position_vector_cli_.call(traj_vec_srv)){
				int lookahead = 3;

				tf::StampedTransform first_l_foot_transform;
				try {
					listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),first_l_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				for(unsigned int ind = 1; ind < traj_vec_srv.response.PositionArray.size()-lookahead; ind++){
					ros::spinOnce();

					XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
							traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
							first_l_foot_transform.getOrigin().x(), first_l_foot_transform.getOrigin().y(), first_l_foot_transform.getOrigin().z(),
							QuatToRoll(first_l_foot_transform.getRotation()), QuatToPitch(first_l_foot_transform.getRotation()), QuatToYaw(first_l_foot_transform.getRotation()));

					double newx = tranform.x;
					double newy = tranform.y;
					double newz = tranform.z;
					double newroll = tranform.roll;
					double newpitch = tranform.pitch;
					double newyaw = tranform.yaw;


					tf::StampedTransform l_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					PoseController::foot_movement move;
					ros::spinOnce();
					legs_val_calc::legs_val_calc v;
					v.request.x_dot = (newx - l_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.y_dot = (newy - l_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.z_dot = (newz - l_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.roll_dot = (newroll - QuatToRoll(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.pitch_dot = (newpitch - QuatToPitch(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
					v.request.yaw_dot = (newyaw - QuatToYaw(l_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);

					if(legs_val_calc_cli_.call(v)){

						move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]] + v.response.q_left_dot[0]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]] + v.response.q_left_dot[1]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]] + v.response.q_left_dot[2]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]] + v.response.q_left_dot[3]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]] + v.response.q_left_dot[4]*traj_vec_srv.response.dt[ind];
						move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]] + v.response.q_left_dot[5]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]];// + tfv.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]];// + tfv.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]];// + tfv.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]];// + tfv.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]];// + tfv.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
						move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]];// + tfv.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];

						if(!posecontroller_cli.call(move)){
							ROS_ERROR("Could not reach PoseController server");
							res.success = false;
							return false;
						}
					}else{
						ROS_ERROR("Could not reach gen leg velocity vector server");
						res.success = false;
						return false;
					}
					ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
				}
			}else{
				ROS_ERROR("Could not reach walking_position_vector server");
				res.success = false;
				return false;
			}
		}else{
			if(!req.LinkToMove.compare("r_leg")){
				traj_vec_srv.request.Position.x = req.PositionDestination.x;
				traj_vec_srv.request.Position.y = req.PositionDestination.y;
				traj_vec_srv.request.Position.z = req.PositionDestination.z;
				traj_vec_srv.request.Angle.x = req.AngleDestination.x;
				traj_vec_srv.request.Angle.y = req.AngleDestination.y;
				traj_vec_srv.request.Angle.z = req.AngleDestination.z;

				XYZRPY tranform = VectorTransposeTranformation(	traj_vec_srv.request.Position.x, traj_vec_srv.request.Position.y, traj_vec_srv.request.Position.z,
						traj_vec_srv.request.Angle.x, traj_vec_srv.request.Angle.y, traj_vec_srv.request.Angle.z,
						original_r_foot_transform.getOrigin().x(), original_r_foot_transform.getOrigin().y(), original_r_foot_transform.getOrigin().z(),
						QuatToRoll(original_r_foot_transform.getRotation()), QuatToPitch(original_r_foot_transform.getRotation()), QuatToYaw(original_r_foot_transform.getRotation()));

				traj_vec_srv.request.Position.x = tranform.x;
				traj_vec_srv.request.Position.y = tranform.y;
				traj_vec_srv.request.Position.z = tranform.z;
				traj_vec_srv.request.Angle.x = tranform.roll;
				traj_vec_srv.request.Angle.y = tranform.pitch;
				traj_vec_srv.request.Angle.z = tranform.yaw;

				if(walking_position_vector_cli_.call(traj_vec_srv)){
					int lookahead = 3;

					tf::StampedTransform first_r_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),first_r_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					for(unsigned int ind = 1; ind < traj_vec_srv.response.PositionArray.size()-lookahead; ind++){
						ros::spinOnce();

						XYZRPY tranform = VectorTranformation(	traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.PositionArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
								traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].x, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].y, traj_vec_srv.response.AngleArray[std::min((unsigned int)traj_vec_srv.response.PositionArray.size(), ind+lookahead)].z,
								first_r_foot_transform.getOrigin().x(), first_r_foot_transform.getOrigin().y(), first_r_foot_transform.getOrigin().z(),
								QuatToRoll(first_r_foot_transform.getRotation()), QuatToPitch(first_r_foot_transform.getRotation()), QuatToYaw(first_r_foot_transform.getRotation()));

						double newx = tranform.x;
						double newy = tranform.y;
						double newz = tranform.z;
						double newroll = tranform.roll;
						double newpitch = tranform.pitch;
						double newyaw = tranform.yaw;

						tf::StampedTransform r_foot_transform;
						try {
							listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
							listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
						} catch (tf::TransformException &ex) {
							ROS_ERROR("%s",ex.what());
						}

						PoseController::foot_movement move;
						ros::spinOnce();
						legs_val_calc::legs_val_calc v;
						v.request.x_dot = (newx - r_foot_transform.getOrigin().x())/(lookahead*traj_vec_srv.response.dt[ind]);
						v.request.y_dot = (newy - r_foot_transform.getOrigin().y())/(lookahead*traj_vec_srv.response.dt[ind]);
						v.request.z_dot = (newz - r_foot_transform.getOrigin().z())/(lookahead*traj_vec_srv.response.dt[ind]);
						v.request.roll_dot = (newroll - QuatToRoll(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
						v.request.pitch_dot = (newpitch - QuatToPitch(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);
						v.request.yaw_dot = (newyaw - QuatToYaw(r_foot_transform.getRotation()))/(lookahead*traj_vec_srv.response.dt[ind]);

						if(legs_val_calc_cli_.call(v)){

							move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]] + v.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]] + v.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]] + v.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]] + v.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]] + v.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
							move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]] + v.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]];// + tfv.response.q_right_dot[0]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]];// + tfv.response.q_right_dot[1]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]];// + tfv.response.q_right_dot[2]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]];// + tfv.response.q_right_dot[3]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]];// + tfv.response.q_right_dot[4]*traj_vec_srv.response.dt[ind];
							move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]];// + tfv.response.q_right_dot[5]*traj_vec_srv.response.dt[ind];

							if(!posecontroller_cli.call(move)){
								ROS_ERROR("Could not reach PoseController server");
								res.success = false;
								return false;
							}
						}else{
							ROS_ERROR("Could not reach gen leg velocity vector server");
							res.success = false;
							return false;
						}
						ros::Duration(traj_vec_srv.response.dt[ind]).sleep();
					}
				}else{
					ROS_ERROR("Could not reach walking_position_vector server");
					res.success = false;
					return false;
				}
			}else{
				ROS_ERROR("Wrong link name");
				res.success = false;
				return false;
			}
		}

		res.success = true;
		return true;

	}


	bool make_step(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		if(!req.LinkToMove.compare("l_leg")){
			std_srvs::Empty e;
			//reset_posecontroller_cli.call(e);

			tf::StampedTransform l_foot_transform;
			try {
				listener.waitForTransform("/l_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/l_foot","/pelvis",ros::Time(0),l_foot_transform);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}

			XYZRPY tranform = VectorTranformation(	req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z,
					req.AngleDestination.x, req.AngleDestination.y, req.AngleDestination.z,
					l_foot_transform.getOrigin().x(), l_foot_transform.getOrigin().y(), l_foot_transform.getOrigin().z(),
					QuatToRoll(l_foot_transform.getRotation()), QuatToPitch(l_foot_transform.getRotation()), QuatToYaw(l_foot_transform.getRotation()));

			move_pelvis::move_pelvis move;

			//Raise left leg
			move.request.PositionDestination.x = 0;
			move.request.PositionDestination.y = -0.0;
			move.request.PositionDestination.z = 0.02;
			move.request.AngleDestination.x = 0.0;
			move.request.AngleDestination.y = 0.0;
			move.request.AngleDestination.z = 0.0;
			move.request.LinkToMove = "l_leg";
			ROS_INFO("Moving left leg forward");
			if(!this->gen_traj(move.request, move.response)){
				ROS_ERROR("Could not move left leg forward");
				return false;
			}

			//ros::Duration(2.0).sleep();


			//Move left leg forward
			move.request.PositionDestination.x = tranform.x;
			move.request.PositionDestination.y = tranform.y;
			move.request.PositionDestination.z = tranform.z;
			move.request.AngleDestination.x = tranform.roll;
			move.request.AngleDestination.y = tranform.pitch;
			move.request.AngleDestination.z = tranform.yaw;
			move.request.LinkToMove = "l_leg";
			ROS_INFO("Moving left leg forward");
			if(!this->gen_traj(move.request, move.response)){
				ROS_ERROR("Could not move left leg forward");
				return false;
			}

			//Fix orientation
			/*tf::StampedTransform l_foot_to_r;
			try {
				listener.waitForTransform("/l_foot","/r_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/l_foot","/r_foot",ros::Time(0),l_foot_to_r);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}*/

			//tf::StampedTransform l_foot_transform;
			try {
				listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}

			XYZRPY tranform1 = VectorTranformation(	l_foot_transform.getOrigin().x(), l_foot_transform.getOrigin().y(), l_foot_transform.getOrigin().z(),
					QuatToRoll(l_foot_transform.getRotation()), QuatToPitch(l_foot_transform.getRotation()), QuatToYaw(l_foot_transform.getRotation()),
					0, 0, 0,
					imu.x, imu.y, imu.z);


			move.request.PositionDestination.x = 0;
			move.request.PositionDestination.y = -0.0;
			move.request.PositionDestination.z = 0;
			move.request.AngleDestination.x = -tranform1.roll;
			move.request.AngleDestination.y = -tranform1.pitch;
			move.request.AngleDestination.z = 0;//QuatToYaw(l_foot_to_r.getRotation());
			move.request.LinkToMove = "l_leg";
			ROS_INFO("Moving left leg forward");
			if(!this->gen_traj(move.request, move.response)){
				ROS_ERROR("Could not move left leg forward");
				return false;
			}

			//Lower leg
			std_srvs::Empty em;
			if(!this->step_down(em.request, em.response)){
				ROS_ERROR("Could not lower left leg");
				return false;
			}



			return true;
		}else{
			if(!req.LinkToMove.compare("r_leg")){


				std_srvs::Empty e;
				//reset_posecontroller_cli.call(e);

				tf::StampedTransform r_foot_transform;
				try {
					listener.waitForTransform("/r_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/r_foot","/pelvis",ros::Time(0),r_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				XYZRPY tranform = VectorTranformation(	req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z,
						req.AngleDestination.x, req.AngleDestination.y, req.AngleDestination.z,
						r_foot_transform.getOrigin().x(), r_foot_transform.getOrigin().y(), r_foot_transform.getOrigin().z(),
						QuatToRoll(r_foot_transform.getRotation()), QuatToPitch(r_foot_transform.getRotation()), QuatToYaw(r_foot_transform.getRotation()));

				move_pelvis::move_pelvis move;

				//Raise left leg
				move.request.PositionDestination.x = 0;
				move.request.PositionDestination.y = -0.0;
				move.request.PositionDestination.z = 0.02;
				move.request.AngleDestination.x = 0.0;
				move.request.AngleDestination.y = 0.0;
				move.request.AngleDestination.z = 0.0;
				move.request.LinkToMove = "r_leg";
				ROS_INFO("Moving left right forward");
				if(!this->gen_traj(move.request, move.response)){
					ROS_ERROR("Could not move right leg forward");
					return false;
				}

				//ros::Duration(2.0).sleep();


				//Move left leg forward
				move.request.PositionDestination.x = tranform.x;
				move.request.PositionDestination.y = tranform.y;
				move.request.PositionDestination.z = tranform.z;
				move.request.AngleDestination.x = tranform.roll;
				move.request.AngleDestination.y = tranform.pitch;
				move.request.AngleDestination.z = tranform.yaw;
				move.request.LinkToMove = "r_leg";
				ROS_INFO("Moving left right forward");
				if(!this->gen_traj(move.request, move.response)){
					ROS_ERROR("Could not move right leg forward");
					return false;
				}

				//Fix orientation
				/*tf::StampedTransform r_foot_to_l;
				try {
					listener.waitForTransform("/r_foot","/l_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/r_foot","/l_foot",ros::Time(0),r_foot_to_l);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}*/

				//tf::StampedTransform r_foot_transform;
				try {
					listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				XYZRPY tranform1 = VectorTranformation(	r_foot_transform.getOrigin().x(), r_foot_transform.getOrigin().y(), r_foot_transform.getOrigin().z(),
						QuatToRoll(r_foot_transform.getRotation()), QuatToPitch(r_foot_transform.getRotation()), QuatToYaw(r_foot_transform.getRotation()),
						0, 0, 0,
						imu.x, imu.y, imu.z);


				move.request.PositionDestination.x = 0;
				move.request.PositionDestination.y = -0.0;
				move.request.PositionDestination.z = 0;
				move.request.AngleDestination.x = -tranform1.roll;
				move.request.AngleDestination.y = -tranform1.pitch;
				move.request.AngleDestination.z = 0;//QuatToYaw(l_foot_to_r.getRotation());
				move.request.LinkToMove = "r_leg";
				ROS_INFO("Moving left right forward");
				if(!this->gen_traj(move.request, move.response)){
					ROS_ERROR("Could not move right leg forward");
					return false;
				}

				//Lower leg
				std_srvs::Empty em;
				if(!this->step_down(em.request, em.response)){
					ROS_ERROR("Could not lower right leg");
					return false;
				}


				return true;
			}else{
				ROS_ERROR("Wrong link %s", req.LinkToMove.c_str());
				return false;
			}
		}
	}


	bool walk_legs_by_pelvis(move_pelvis::move_pelvis::Request &req, move_pelvis::move_pelvis::Response &res){
		if(!req.LinkToMove.compare("l_leg")){
			std_srvs::Empty e;
			//reset_posecontroller_cli.call(e);

			tf::StampedTransform l_foot_transform;
			try {
				listener.waitForTransform("/l_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/l_foot","/pelvis",ros::Time(0),l_foot_transform);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}

			XYZRPY tranform = VectorTranformation(	req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z,
					req.AngleDestination.x, req.AngleDestination.y, req.AngleDestination.z,
					l_foot_transform.getOrigin().x(), l_foot_transform.getOrigin().y(), l_foot_transform.getOrigin().z(),
					QuatToRoll(l_foot_transform.getRotation()), QuatToPitch(l_foot_transform.getRotation()), QuatToYaw(l_foot_transform.getRotation()));

			move_pelvis::move_pelvis move;

			//ros::Duration(2.0).sleep();


			//Move left leg forward
			move.request.PositionDestination.x = tranform.x;
			move.request.PositionDestination.y = tranform.y;
			move.request.PositionDestination.z = tranform.z;
			move.request.AngleDestination.x = tranform.roll;
			move.request.AngleDestination.y = tranform.pitch;
			move.request.AngleDestination.z = tranform.yaw;
			move.request.LinkToMove = "l_leg";
			ROS_INFO("Moving left leg forward");
			if(!this->gen_traj(move.request, move.response)){
				ROS_ERROR("Could not move left leg forward");
				return false;
			}


			return true;
		}else{
			if(!req.LinkToMove.compare("r_leg")){


				std_srvs::Empty e;
				//reset_posecontroller_cli.call(e);

				tf::StampedTransform r_foot_transform;
				try {
					listener.waitForTransform("/r_foot","/pelvis",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/r_foot","/pelvis",ros::Time(0),r_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				XYZRPY tranform = VectorTranformation(	req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z,
						req.AngleDestination.x, req.AngleDestination.y, req.AngleDestination.z,
						r_foot_transform.getOrigin().x(), r_foot_transform.getOrigin().y(), r_foot_transform.getOrigin().z(),
						QuatToRoll(r_foot_transform.getRotation()), QuatToPitch(r_foot_transform.getRotation()), QuatToYaw(r_foot_transform.getRotation()));

				move_pelvis::move_pelvis move;

				//ros::Duration(2.0).sleep();


				//Move left leg forward
				move.request.PositionDestination.x = tranform.x;
				move.request.PositionDestination.y = tranform.y;
				move.request.PositionDestination.z = tranform.z;
				move.request.AngleDestination.x = tranform.roll;
				move.request.AngleDestination.y = tranform.pitch;
				move.request.AngleDestination.z = tranform.yaw;
				move.request.LinkToMove = "r_leg";
				ROS_INFO("Moving left right forward");
				if(!this->gen_traj(move.request, move.response)){
					ROS_ERROR("Could not move right leg forward");
					return false;
				}


				return true;
			}else{
				ROS_ERROR("Wrong link %s", req.LinkToMove.c_str());
				return false;
			}
		}
	}









	bool step_down(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
		double dt = 0.01;

		if(forcesensors.r_foot.force.z < 0.05 * forcesensors.l_foot.force.z){
			//We need to lower right leg
			while(forcesensors.r_foot.force.z < 0.15 * forcesensors.l_foot.force.z){
				ros::spinOnce();
				tf::StampedTransform l_foot_transform;
				try {
					listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}
				tf::StampedTransform original_l_foot_to_r;
				try {
					listener.waitForTransform("/l_foot","/r_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/l_foot","/r_foot",ros::Time(0),original_l_foot_to_r);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				XYZRPY transform = VectorTranformation(	0, 0, -0.0005,
						0, 0, 0,
						original_l_foot_to_r.getOrigin().x(), original_l_foot_to_r.getOrigin().y(), original_l_foot_to_r.getOrigin().z(),
						QuatToRoll(original_l_foot_to_r.getRotation()), QuatToPitch(original_l_foot_to_r.getRotation()), QuatToYaw(original_l_foot_to_r.getRotation()));
				XYZRPY transform2 = VectorTranformation(transform.x, transform.y, transform.z,
						transform.roll, transform.pitch, transform.yaw,
						l_foot_transform.getOrigin().x(), l_foot_transform.getOrigin().y(), l_foot_transform.getOrigin().z(),
						QuatToRoll(l_foot_transform.getRotation()), QuatToPitch(l_foot_transform.getRotation()), QuatToYaw(l_foot_transform.getRotation()));

				double newx = transform2.x;
				double newy = transform2.y;
				double newz = transform2.z;
				double newroll = transform2.roll;
				double newpitch = transform2.pitch;
				double newyaw = transform2.yaw;

				tf::StampedTransform r_foot_transform;
				try {
					listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				PoseController::foot_movement move;
				ros::spinOnce();
				legs_val_calc::legs_val_calc v;
				v.request.x_dot = (newx - r_foot_transform.getOrigin().x())/(dt);//(traj_vec_srv.response.PositionArray[ind].x + original_l_foot_to_r.getOrigin().x() - l_foot_to_r.getOrigin().x())/traj_vec_srv.response.dt[ind];
				v.request.y_dot = (newy - r_foot_transform.getOrigin().y())/(dt);//(traj_vec_srv.response.PositionArray[ind].y + original_l_foot_to_r.getOrigin().y() - l_foot_to_r.getOrigin().y())/traj_vec_srv.response.dt[ind];
				v.request.z_dot = (newz - r_foot_transform.getOrigin().z())/(dt);//(traj_vec_srv.response.PositionArray[ind].z + original_l_foot_to_r.getOrigin().z() - l_foot_to_r.getOrigin().z())/traj_vec_srv.response.dt[ind];
				v.request.roll_dot = (newroll - QuatToRoll(r_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].x + QuatToRoll(original_l_foot_to_r.getRotation()) - QuatToRoll(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.x;
				v.request.pitch_dot = (newpitch - QuatToPitch(r_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].y + QuatToPitch(original_l_foot_to_r.getRotation()) - QuatToPitch(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.y;
				v.request.yaw_dot = (newyaw - QuatToYaw(r_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].z + QuatToYaw(original_l_foot_to_r.getRotation()) - QuatToYaw(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.z;

				legs_val_calc::legs_val_calc tfv;
				tfv.request.x_dot = l_foot_transform.getOrigin().x() - l_foot_transform.getOrigin().x();
				tfv.request.y_dot = l_foot_transform.getOrigin().y() - l_foot_transform.getOrigin().y();
				tfv.request.z_dot = l_foot_transform.getOrigin().z() - l_foot_transform.getOrigin().z();
				tfv.request.roll_dot = QuatToRoll(l_foot_transform.getRotation()) - QuatToRoll(l_foot_transform.getRotation());// - imu.x;
				tfv.request.pitch_dot = QuatToPitch(l_foot_transform.getRotation()) - QuatToPitch(l_foot_transform.getRotation());// - imu.y;
				tfv.request.yaw_dot = QuatToYaw(l_foot_transform.getRotation()) - QuatToYaw(l_foot_transform.getRotation());// - imu.z;

				if(!legs_val_calc_cli_.call(tfv)){
					ROS_ERROR("Could not reach gen leg velocity vector server");
					return false;
				}

				if(legs_val_calc_cli_.call(v)){
					move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]] + v.response.q_right_dot[0]*dt;
					move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]] + v.response.q_right_dot[1]*dt;
					move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]] + v.response.q_right_dot[2]*dt;
					move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]] + v.response.q_right_dot[3]*dt;
					move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]] + v.response.q_right_dot[4]*dt;
					move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]] + v.response.q_right_dot[5]*dt;
					move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]] + tfv.response.q_right_dot[0]*dt;
					move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]] + tfv.response.q_right_dot[1]*dt;
					move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]] + tfv.response.q_right_dot[2]*dt;
					move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]] + tfv.response.q_right_dot[3]*dt;
					move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]] + tfv.response.q_right_dot[4]*dt;
					move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]] + tfv.response.q_right_dot[5]*dt;

					if(!posecontroller_cli.call(move)){
						ROS_ERROR("Could not reach PoseController server");
						return false;
					}
				}else{
					ROS_ERROR("Could not reach gen leg velocity vector server");
					return false;
				}
				ros::Duration(dt).sleep();
			}
		}else{
			if(forcesensors.l_foot.force.z < 0.05 * forcesensors.r_foot.force.z){
				//We need to lower left leg
				while(forcesensors.l_foot.force.z < 0.15 * forcesensors.r_foot.force.z){
					ros::spinOnce();
					tf::StampedTransform r_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),r_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}
					tf::StampedTransform original_r_foot_to_l;
					try {
						listener.waitForTransform("/r_foot","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/r_foot","/l_foot",ros::Time(0),original_r_foot_to_l);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					XYZRPY transform = VectorTranformation(	0, 0, -0.0005,
							0, 0, 0,
							original_r_foot_to_l.getOrigin().x(), original_r_foot_to_l.getOrigin().y(), original_r_foot_to_l.getOrigin().z(),
							QuatToRoll(original_r_foot_to_l.getRotation()), QuatToPitch(original_r_foot_to_l.getRotation()), QuatToYaw(original_r_foot_to_l.getRotation()));
					XYZRPY transform2 = VectorTranformation(transform.x, transform.y, transform.z,
							transform.roll, transform.pitch, transform.yaw,
							r_foot_transform.getOrigin().x(), r_foot_transform.getOrigin().y(), r_foot_transform.getOrigin().z(),
							QuatToRoll(r_foot_transform.getRotation()), QuatToPitch(r_foot_transform.getRotation()), QuatToYaw(r_foot_transform.getRotation()));

					double newx = transform2.x;
					double newy = transform2.y;
					double newz = transform2.z;
					double newroll = transform2.roll;
					double newpitch = transform2.pitch;
					double newyaw = transform2.yaw;

					tf::StampedTransform l_foot_transform;
					try {
						listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
						listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),l_foot_transform);
					} catch (tf::TransformException &ex) {
						ROS_ERROR("%s",ex.what());
					}

					PoseController::foot_movement move;
					ros::spinOnce();
					legs_val_calc::legs_val_calc v;
					v.request.x_dot = (newx - l_foot_transform.getOrigin().x())/(dt);//(traj_vec_srv.response.PositionArray[ind].x + original_l_foot_to_r.getOrigin().x() - l_foot_to_r.getOrigin().x())/traj_vec_srv.response.dt[ind];
					v.request.y_dot = (newy - l_foot_transform.getOrigin().y())/(dt);//(traj_vec_srv.response.PositionArray[ind].y + original_l_foot_to_r.getOrigin().y() - l_foot_to_r.getOrigin().y())/traj_vec_srv.response.dt[ind];
					v.request.z_dot = (newz - l_foot_transform.getOrigin().z())/(dt);//(traj_vec_srv.response.PositionArray[ind].z + original_l_foot_to_r.getOrigin().z() - l_foot_to_r.getOrigin().z())/traj_vec_srv.response.dt[ind];
					v.request.roll_dot = (newroll - QuatToRoll(l_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].x + QuatToRoll(original_l_foot_to_r.getRotation()) - QuatToRoll(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.x;
					v.request.pitch_dot = (newpitch - QuatToPitch(l_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].y + QuatToPitch(original_l_foot_to_r.getRotation()) - QuatToPitch(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.y;
					v.request.yaw_dot = (newyaw - QuatToYaw(l_foot_transform.getRotation()))/(dt);//(traj_vec_srv.response.AngleArray[ind].z + QuatToYaw(original_l_foot_to_r.getRotation()) - QuatToYaw(l_foot_to_r.getRotation()))/traj_vec_srv.response.dt[ind];// - imu.z;

					legs_val_calc::legs_val_calc tfv;
					tfv.request.x_dot = r_foot_transform.getOrigin().x() - r_foot_transform.getOrigin().x();
					tfv.request.y_dot = r_foot_transform.getOrigin().y() - r_foot_transform.getOrigin().y();
					tfv.request.z_dot = r_foot_transform.getOrigin().z() - r_foot_transform.getOrigin().z();
					tfv.request.roll_dot = QuatToRoll(r_foot_transform.getRotation()) - QuatToRoll(r_foot_transform.getRotation());// - imu.x;
					tfv.request.pitch_dot = QuatToPitch(r_foot_transform.getRotation()) - QuatToPitch(r_foot_transform.getRotation());// - imu.y;
					tfv.request.yaw_dot = QuatToYaw(r_foot_transform.getRotation()) - QuatToYaw(r_foot_transform.getRotation());// - imu.z;

					if(!legs_val_calc_cli_.call(tfv)){
						ROS_ERROR("Could not reach gen leg velocity vector server");
						return false;
					}

					if(legs_val_calc_cli_.call(v)){
						move.request.l_leg_uhz = posecontroller_positions[joints["l_leg_uhz"]] + v.response.q_right_dot[0]*dt;
						move.request.l_leg_mhx = posecontroller_positions[joints["l_leg_mhx"]] + v.response.q_right_dot[1]*dt;
						move.request.l_leg_lhy = posecontroller_positions[joints["l_leg_lhy"]] + v.response.q_right_dot[2]*dt;
						move.request.l_leg_kny = posecontroller_positions[joints["l_leg_kny"]] + v.response.q_right_dot[3]*dt;
						move.request.l_leg_uay = posecontroller_positions[joints["l_leg_uay"]] + v.response.q_right_dot[4]*dt;
						move.request.l_leg_lax = posecontroller_positions[joints["l_leg_lax"]] + v.response.q_right_dot[5]*dt;
						move.request.r_leg_uhz = posecontroller_positions[joints["r_leg_uhz"]] + tfv.response.q_right_dot[0]*dt;
						move.request.r_leg_mhx = posecontroller_positions[joints["r_leg_mhx"]] + tfv.response.q_right_dot[1]*dt;
						move.request.r_leg_lhy = posecontroller_positions[joints["r_leg_lhy"]] + tfv.response.q_right_dot[2]*dt;
						move.request.r_leg_kny = posecontroller_positions[joints["r_leg_kny"]] + tfv.response.q_right_dot[3]*dt;
						move.request.r_leg_uay = posecontroller_positions[joints["r_leg_uay"]] + tfv.response.q_right_dot[4]*dt;
						move.request.r_leg_lax = posecontroller_positions[joints["r_leg_lax"]] + tfv.response.q_right_dot[5]*dt;

						if(!posecontroller_cli.call(move)){
							ROS_ERROR("Could not reach PoseController server");
							return false;
						}
					}else{
						ROS_ERROR("Could not reach gen leg velocity vector server");
						return false;
					}
					ros::Duration(dt).sleep();
				}
			}else{
				//Do nothing
			}
		}
		return true;
	}



	int index_with_lookahead(int ind, int total, int lookahead){
		return std::min(total, ind+lookahead);
	}


};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "walk_legs_service");
	ROS_INFO("starting walk_legs service");
	walk_legs_service* c = new walk_legs_service();
	ROS_INFO("started walk_legs service");

	ros::spin();
	return 0;
}



