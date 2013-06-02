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
#include "move_hand/pelvis_move_hand.h"
#include "move_hand/wheel_move_hand.h"
#include "move_hand/matrix.h"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <atlas_msgs/AtlasCommand.h>
#include <math.h>

class move_hand_service{

protected:
	ros::NodeHandle nh_, nh2_,nh3_,nh4_,nh5_;
	ros::NodeHandle* rosnode;
	ros::ServiceServer move_hand_srv_,pelvis_move_hand_srv_,C66_matrix_srv_,wheel_move_hand_srv_;
	ros::ServiceClient traj_vector_cli_,wheel_traj_vector_cli_;
	ros::ServiceClient arms_val_calc_cli_;
	ros::Subscriber joint_states_sub_;
	double q0_l,q1_l,q2_l,q3_l,q4_l,q5_l;
	double q0_r,q1_r,q2_r,q3_r,q4_r,q5_r;
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> traj_client_;
	tf::TransformListener listener;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;

private:
	int _n;
	 double QuatToRoll(double x, double y, double z, double w){
	        return atan2(2*(w*x + y*z), 1 - 2*(pow(x,2) + pow(y,2)));
	    }
	    double QuatToRoll(const tf::Quaternion &quat){
	        return atan2(2*(quat.w()*quat.x() + quat.y()*quat.z()), 1 - 2*(pow(quat.x(),2) + pow(quat.y(),2)));
	    }
	    double QuatToRoll(const geometry_msgs::Quaternion &quat){
	        return atan2(2*(quat.w*quat.x + quat.y*quat.z), 1 - 2*(pow(quat.x,2) + pow(quat.y,2)));
	    }


	    double QuatToPitch(double x, double y, double z, double w){
	        return asin(2*(w*y - z*x));
	    }
	    double QuatToPitch(const tf::Quaternion &quat){
	        return asin(2*(quat.w()*quat.y() - quat.z()*quat.x()));
	    }
	    double QuatToPitch(const geometry_msgs::Quaternion &quat){
	        return asin(2*(quat.w*quat.y - quat.z*quat.x));
	    }


	    double QuatToYaw(double x, double y, double z, double w){
	        return atan2(2*(w*z + x*y), 1 - 2*(pow(y,2) + pow(z,2)));
	    }
	    double QuatToYaw(const tf::Quaternion &quat){
	        return atan2(2*(quat.w()*quat.z() + quat.x()*quat.y()), 1 - 2*(pow(quat.y(),2) + pow(quat.z(),2)));
	    }
	    double QuatToYaw(const geometry_msgs::Quaternion &quat){
	        return atan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(pow(quat.y,2) + pow(quat.z,2)));
	    }
public:
	move_hand_service():
		traj_client_(nh2_,"atlas_controller/follow_joint_trajectory", true)
	{
		rosnode = new ros::NodeHandle();
		ros::NodeHandle nh_private("~");
		traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("traj_vector_server");
		wheel_traj_vector_cli_ = nh_.serviceClient<traj_splitter_to_vector::trajectory_vector>("wheel_traj_vector_server");
		while((!traj_vector_cli_.waitForExistence(ros::Duration(1.0)))&&(!wheel_traj_vector_cli_.waitForExistence(ros::Duration(1.0)))){
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
		pub_joint_commands_ = rosnode->advertise<atlas_msgs::AtlasCommand>("/atlas/atlas_command", 1, true);

		move_hand_srv_ = nh_.advertiseService("move_hand", &move_hand_service::gen_traj, this);
		ROS_INFO("running move hand service");

		pelvis_move_hand_srv_ = nh3_.advertiseService("pelvis_move_hand",&move_hand_service::pelvis_move_hand_CB,this);
		ROS_INFO("running pelvis move hand service");

		C66_matrix_srv_ = nh4_.advertiseService("C66_matrix",&move_hand_service::matrix_in,this);

		wheel_move_hand_srv_ = nh5_.advertiseService("wheel_move_hand",&move_hand_service::wheel_move_hand_CB,this);
		ROS_INFO("running wheel move hand service");

	}
	~move_hand_service(){
		delete rosnode;
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
		atlas_msgs::AtlasCommand atlas_command;
		//ROS_INFO("Received request to move %s to (%f,%f,%f)", req.LinkToMove.c_str(), req.PositionDestination.x, req.PositionDestination.y, req.PositionDestination.z);
		traj_vec_left_srv.request.Position = req.PositionDestination_left;
		traj_vec_left_srv.request.Angle = req.AngleDestination_left;
		traj_vec_left_srv.request.segments_number = 1000 ;
		traj_vec_left_srv.request.total_time = 5 ;

                traj_vec_right_srv.request.Position = req.PositionDestination_right;
                traj_vec_right_srv.request.Angle = req.AngleDestination_right;
                traj_vec_right_srv.request.segments_number = 1000 ;
                traj_vec_right_srv.request.total_time = 5 ;
		if(req.quick){
			traj_vec_left_srv.request.segments_number = 100 ;
			traj_vec_left_srv.request.total_time = 2;
			traj_vec_right_srv.request.segments_number = 100 ;
			traj_vec_right_srv.request.total_time = 2;
		}

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
		for (unsigned int i = 0; i < 16; i++)
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
			atlas_command.k_effort[i] = 0;
		}

		for (unsigned int i = 16; i < n; i++)
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
			//double current_dt = 0;

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

							atlas_command.position[joints["l_arm_usy"]] = p0_l + v_left.response.q_left_dot[0]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_usy"]] = v_left.response.q_left_dot[0];
							p0_l = atlas_command.position[joints["l_arm_usy"]];
							atlas_command.position[joints["l_arm_shx"]] = p1_l + v_left.response.q_left_dot[1]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_shx"]] = v_left.response.q_left_dot[1];
							p1_l = atlas_command.position[joints["l_arm_shx"]];
							atlas_command.position[joints["l_arm_ely"]] = p2_l + v_left.response.q_left_dot[2]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_ely"]] = v_left.response.q_left_dot[2];
							p2_l = atlas_command.position[joints["l_arm_ely"]];
							atlas_command.position[joints["l_arm_elx"]] = p3_l + v_left.response.q_left_dot[3]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_elx"]] = v_left.response.q_left_dot[3];
							p3_l = atlas_command.position[joints["l_arm_elx"]];
							atlas_command.position[joints["l_arm_uwy"]] = p4_l + v_left.response.q_left_dot[4]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_uwy"]] = v_left.response.q_left_dot[4];
							p4_l = atlas_command.position[joints["l_arm_uwy"]];
							atlas_command.position[joints["l_arm_mwx"]] = p5_l + v_left.response.q_left_dot[5]*traj_vec_left_srv.response.dt[ind];
							//atlas_command.velocity[joints["l_arm_mwx"]] = v_left.response.q_left_dot[5];
							p5_l = atlas_command.position[joints["l_arm_mwx"]];


								atlas_command.position[joints["r_arm_usy"]] = p0_r + v_right.response.q_right_dot[0]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_usy"]] = v_right.response.q_right_dot[0];
								p0_r = atlas_command.position[joints["r_arm_usy"]];
								atlas_command.position[joints["r_arm_shx"]] = p1_r + v_right.response.q_right_dot[1]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_shx"]] = v_right.response.q_right_dot[1];
								p1_r = atlas_command.position[joints["r_arm_shx"]];
								atlas_command.position[joints["r_arm_ely"]] = p2_r + v_right.response.q_right_dot[2]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_ely"]] = v_right.response.q_right_dot[2];
								p2_r = atlas_command.position[joints["r_arm_ely"]];
								atlas_command.position[joints["r_arm_elx"]] = p3_r + v_right.response.q_right_dot[3]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_elx"]] = v_right.response.q_right_dot[3];
								p3_r = atlas_command.position[joints["r_arm_elx"]];
								atlas_command.position[joints["r_arm_uwy"]] = p4_r + v_right.response.q_right_dot[4]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_uwy"]] = v_right.response.q_right_dot[4];
								p4_r = atlas_command.position[joints["r_arm_uwy"]];
								atlas_command.position[joints["r_arm_mwx"]] = p5_r + v_right.response.q_right_dot[5]*traj_vec_right_srv.response.dt[ind];
								//atlas_command.velocity[joints["r_arm_mwx"]] = v_right.response.q_right_dot[5];
								p5_r = atlas_command.position[joints["r_arm_mwx"]];


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

					atlas_command.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
					atlas_command.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
					atlas_command.position[joints["back_mby"]] = positions[joints["back_mby"]];
					atlas_command.position[joints["back_ubx"]] = positions[joints["back_ubx"]];

					double positions_check[12];
					positions_check[0] = p0_l;positions_check[1] = p1_l;positions_check[2] = p2_l;positions_check[3] = p3_l;positions_check[4] = p4_l;
					positions_check[5] =p5_l;positions_check[6] = p0_r;positions_check[7] = p1_r;positions_check[8] = p2_r;positions_check[9] = p3_r;
					positions_check[10] =p4_r;positions_check[11] =p5_r;
					if (!((positions_check[0]>=-1.9635)&&(positions_check[0]<=1.9635))){
						ROS_INFO("cannot reach position because l_arm_usy is %f",positions_check[0]);
						return false;
					}else if(!((positions_check[1]>=-1.39626)&&(positions_check[1]<=1.74533))){
						ROS_INFO("cannot reach position because l_arm_shx is %f",positions_check[1]);
						return false;
					}else if(!((positions_check[2]>=0)&&(positions_check[2]<=3.14159))){
						ROS_INFO("cannot reach position because l_arm_ely is %f",positions_check[2]);
						return false;
					}else if(!((positions_check[3]>=0)&&(positions_check[3]<=2.35619))){
						ROS_INFO("cannot reach position because l_arm_elx is %f",positions_check[3]);
						return false;
					}else if(!((positions_check[4]>=-1.571)&&(positions_check[4]<=1.571))){
						ROS_INFO("cannot reach position because l_arm_uwy is %f",positions_check[4]);
						return false;
					}else if(!((positions_check[5]>=-0.436)&&(positions_check[5]<=1.571))){
						ROS_INFO("cannot reach position because l_arm_mwx is %f",positions_check[5]);
						return false;
					}else if(!((positions_check[6]>=-1.9635)&&(positions_check[6]<=1.9635))){
						ROS_INFO("cannot reach position because r_arm_usy is %f",positions_check[6]);
						return false;
					}else if(!((positions_check[7]>=-1.74533)&&(positions_check[7]<=1.39626))){
						ROS_INFO("cannot reach position because r_arm_shx is %f",positions_check[7]);
						return false;
					}else if(!((positions_check[8]>=0)&&(positions_check[8]<=3.14159))){
						ROS_INFO("cannot reach position because r_arm_ely is %f",positions_check[8]);
						return false;
					}else if(!((positions_check[9]>=-2.35619)&&(positions_check[9]<=0))){
						ROS_INFO("cannot reach position because r_arm_elx is %f",positions_check[9]);
						return false;
					}else if(!((positions_check[10]>=-1.571)&&(positions_check[10]<=1.571))){
						ROS_INFO("cannot reach position because r_arm_uwy is %f",positions_check[10]);
						return false;
					}else if(!((positions_check[11]>=-1.571)&&(positions_check[11]<=0.436))){
						ROS_INFO("cannot reach position because r_arm_mwx is %f",positions_check[11]);
						return false;
					}
					// To be reached 1+ind second after the start of the trajectory
					pub_joint_commands_.publish(atlas_command);
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

	bool pelvis_move_hand_CB(move_hand::pelvis_move_handRequest &req,move_hand::pelvis_move_handResponse &res){
	  double cosr_left,sinr_left,cosp_left,sinp_left,cosy_left,siny_left,cosr_right,sinr_right,cosp_right,sinp_right,cosy_right,siny_right;
	  double cosr_tf_left,sinr_tf_left,cosp_tf_left,sinp_tf_left,cosy_tf_left,siny_tf_left,cosr_tf_right,sinr_tf_right,cosp_tf_right,sinp_tf_right,cosy_tf_right,siny_tf_right;
	  cosr_left = cos(req.AngleDestination_left.x);
	  cosp_left = cos(req.AngleDestination_left.y);
	  cosy_left = cos(req.AngleDestination_left.z);
          sinr_left = sin(req.AngleDestination_left.x);
          sinp_left = sin(req.AngleDestination_left.y);
          siny_left = sin(req.AngleDestination_left.z);

          cosr_right = cos(req.AngleDestination_right.x);
          cosp_right = cos(req.AngleDestination_right.y);
          cosy_right = cos(req.AngleDestination_right.z);
          sinr_right = sin(req.AngleDestination_right.x);
          sinp_right = sin(req.AngleDestination_right.y);
          siny_right = sin(req.AngleDestination_right.z);

          Eigen::Matrix4f left_in,right_in,tf_left,tf_right;

          left_in <<     cosp_left*cosy_left,     cosy_left*sinp_left*sinr_left-cosr_left*siny_left,     sinr_left*siny_left+cosr_left*cosy_left*sinp_left,     req.PositionDestination_left.x,
                         cosp_left*siny_left,     cosr_left*cosy_left+cosy_left*sinp_left*sinr_left,     cosr_left*sinp_left*siny_left-cosy_left*sinr_left,     req.PositionDestination_left.y,
                         -sinp_left,                    cosp_left*sinr_left,                                      cosp_left*cosr_left,                          req.PositionDestination_left.z,
                           0,                                       0,                                                   0,                                                  1;

          right_in <<     cosp_right*cosy_right,     cosy_right*sinp_right*sinr_right-cosr_right*siny_right,     sinr_right*siny_right+cosr_right*cosy_right*sinp_right,     req.PositionDestination_right.x,
                         cosp_right*siny_right,     cosr_right*cosy_right+cosy_right*sinp_right*sinr_right,     cosr_right*sinp_right*siny_right-cosy_right*sinr_right,     req.PositionDestination_right.y,
                         -sinp_right,                    cosp_right*sinr_right,                                      cosp_right*cosr_right,                                 req.PositionDestination_right.z,
                           0,                                    0,                                                              0,                                                  1;
          tf::StampedTransform transform_left,transform_right,transform_left_finger,transform_right_finger;
          ROS_INFO("taking tf info");
          try {
              listener.waitForTransform("/l_clav","/pelvis",ros::Time(0),ros::Duration(0.2));
              listener.lookupTransform("/l_clav","/pelvis",ros::Time(0),transform_left);
            } catch (tf::TransformException &ex) {
              ROS_ERROR("%s",ex.what());
            }

            try {
                listener.waitForTransform("/r_clav","/pelvis",ros::Time(0),ros::Duration(0.2));
                listener.lookupTransform("/r_clav","/pelvis",ros::Time(0),transform_right);
              } catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
              }

              try {
                  listener.waitForTransform("/l_clav","/left_f1_0",ros::Time(0),ros::Duration(0.2));
                  listener.lookupTransform("/l_clav","/left_f1_0",ros::Time(0),transform_left_finger);
                } catch (tf::TransformException &ex) {
                  ROS_ERROR("%s",ex.what());
                }

                try {
                    listener.waitForTransform("/r_clav","/right_f1_0",ros::Time(0),ros::Duration(0.2));
                    listener.lookupTransform("/r_clav","/right_f1_0",ros::Time(0),transform_right_finger);
                  } catch (tf::TransformException &ex) {
                    ROS_ERROR("%s",ex.what());
                  }

              cosr_tf_left = cos(QuatToRoll(transform_left.getRotation()));
                cosp_tf_left = cos(QuatToPitch(transform_left.getRotation()));
                cosy_tf_left = cos(QuatToYaw(transform_left.getRotation()));
                sinr_tf_left = sin(QuatToRoll(transform_left.getRotation()));
                sinp_tf_left = sin(QuatToPitch(transform_left.getRotation()));
                siny_tf_left = sin(QuatToYaw(transform_left.getRotation()));

                cosr_tf_right = cos(QuatToRoll(transform_right.getRotation()));
                  cosp_tf_right = cos(QuatToPitch(transform_right.getRotation()));
                  cosy_tf_right = cos(QuatToYaw(transform_right.getRotation()));
                  sinr_tf_right = sin(QuatToRoll(transform_right.getRotation()));
                  sinp_tf_right = sin(QuatToPitch(transform_right.getRotation()));
                  siny_tf_right = sin(QuatToYaw(transform_right.getRotation()));

                  tf_left <<     cosp_tf_left*cosy_tf_left,     cosy_tf_left*sinp_tf_left*sinr_tf_left-cosr_tf_left*siny_tf_left,     sinr_tf_left*siny_tf_left+cosr_tf_left*cosy_tf_left*sinp_tf_left,     transform_left.getOrigin().getX(),
                                 cosp_tf_left*siny_tf_left,     cosr_tf_left*cosy_tf_left+cosy_tf_left*sinp_tf_left*sinr_tf_left,     cosr_tf_left*sinp_tf_left*siny_tf_left-cosy_tf_left*sinr_tf_left,     transform_left.getOrigin().getY(),
                                 -sinp_tf_left,                    cosp_tf_left*sinr_tf_left,                                              cosp_tf_left*cosr_tf_left,                                       transform_left.getOrigin().getZ(),
                                   0,                                       0,                                                                               0,                                                            1;

                  tf_right <<     cosp_tf_right*cosy_tf_right,     cosy_tf_right*sinp_tf_right*sinr_tf_right-cosr_tf_right*siny_tf_right,     sinr_tf_right*siny_tf_right+cosr_tf_right*cosy_tf_right*sinp_tf_right,     transform_right.getOrigin().getX(),
                                 cosp_tf_right*siny_tf_right,     cosr_tf_right*cosy_tf_right+cosy_tf_right*sinp_tf_right*sinr_tf_right,     cosr_tf_right*sinp_tf_right*siny_tf_right-cosy_tf_right*sinr_tf_right,     transform_right.getOrigin().getY(),
                                 -sinp_tf_right,                    cosp_tf_right*sinr_tf_right,                                              cosp_tf_right*cosr_tf_right,                                              transform_right.getOrigin().getZ(),
                                   0,                                       0,                                                                               0,                                                            1;

                  Eigen::Matrix4f right_mat,left_mat;
                  std::cout << "tf_right: " <<tf_right<< std::endl;
                  std::cout << "right_in: " <<tf_right<< std::endl;

                  right_mat = tf_right * right_in;
                  left_mat = tf_left * left_in;
                  move_hand::move_hand move_hand_msg;

                  if(!(req.PositionDestination_left.x==0 && req.PositionDestination_left.y==0 && req.PositionDestination_left.z==0))
                  {
                  move_hand_msg.request.PositionDestination_left.x = left_mat(0,3) - transform_left_finger.getOrigin().getX();
                  move_hand_msg.request.PositionDestination_left.y = left_mat(1,3) - transform_left_finger.getOrigin().getY();
                  move_hand_msg.request.PositionDestination_left.z = left_mat(2,3) - transform_left_finger.getOrigin().getZ();

                  move_hand_msg.request.AngleDestination_left.x = (atan2((double)left_mat(2,1),(double)left_mat(2,2))) - QuatToRoll(transform_left_finger.getRotation());
                  move_hand_msg.request.AngleDestination_left.y = (atan2((double)left_mat(2,0)*-1,sqrt(pow((double)left_mat(2,1),2)+pow((double)left_mat(2,2),2))))- QuatToPitch(transform_left_finger.getRotation());
                  move_hand_msg.request.AngleDestination_left.z = (atan2((double)left_mat(1,0),(double)left_mat(0,0)))- QuatToYaw(transform_left_finger.getRotation());
                  ROS_INFO("left x y z roll pitch yaw: %f %f %f %f %f %f",move_hand_msg.request.PositionDestination_left.x,move_hand_msg.request.PositionDestination_left.y,move_hand_msg.request.PositionDestination_left.z,
                                                                           move_hand_msg.request.AngleDestination_left.x,move_hand_msg.request.AngleDestination_left.y,move_hand_msg.request.AngleDestination_left.z);
                  }
                  if(!(req.PositionDestination_right.x==0 && req.PositionDestination_right.y==0 && req.PositionDestination_right.z==0))
                  {
                  move_hand_msg.request.PositionDestination_right.x = right_mat(0,3) - transform_right_finger.getOrigin().getX();
                  move_hand_msg.request.PositionDestination_right.y = right_mat(1,3) - transform_right_finger.getOrigin().getY();
                  move_hand_msg.request.PositionDestination_right.z = right_mat(2,3) - transform_right_finger.getOrigin().getZ();

                  move_hand_msg.request.AngleDestination_right.x = (atan2((double)right_mat(2,1),(double)right_mat(2,2))) - QuatToRoll(transform_right_finger.getRotation());
                  move_hand_msg.request.AngleDestination_right.y = (atan2((double)right_mat(2,0)*-1,sqrt(pow((double)right_mat(2,1),2)+pow((double)right_mat(2,2),2)))) - QuatToPitch(transform_right_finger.getRotation());
                  move_hand_msg.request.AngleDestination_right.z = (atan2((double)right_mat(1,0),(double)right_mat(0,0)))- QuatToYaw(transform_right_finger.getRotation());
                  ROS_INFO("right x y z roll pitch yaw: %f %f %f %f %f %f",move_hand_msg.request.PositionDestination_right.x,move_hand_msg.request.PositionDestination_right.y,move_hand_msg.request.PositionDestination_right.z,
                                                                          move_hand_msg.request.AngleDestination_right.x,move_hand_msg.request.AngleDestination_right.y,move_hand_msg.request.AngleDestination_right.z);
                  move_hand_msg.request.quick = req.quick;
                  }
                  ROS_INFO("moving hand");

                  if (gen_traj(move_hand_msg.request,move_hand_msg.response))
                  {
                    res.success = move_hand_msg.response.success;
                    return true;
                  }
                  ROS_INFO("move_hand service Error");
                  ROS_INFO("restarting hands");
                  position1();
                  ros::Duration(0.5).sleep();

                  ROS_INFO("moving hand");
                  ros::spinOnce();

                           ROS_INFO("taking tf info");
                           try {
                               listener.waitForTransform("/l_clav","/pelvis",ros::Time(0),ros::Duration(0.2));
                               listener.lookupTransform("/l_clav","/pelvis",ros::Time(0),transform_left);
                             } catch (tf::TransformException &ex) {
                               ROS_ERROR("%s",ex.what());
                             }

                             try {
                                 listener.waitForTransform("/r_clav","/pelvis",ros::Time(0),ros::Duration(0.2));
                                 listener.lookupTransform("/r_clav","/pelvis",ros::Time(0),transform_right);
                               } catch (tf::TransformException &ex) {
                                 ROS_ERROR("%s",ex.what());
                               }

                               try {
                                   listener.waitForTransform("/l_clav","/left_f1_0",ros::Time(0),ros::Duration(0.2));
                                   listener.lookupTransform("/l_clav","/left_f1_0",ros::Time(0),transform_left_finger);
                                 } catch (tf::TransformException &ex) {
                                   ROS_ERROR("%s",ex.what());
                                 }

                                 try {
                                     listener.waitForTransform("/r_clav","/right_f1_0",ros::Time(0),ros::Duration(0.2));
                                     listener.lookupTransform("/r_clav","/right_f1_0",ros::Time(0),transform_right_finger);
                                   } catch (tf::TransformException &ex) {
                                     ROS_ERROR("%s",ex.what());
                                   }

                               cosr_tf_left = cos(QuatToRoll(transform_left.getRotation()));
                                 cosp_tf_left = cos(QuatToPitch(transform_left.getRotation()));
                                 cosy_tf_left = cos(QuatToYaw(transform_left.getRotation()));
                                 sinr_tf_left = sin(QuatToRoll(transform_left.getRotation()));
                                 sinp_tf_left = sin(QuatToPitch(transform_left.getRotation()));
                                 siny_tf_left = sin(QuatToYaw(transform_left.getRotation()));

                                 cosr_tf_right = cos(QuatToRoll(transform_right.getRotation()));
                                   cosp_tf_right = cos(QuatToPitch(transform_right.getRotation()));
                                   cosy_tf_right = cos(QuatToYaw(transform_right.getRotation()));
                                   sinr_tf_right = sin(QuatToRoll(transform_right.getRotation()));
                                   sinp_tf_right = sin(QuatToPitch(transform_right.getRotation()));
                                   siny_tf_right = sin(QuatToYaw(transform_right.getRotation()));

                                   tf_left <<     cosp_tf_left*cosy_tf_left,     cosy_tf_left*sinp_tf_left*sinr_tf_left-cosr_tf_left*siny_tf_left,     sinr_tf_left*siny_tf_left+cosr_tf_left*cosy_tf_left*sinp_tf_left,     transform_left.getOrigin().getX(),
                                                  cosp_tf_left*siny_tf_left,     cosr_tf_left*cosy_tf_left+cosy_tf_left*sinp_tf_left*sinr_tf_left,     cosr_tf_left*sinp_tf_left*siny_tf_left-cosy_tf_left*sinr_tf_left,     transform_left.getOrigin().getY(),
                                                  -sinp_tf_left,                    cosp_tf_left*sinr_tf_left,                                              cosp_tf_left*cosr_tf_left,                                       transform_left.getOrigin().getZ(),
                                                    0,                                       0,                                                                               0,                                                            1;

                                   tf_right <<     cosp_tf_right*cosy_tf_right,     cosy_tf_right*sinp_tf_right*sinr_tf_right-cosr_tf_right*siny_tf_right,     sinr_tf_right*siny_tf_right+cosr_tf_right*cosy_tf_right*sinp_tf_right,     transform_right.getOrigin().getX(),
                                                  cosp_tf_right*siny_tf_right,     cosr_tf_right*cosy_tf_right+cosy_tf_right*sinp_tf_right*sinr_tf_right,     cosr_tf_right*sinp_tf_right*siny_tf_right-cosy_tf_right*sinr_tf_right,     transform_right.getOrigin().getY(),
                                                  -sinp_tf_right,                    cosp_tf_right*sinr_tf_right,                                              cosp_tf_right*cosr_tf_right,                                              transform_right.getOrigin().getZ(),
                                                    0,                                       0,                                                                               0,                                                            1;


                                   std::cout << "tf_right: " <<tf_right<< std::endl;
                                   std::cout << "right_in: " <<tf_right<< std::endl;

                                   right_mat = tf_right * right_in;
                                   left_mat = tf_left * left_in;


                                   if(!(req.PositionDestination_left.x==0 && req.PositionDestination_left.y==0 && req.PositionDestination_left.z==0))
                                   {
                                   move_hand_msg.request.PositionDestination_left.x = left_mat(0,3) - transform_left_finger.getOrigin().getX();
                                   move_hand_msg.request.PositionDestination_left.y = left_mat(1,3) - transform_left_finger.getOrigin().getY();
                                   move_hand_msg.request.PositionDestination_left.z = left_mat(2,3) - transform_left_finger.getOrigin().getZ();

                                   move_hand_msg.request.AngleDestination_left.x = (atan2((double)left_mat(2,1),(double)left_mat(2,2))) - QuatToRoll(transform_left_finger.getRotation());
                                   move_hand_msg.request.AngleDestination_left.y = (atan2((double)left_mat(2,0)*-1,sqrt(pow((double)left_mat(2,1),2)+pow((double)left_mat(2,2),2))))- QuatToPitch(transform_left_finger.getRotation());
                                   move_hand_msg.request.AngleDestination_left.z = (atan2((double)left_mat(1,0),(double)left_mat(0,0)))- QuatToYaw(transform_left_finger.getRotation());
                                   ROS_INFO("left x y z roll pitch yaw: %f %f %f %f %f %f",move_hand_msg.request.PositionDestination_left.x,move_hand_msg.request.PositionDestination_left.y,move_hand_msg.request.PositionDestination_left.z,
                                                                                            move_hand_msg.request.AngleDestination_left.x,move_hand_msg.request.AngleDestination_left.y,move_hand_msg.request.AngleDestination_left.z);
                                   }
                                   if(!(req.PositionDestination_right.x==0 && req.PositionDestination_right.y==0 && req.PositionDestination_right.z==0))
                                   {
                                   move_hand_msg.request.PositionDestination_right.x = right_mat(0,3) - transform_right_finger.getOrigin().getX();
                                   move_hand_msg.request.PositionDestination_right.y = right_mat(1,3) - transform_right_finger.getOrigin().getY();
                                   move_hand_msg.request.PositionDestination_right.z = right_mat(2,3) - transform_right_finger.getOrigin().getZ();

                                   move_hand_msg.request.AngleDestination_right.x = (atan2((double)right_mat(2,1),(double)right_mat(2,2))) - QuatToRoll(transform_right_finger.getRotation());
                                   move_hand_msg.request.AngleDestination_right.y = (atan2((double)right_mat(2,0)*-1,sqrt(pow((double)right_mat(2,1),2)+pow((double)right_mat(2,2),2)))) - QuatToPitch(transform_right_finger.getRotation());
                                   move_hand_msg.request.AngleDestination_right.z = (atan2((double)right_mat(1,0),(double)right_mat(0,0)))- QuatToYaw(transform_right_finger.getRotation());
                                   ROS_INFO("right x y z roll pitch yaw: %f %f %f %f %f %f",move_hand_msg.request.PositionDestination_right.x,move_hand_msg.request.PositionDestination_right.y,move_hand_msg.request.PositionDestination_right.z,
                                                                                           move_hand_msg.request.AngleDestination_right.x,move_hand_msg.request.AngleDestination_right.y,move_hand_msg.request.AngleDestination_right.z);
                                   move_hand_msg.request.quick = req.quick;
                                   }
                                   ROS_INFO("moving hand");

                                   if (gen_traj(move_hand_msg.request,move_hand_msg.response))
                                   {
                                     res.success = move_hand_msg.response.success;
                                     return true;
                                   }
                                    ROS_INFO("move_hand service Error");
                  return false;
	}

	bool matrix_in(move_hand::matrixRequest &req,move_hand::matrixResponse &res){
	  Eigen::Matrix4f in_mat;
	  in_mat << req.matrix[0], req.matrix[1], req.matrix[2], req.matrix[3],
	            req.matrix[4], req.matrix[5], req.matrix[6], req.matrix[7],
	            req.matrix[8], req.matrix[9], req.matrix[10],req.matrix[11],
	            req.matrix[12],req.matrix[13],req.matrix[14],req.matrix[15];


	  Eigen::Matrix4f TF1,TF2;
	            TF1(0,0)=0.956958030337725;
	            TF1(0,1)=0.187313741592912;
	            TF1(0,2)=-0.221686468650221;
	            TF1(0,3)=0.555000000000000;
	            TF1(1,0)=0.226085162286393;
	            TF1(1,1)=0.132946235442734;
	            TF1(1,2)=0.974105103609595;
	            TF1(1,3)=-0.289000000000000;
	            TF1(2,0)= 0.181980294444418;
	            TF1(2,1)=-0.982297722533646;
	            TF1(2,2)=-0.044433734247019;
	            TF1(2,3)=0.136000000000000;
	            TF1(3,0)=0;
	            TF1(3,1)=0;
	            TF1(3,2)=0;
	            TF1(3,3)=1;

                    TF2(0,0)=0.956958030337725;
                    TF2(0,1)=0.187313741592912;
                    TF2(0,2)=-0.221686468650221;
                    TF2(0,3)=0.505000000000000;
                    TF2(1,0)=0.226085162286393;
                    TF2(1,1)=0.132946235442734;
                    TF2(1,2)=0.974105103609595;
                    TF2(1,3)=-0.339000000000000;
                    TF2(2,0)= 0.181980294444418;
                    TF2(2,1)=-0.982297722533646;
                    TF2(2,2)=-0.044433734247019;
                    TF2(2,3)=0.136000000000000;
                    TF2(3,0)=0;
                    TF2(3,1)=0;
                    TF2(3,2)=0;
                    TF2(3,3)=1;

	            Eigen::Matrix4f mat,mat2;
	            mat=in_mat*TF1;
	            double x= mat(0,3);
	            double y= mat(1,3);
	            double z= mat(2,3);
	            double roll    =(std::atan2((double)mat(2,1),(double)mat(2,2)));
	            double pitch = (std::atan2((double)-mat(2,0),std::sqrt(std::pow((double)mat(2,1),2) +std::pow((double)mat(2,2),2) )));
	            double yaw   = (std::atan2((double)mat(1,0),(double)mat(0,0)));

	            mat2=in_mat*TF2;
                    double x2= mat2(0,3);
                    double y2= mat2(1,3);
                    double z2= mat2(2,3);
                    double roll2    =(std::atan2((double)mat2(2,1),(double)mat2(2,2)));
                    double pitch2 = (std::atan2((double)-mat2(2,0),std::sqrt(std::pow((double)mat2(2,1),2) +std::pow((double)mat2(2,2),2) )));
                    double yaw2   = (std::atan2((double)mat2(1,0),(double)mat2(0,0)));

	            std::cout<<"x:"<<x<<"\n";
	            std::cout<<"y:"<<y<<"\n";
	            std::cout<<"z:"<<z<<"\n";
	            std::cout<<"roll:"<<roll<<"\n";
	            std::cout<<"pitch:"<<pitch<<"\n";
	            std::cout<<"yaw:"<<yaw<<"\n";

	            move_hand::pelvis_move_hand move_msg,move_msg2;

	                move_msg.request.PositionDestination_right.x =  x;
	                move_msg.request.PositionDestination_right.y =  y;
	                move_msg.request.PositionDestination_right.z =  z;
	                move_msg.request.AngleDestination_right.x = roll;
	                move_msg.request.AngleDestination_right.y = pitch;
	                move_msg.request.AngleDestination_right.z = yaw;

                        move_msg2.request.PositionDestination_right.x =  x2;
                        move_msg2.request.PositionDestination_right.y =  y2;
                        move_msg2.request.PositionDestination_right.z =  z2;
                        move_msg2.request.AngleDestination_right.x = roll2;
                        move_msg2.request.AngleDestination_right.y = pitch2;
                        move_msg2.request.AngleDestination_right.z = yaw2;

                        if (!pelvis_move_hand_CB(move_msg2.request,move_msg2.response)){
                        ROS_INFO("error in pelvis_move_hand service");
                        return false;
                        }

                        ros::spinOnce();


          if (pelvis_move_hand_CB(move_msg.request,move_msg.response))
          {
            res.success = move_msg.response.success;
            return true;
          }
          ROS_INFO("error in pelvis_move_hand service");
          return false;
	}
	bool wheel_move_hand_CB(move_hand::wheel_move_handRequest &req,move_hand::wheel_move_handResponse &res){
		double cone = 0.5;
		Eigen::Matrix4f Rr,Rp,Ry,Rd,Rcone,Rl;
		Rd << 1,0,0,req.wheel_center.x,
			  0,1,0,req.wheel_center.y,
			  0,0,1,req.wheel_center.z,
			  0,0,0,			1;

		Rcone << cos(cone), 0, sin(cone), 0,
					0, 		1, 	 0, 	 0,
				-sin(cone), 0, cos(cone),0,
					0, 		0, 	 0, 	 1;

		Rp << cos(req.pitch), 0, sin(req.pitch), 0,
				0, 		      1, 		0, 		 0,
			-sin(req.pitch), 0 ,cos(req.pitch),  0,
					0,		 0, 	0,			 1;

		Ry << cos(req.yaw), -sin(req.yaw), 0, 0,
			  sin(req.yaw), cos(req.yaw),  0, 0,
			       0,			 0, 	   1, 0,
			       0, 			 0,        0, 1;
		ROS_INFO("start steering");
		for(double i=req.fangle;fabs(i-(req.phi+req.fangle))>=req.dangle;i=i+copysign(req.dangle,req.phi)){
			if(i==req.fangle) continue;
			ROS_INFO("angel is %f",i);
			Rr << 1, 	0,				 0,					 0,
				  0, cos(i-M_PI/2), -sin(i-M_PI/2), 	  	 0,
				  0, sin(i-M_PI/2), cos(i-M_PI/2), 			 0,
				  0, 	 0, 			 0, 				 1;
			Eigen::Vector4f Rvec,XYZ;
			Rvec << 0 , req.radius*cos(i) , req.radius*sin(i) ,1;
			XYZ = Rd*Ry*Rp*Rvec;
			Rl = Ry*Rp*Rr*Rcone;
			double roll = atan2((double)Rl(2,1),(double)Rl(2,2));
			roll-=M_PI;
			double pitch = atan2((double)Rl(2,0)*-1,sqrt(pow((double)Rl(2,1),2)+pow((double)Rl(2,2),2)));
			double yaw = atan2((double)Rl(1,0),(double)Rl(0,0));
			move_hand::pelvis_move_hand msg;
			msg.request.PositionDestination_right.x = XYZ(0); msg.request.PositionDestination_right.y = XYZ(1); msg.request.PositionDestination_right.z = XYZ(2);
			msg.request.AngleDestination_right.x = roll; msg.request.AngleDestination_right.y = pitch; msg.request.AngleDestination_right.z = yaw;
			msg.request.quick = true;
			ROS_INFO("steer to x y z r p y: %f,%f,%f,%f,%f,%f",msg.request.PositionDestination_right.x,msg.request.PositionDestination_right.y,
					msg.request.PositionDestination_right.z,msg.request.AngleDestination_right.x,msg.request.AngleDestination_right.y,msg.request.AngleDestination_right.z);
			if(!(pelvis_move_hand_CB(msg.request,msg.response))){
				ROS_INFO("error when calling pelvis move hand from wheel move hand");
				return false;
			}
			ros::spinOnce();
		}
		ROS_INFO("end steering");
		return true;
	}


	void position1(){
		ros::spinOnce();
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
	                  		for (unsigned int i = 0; i < 16; i++)
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
	                  			atlas_command.k_effort[i] = 0;
	                  		}

	                  		for (unsigned int i = 16; i < n; i++)
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
	                          double l_arm_usy = positions[joints["l_arm_usy"]],
	                        		  l_arm_shx = positions[joints["l_arm_shx"]],
	                        		  l_arm_ely = positions[joints["l_arm_ely"]],
	                        		  l_arm_elx = positions[joints["l_arm_elx"]],
	                        		  l_arm_uwy = positions[joints["l_arm_uwy"]],
	                        		  l_arm_mwx = positions[joints["l_arm_mwx"]],
	                        		  r_arm_usy = positions[joints["r_arm_usy"]],
	                        		  r_arm_shx = positions[joints["r_arm_shx"]],
	                        		  r_arm_ely = positions[joints["r_arm_ely"]],
	                        		  r_arm_elx = positions[joints["r_arm_elx"]],
	                        		  r_arm_uwy = positions[joints["r_arm_uwy"]],
	                        		  r_arm_mwx = positions[joints["r_arm_mwx"]];

	for(unsigned int i = 1; i < 101; i++){
	                          atlas_command.position[joints["l_arm_usy"]] = l_arm_usy+(-0.3-l_arm_usy)*(i/100.0);
	                          atlas_command.position[joints["l_arm_shx"]] = l_arm_shx+(-0.9-l_arm_shx)*(i/100.0);
	                          atlas_command.position[joints["l_arm_ely"]] = l_arm_ely+(1.2-l_arm_ely)*(i/100.0);
	                          atlas_command.position[joints["l_arm_elx"]] = l_arm_elx+(1.5-l_arm_elx)*(i/100.0);
	                          atlas_command.position[joints["l_arm_uwy"]] = l_arm_uwy+(0.6-l_arm_uwy)*(i/100.0);
	                          atlas_command.position[joints["l_arm_mwx"]] = l_arm_mwx+(0.1-l_arm_mwx)*(i/100.0);

	                          atlas_command.position[joints["r_arm_usy"]] = r_arm_usy+(-0.3-r_arm_usy)*(i/100.0);
	                          atlas_command.position[joints["r_arm_shx"]] = r_arm_shx+(0.9-r_arm_shx)*(i/100.0);
	                          atlas_command.position[joints["r_arm_ely"]] = r_arm_ely+(1.2-r_arm_ely)*(i/100.0);
	                          atlas_command.position[joints["r_arm_elx"]] = r_arm_elx+(-1.5-r_arm_elx)*(i/100.0);
	                          atlas_command.position[joints["r_arm_uwy"]] = r_arm_uwy+(0.6-r_arm_uwy)*(i/100.0);
	                          atlas_command.position[joints["r_arm_mwx"]] = r_arm_mwx+(0.1-r_arm_mwx)*(i/100.0);



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

	                          atlas_command.position[joints["neck_ay"]] = positions[joints["neck_ay"]];
	                          atlas_command.position[joints["back_lbz"]] = positions[joints["back_lbz"]];
	                          atlas_command.position[joints["back_mby"]] = positions[joints["back_mby"]];
	                          atlas_command.position[joints["back_ubx"]] = positions[joints["back_ubx"]];

	                          pub_joint_commands_.publish(atlas_command);
	                          ros::Duration(0.05).sleep();
	                          ros::spinOnce();
	}
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



