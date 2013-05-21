/*
 * This class maintains the posture stability.
 * Dependencies:
 *
 * Advertises:
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
//#include <C45_PostureControl/back_lbz_poller_service.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <osrf_msgs/JointCommands.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/WrenchStamped.h>
#include <atlas_msgs/ForceTorqueSensors.h>
//#include <hrl_kinematics/TestStability.h>
#include <C43_LocalBodyCOM/Kinematics.h>
#include <C43_LocalBodyCOM/CoM_Array_msg.h>
#include <PoseController/back_movement.h>
#include <tf/transform_listener.h>
#include <pelvis_leg_target/static_leg.h>


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
	//C45_PostureControl::back_lbz_poller_service back_;
	ros::Time clock;
	ros::Subscriber back_lbz_state_sub_;
	double back_lbz_state_;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;
	ros::Subscriber joint_states_sub_, imu_sub_, pcom_sub_;
	ros::Subscriber force_sub_;
	geometry_msgs::Point imu, imu_v;
	atlas_msgs::ForceTorqueSensors forcesensors;
	//geometry_msgs::Point com;
	ros::ServiceClient posecontroller_cli, static_leg_cli;
	tf::TransformListener listener;
	C43_LocalBodyCOM::CoM_Array_msg com;

public:
	stability_maintainer(std::string name)
	:as_(nh2_, name, false), action_name_(name){
		/*back_mby_stab_pid.initPid(p,i,d,M_PI,-M_PI);
		back_ubx_stab_pid.initPid(p,i,d,M_PI,-M_PI);*/

		//rosnode = new ros::NodeHandle();
		positions.resize(28);
		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&stability_maintainer::joint_states_CB,this);
		force_sub_ = nh_.subscribe(/*"/atlas/force_torque_sensors"*/"/filtered_contacts",100,&stability_maintainer::force_CB,this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100, &stability_maintainer::imu_CB,this);
		//pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);
		pcom_sub_ = nh_.subscribe("/c43_local_body/com", 1, &stability_maintainer::get_com_from_hrl_kinematics, this);

		posecontroller_cli = nh2_.serviceClient<PoseController::back_movement>("/PoseController/back_movement");

		while(!posecontroller_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /PoseController/back_movement service");
		}


		while(force_sub_.getNumPublishers() == 0){
			std::string c = std::string("Waiting for the ")+ force_sub_.getTopic() + " topic";
			ROS_INFO(c.c_str());
			ros::spinOnce();
			ros::Duration(0.1).sleep();
		}

		/*static_leg_cli = nh2_.serviceClient<pelvis_leg_target::static_leg>("/current_static_leg");

		while(!static_leg_cli.waitForExistence(ros::Duration(1.0))){
			ROS_INFO("Waiting for the /current_static_leg");
		}*/

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&stability_maintainer::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&stability_maintainer::preemptCB, this));

		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~stability_maintainer(){
		as_.setAborted();
	}
	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		positions = state->position;
	}

	//Get foot contact
	void force_CB(const atlas_msgs::ForceTorqueSensorsConstPtr& contact){
		forcesensors.header = contact->header;
		forcesensors.l_foot = contact->l_foot;
		forcesensors.r_foot = contact->r_foot;
	}

	void get_com_from_hrl_kinematics(const C43_LocalBodyCOM::CoM_Array_msgConstPtr& comArray){
		com = *comArray;
		//this->_CoM.x = com.x();
		//this->_CoM.y = com.y();
	}

	double QuatToRoll(double x, double y, double z, double w){
		return atan2(2*(w*x + y*z), 1 - 2*(pow(x,2) + pow(y,2)));
	}
	double QuatToPitch(double x, double y, double z, double w){
		return asin(2*(w*y - z*x));
	}
	double QuatToYaw(double x, double y, double z, double w){
		return atan2(2*(w*z + x*y), 1 - 2*(pow(y,2) + pow(z,2)));
	}

	void imu_CB(const sensor_msgs::ImuConstPtr& imuC){

		double x = imuC->orientation.x;
		double y = imuC->orientation.y;
		double z = imuC->orientation.z;
		double w = imuC->orientation.w;
		/*Roll*/	imu.x = atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
		/*Pitch*/	imu.y = asin(-2*(x*z - w*y));
		/*Yaw*/		imu.z = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
		imu_v.x = imuC->angular_velocity.x;
		imu_v.y = imuC->angular_velocity.y;
		imu_v.z = imuC->angular_velocity.z;

	}

	void goalCB(){
		int segments = 1000;
		double total_time = 1;

		if(pcom_sub_.getNumPublishers() == 0){
			ROS_ERROR("CoM topic must be published");
			C0_RobilTask::RobilTaskResult _res;
			_res.success = C0_RobilTask::RobilTask::FAULT;
			as_.setAborted(_res);
			ROS_INFO("End time: %f", ros::Time::now().toSec());
			return;
		}

		clock = ros::Time::now();
		ROS_INFO("Start time: %f", ros::Time::now().toSec());
		std::string goal_params = as_.acceptNewGoal()->parameters;

		//double start_back_mby = positions[joints["back_mby"]];
		//double start_back_ubx = positions[joints["back_ubx"]];
		//double part_roll = (imu.x-start_back_ubx) / segments;
		//double part_pitch = (imu.y-start_back_mby) / segments;
		double dt = 0.01;

		double oldx = 0, oldy = 0;
		double kproll = 1;
		double kppitch = 1;
		//for(int i=0; i<segments; i++){
		while(nh2_.ok()){
			ros::spinOnce();
			if(as_.isPreemptRequested()){
				ROS_ERROR("%s: Preempted 53453", action_name_.c_str());
				as_.setAborted();
				break;
			}
			/*jointcommands.position[joints["l_leg_uhz"]] = positions[joints["l_leg_uhz"]];
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
			jointcommands.position[joints["r_arm_mwx"]] = positions[joints["r_arm_mwx"]];*/
			//jointcommands.position[joints["neck_ay"]] = 0;//positions[joints["neck_ay"]];

			double t = ros::Time::now().sec;
			/*jointcommands.position[joints["back_lbz"]] = 0.01*sin(t);//positions[joints["back_lbz"]];
			jointcommands.velocity[joints["back_lbz"]] = 0.01*cos(t);//positions[joints["back_lbz"]];
			jointcommands.kp_velocity[joints["back_lbz"]] = 1;//positions[joints["back_lbz"]];
			jointcommands.kp_position[joints["back_lbz"]] = 20;//positions[joints["back_lbz"]];
			jointcommands.ki_position[joints["back_lbz"]] = 10;//positions[joints["back_lbz"]];
			jointcommands.kd_position[joints["back_lbz"]] = 5;//positions[joints["back_lbz"]];*/
			//jointcommands.position[joints["back_mby"]] =  /*positions[joints["back_mby"]];//0;//*/1.0*-imu.y + 0.0*imu_v.y;//
			//jointcommands.velocity[joints["back_mby"]] = (positions[joints["back_mby"]]-(imu.y))/dt;//positions[joints["back_lbz"]];
			//jointcommands.kp_velocity[joints["back_mby"]] = 0.005;//positions[joints["back_lbz"]];
			//jointcommands.kp_position[joints["back_mby"]] = 10000;//positions[joints["back_lbz"]];
			//jointcommands.ki_position[joints["back_mby"]] = 2;//positions[joints["back_lbz"]];
			//jointcommands.kd_position[joints["back_mby"]] = 50;//positions[joints["back_lbz"]];
			//jointcommands.position[joints["back_ubx"]] =  /*positions[joints["back_ubx"]];//0;//*/1.0*-imu.x + 0.0*imu_v.y;//
			/*jointcommands.velocity[joints["back_ubx"]] = (positions[joints["back_ubx"]]+(imu.x))/dt;//positions[joints["back_lbz"]];
			jointcommands.kp_velocity[joints["back_ubx"]] = 1;//positions[joints["back_lbz"]];
			jointcommands.kp_position[joints["back_ubx"]] = 20;//positions[joints["back_lbz"]];
			jointcommands.ki_position[joints["back_ubx"]] = 10;//positions[joints["back_lbz"]];
			jointcommands.kd_position[joints["back_ubx"]] = 5;//positions[joints["back_lbz"]];*/
			/*ROS_INFO("Sending to lbz(%d): %f", joints["back_lbz"], 0.1*sin(t));
			ROS_INFO("Velocity: %f ;kp velocity:%f", jointcommands.velocity[joints["back_lbz"]], jointcommands.kp_velocity[joints["back_lbz"]]);
			ROS_INFO("PID: %f %f %f", jointcommands.kp_position[joints["back_lbz"]], jointcommands.ki_position[joints["back_lbz"]], jointcommands.kd_position[joints["back_lbz"]]);*/
			/*ROS_INFO("Sending imu.pitch to uby(%d): %f", joints["back_mby"], -imu.y);
			ROS_INFO("Sending imu.roll to ubx(%d): %f", joints["back_ubx"], -imu.x);
			ROS_INFO("Sending neck (%d): %f", joints["neck_ay"], positions[joints["neck_ay"]]);*/
			/*jointcommands.effort[joints["back_ubx"]]       = com.y * kproll;
			jointcommands.effort[joints["back_mby"]]       = com.x * kppitch;*/

			// To be reached 1+ind second after the start of the trajectory
			//current_dt += traj_vec_srv.response.dt[ind];
			//pub_joint_commands_.publish(jointcommands);

/*
			pelvis_leg_target::static_leg static_leg;
			if(!static_leg_cli.call(static_leg)){
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setSucceeded(_res);
				return;
			}*/

			tf::StampedTransform pelvis2left,pelvis2right;
			try {
				listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),pelvis2left);
				listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),pelvis2right);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}


			geometry_msgs::Point optimal_com;
			//optimal_com.x = (forcesensors.l_foot.force.z * pelvis2left.getOrigin().x() + forcesensors.r_foot.force.z * pelvis2right.getOrigin().x())/(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z);
			//optimal_com.y = (forcesensors.l_foot.force.z * pelvis2left.getOrigin().y() + forcesensors.r_foot.force.z * pelvis2right.getOrigin().y())/(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z);
			double distl = sqrt(pow(com.x[1],2)+pow(com.y[1],2));
			double distr = sqrt(pow(com.x[2],2)+pow(com.y[2],2));

			double force_distance = forcesensors.r_foot.force.z - forcesensors.l_foot.force.z;
			if(force_distance < -0.8*(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z)){
				optimal_com.x = (distl * com.x[1])
										/((distl+distr));
				optimal_com.y = (distl * com.y[1])
											/((distl+distr));
			}else{
				if(force_distance > 0.8*(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z)){
					optimal_com.x = (distr * com.x[2])
											/((distl+distr));
					optimal_com.y = (distr * com.y[2])
												/((distl+distr));
				}else{
					optimal_com.x = (distr * com.x[2] + distl * com.x[1])
											/((distl+distr));
					optimal_com.y = (distr * com.y[2] + distl * com.y[1])
												/((distl+distr));
				}
			}

			/*optimal_com.x = (distr * com.x[2] * forcesensors.r_foot.force.z + distl * com.x[1] * forcesensors.l_foot.force.z)
							/((distl+distr)*(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z));
			optimal_com.y = (distr * com.y[2] * forcesensors.r_foot.force.z + distl * com.y[1] * forcesensors.l_foot.force.z)
							/((distl+distr)*(forcesensors.l_foot.force.z+forcesensors.r_foot.force.z));*/
			double CoM_P = 1.0;
			double CoM_D = 0.0;
			double leg_com_x = (optimal_com.x - com.x[0]);
			double leg_com_y = (optimal_com.y - com.y[0]);
			double dleg_com_x = (leg_com_x - oldx)/dt;
			double dleg_com_y = (leg_com_y - oldy)/dt;
			//ROS_INFO("com x: optimal: %f current: %f", optimal_com.x, com.x[0]);
			//ROS_INFO("com y: optimal: %f current: %f", optimal_com.y, com.y[0]);
			//ROS_INFO("leg_com_x: %f leg_com_y: %f", leg_com_x, leg_com_y);
/*			if (static_leg.response.leg == "r_foot"){
					leg_com_x = com.x[1];
					leg_com_y = com.y[1];
			}else{
				if (static_leg.response.leg == "l_foot"){
					leg_com_x = com.x[2];
					leg_com_y = com.y[2];
				}else{
					leg_com_x = com.x[0];
					leg_com_y = com.y[0];
				}
			}*/

			PoseController::back_movement move;
			move.request.back_mby = 0.0*-imu.y + 0.0*imu_v.y + CoM_P*optimal_com.x;// + CoM_D*dleg_com_x;
			move.request.back_ubx = 0.0*-imu.x + 0.0*imu_v.x - 0.3*CoM_P*optimal_com.y;// - CoM_D*dleg_com_y ;
			move.request.back_lbz = -100;
			oldx = leg_com_x;
			oldy = leg_com_y;
			if(!posecontroller_cli.call(move)){
				ROS_ERROR("Could not call PoseController");
				//as_.setPreempted();
				as_.setAborted();
			}
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
		ROS_ERROR("%s: Preempted 43214", action_name_.c_str());
		//as_.setPreempted();
		as_.setAborted();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C45_PostureControl_maintain_stability");

	stability_maintainer* stab_maintainer = new stability_maintainer("MaintainStability");
	ROS_INFO("Running stability maintainer action");
	ros::spin();

	return 0;
}
