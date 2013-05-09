/*
 * This class maintains the posture stability.
 * Dependencies:
 * 		com_error service
 * 		/back_mby_position_controller/command topic
 * 		/back_ubx_position_controller/command topic
 *
 * Advertises:
 * 		posture_controller_action action
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
#include <std_srvs/Empty.h>
//#include <C45_PostureControl/back_lbz_poller_service.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <PoseController/back_movement.h>

class posture_controller{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	ros::NodeHandle* rosnode;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	control_toolbox::Pid back_ubx_stab_pid,back_mby_stab_pid; // PIDs for stability
	ros::Publisher turn_angle;
	std::string action_name_;
	std_msgs::Float64 float64_msg;
	ros::Time clock;
	std::map <std::string, int> joints;
	std::vector<double> positions;
	ros::Publisher pub_joint_commands_;
	ros::Subscriber joint_states_sub_;
	ros::ServiceClient backz_cli, start_posecontroller_cli_, stop_posecontroller_cli_;

public:
	posture_controller(std::string name)
	:as_(nh_, name, false), action_name_(name){

		rosnode = new ros::NodeHandle();
		/*back_mby_stab_pid.initPid(p,i,d,M_PI,-M_PI);
		back_ubx_stab_pid.initPid(p,i,d,M_PI,-M_PI);*/
		//COM_error_client = nh_.serviceClient<C45_PostureControl::com_error>("com_error");
		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&posture_controller::joint_states_CB,this);
		backz_cli = nh2_.serviceClient<PoseController::back_movement>("/PoseController/delta_back_movement");

		start_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/start");
		stop_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/stop");


		//Set callback functions
		as_.registerGoalCallback(boost::bind(&posture_controller::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&posture_controller::preemptCB, this));


		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~posture_controller(){}
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

	void goalCB(){
		//Init variables
		/*back_mby_stab_pid.reset();
		 back_ubx_stab_pid.reset();*/
		ros::spinOnce();
		clock = ros::Time::now();

		ROS_INFO("Start time: %f", ros::Time::now().toSec());
		/*ROS_INFO("Current ok: %d", (nh_.ok())?1:0);
		 ROS_INFO("Current isactive: %d", as_.isActive());*/

		double direction = 0;
		std::string goal_params = as_.acceptNewGoal()->parameters;
		if(goal_params == "direction=1"){
			direction = 0.612;
		}else{
			if(goal_params == "direction=0"){
				direction = 0;
			}else{
				if(goal_params == "direction=-1"){
					direction = -0.612;
				}else{
					ROS_ERROR("Bad parameter for direction");
					return;
				}
			}
		}

		ROS_INFO("%s goal: direction %f", action_name_.c_str(), direction);
		//ROS_INFO("Current maintainPosture: %d", turn_to);
		/*ROS_INFO("Current isactive: %d", as_.isActive());
		 ROS_INFO("Current time11: %f", ros::Time::now().toSec());*/

		PoseController::back_movement back;
		double total_time = 1.0;
		int segments = 50;
		//ROS_INFO("Got back_lbz %f", positions[joints["back_lbz"]]);
		double velocity = (direction - positions[joints["back_lbz"]])/total_time;
		std_srvs::Empty e;
		start_posecontroller_cli_.call(e);
		//ROS_INFO("velocity %f", velocity);
		for(int i = 0; i < segments; i++){
			ros::spinOnce();
			back.request.back_lbz = velocity/segments;
			//ROS_INFO("velocity #%d: %f", i+1, back.request.back_lbz);
			//ROS_INFO("#%d joint: %f", i+1, positions[joints["back_lbz"]]);
			//ROS_INFO("#%d direction: %f", i+1, direction);
			if(!backz_cli.call(back)){
		    	 ROS_ERROR("%s: aborted", action_name_.c_str());
		    	 // set the action state to preempted
		    	 as_.setAborted();
		    	 break;
			}
			ros::Duration(total_time/segments).sleep();
		}
		stop_posecontroller_cli_.call(e);

	    /*goal.trajectory.joint_names.push_back("back_lbz" );
		ROS_INFO("Pushed back");
	    goal.trajectory.points.resize(10);
	    for(int ind = 0; ind < 10; ind++){
			ROS_INFO("set %d position", ind);
			goal.trajectory.points[ind].positions.push_back((1/(pow(10-(double)ind,2)))*direction);
			ROS_INFO("set %d time", ind);
    		goal.trajectory.points[ind].time_from_start = ros::Duration(0.5*ind);
			ROS_INFO("set %d", ind);
	    }
    	goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
    	ROS_INFO("sending trajectory");
    	traj_client_.sendGoal(goal);*/
/*
		std_msgs::Float64 d;
		d.data = direction;
		turn_angle.publish(d);
*/
		/*double start_back_lbz = positions[joints["back_lbz"]];
		double part = (direction-positions[joints["back_lbz"]]) / segments;
		for(int i=0; i<segments; i++){
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
			jointcommands.position[joints["back_lbz"]] = start_back_lbz+(part*(i+1));
			jointcommands.position[joints["back_mby"]] = positions[joints["back_mby"]];
			jointcommands.position[joints["back_ubx"]] = positions[joints["back_ubx"]];

			// To be reached 1+ind second after the start of the trajectory
			//current_dt += traj_vec_srv.response.dt[ind];
			pub_joint_commands_.publish(jointcommands);
			ros::Duration((total_time*1.0)/segments).sleep();
		}*/




		//Maintain posture while not preempted and is requested to maintain posture
		/*while (nh_.ok() && as_.isActive()){
		      // check that preempt was not requested by the client
			 if(as_.isNewGoalAvailable()){
				 ROS_ERROR("Got new goal");
				 goal_params = as_.acceptNewGoal()->parameters;
				 break;
			 }
		     if (as_.isPreemptRequested() || !ros::ok())
		     {
		    	 ROS_ERROR("%s: Preempted", action_name_.c_str());
		    	 // set the action state to preempted
		    	 as_.setPreempted();
		    	 break;
		     }
		     ROS_INFO("Current goal: %s", goal_params.c_str());

		     //Update PID
			 COM_error_client.call(com_error_srv);

			 ROS_INFO("CoM: x: %f y:%f", com_error_srv.response.x_error, com_error_srv.response.y_error);

			 back_mby_stab_pid.updatePid(com_error_srv.response.x_error,ros::Time::now()-clock);
			 back_ubx_stab_pid.updatePid(com_error_srv.response.y_error,ros::Time::now()-clock);
			 clock = ros::Time::now();
			 float64_msg.data = -back_mby_stab_pid.getCurrentCmd();
			 back_mby_pub.publish(float64_msg);
			 float64_msg.data = -back_ubx_stab_pid.getCurrentCmd();
			 back_ubx_pub.publish(float64_msg);
			 feedback_.complete=sqrt(pow(com_error_srv.response.x_error,2)+pow(com_error_srv.response.y_error,2));
			 ROS_INFO("Last stability quality: %f", feedback_.complete);
			 as_.publishFeedback(feedback_);
			 ros::Duration(0.01).sleep();
		 }
	     if (as_.isPreemptRequested() || !ros::ok())
	      {
	        ROS_ERROR("%s: Preempted", action_name_.c_str());
	        // set the action state to preempted
	        as_.setPreempted();
	      }*/
		double dist;
		/*do{
			if(back_lbz_poller_cli_.call(back_)){
				back_lbz_state_ = back_.response.state.process_value;
				dist = fabs(back_lbz_state_ - direction);
				//ROS_INFO("Dist: %f", dist);
			}else{
				C0_RobilTask::RobilTaskResult _res;
				_res.success = C0_RobilTask::RobilTask::FAULT;
				as_.setAborted(_res);
				return;
			}
		}while(dist > 0.2);*/

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
	ros::init(argc, argv, "C45_PostureControl_controller");

	posture_controller* posture_control = new posture_controller("C45_PostureControl");
	ROS_INFO("Running posture controller action");
	ros::spin();

	return 0;
}
