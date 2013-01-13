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
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>

class stability_maintainer{
private:
	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	C45_PostureControl::com_error com_error_srv;
	ros::ServiceClient COM_error_client;
	control_toolbox::Pid back_ubx_stab_pid,back_mby_stab_pid; // PIDs for stability
	ros::Publisher back_mby_pub,back_ubx_pub;
	ros::Publisher turn_angle;
	std::string action_name_;
	std_msgs::Float64 float64_msg;
	ros::Time clock;

public:
	stability_maintainer(std::string name)
	:as_(nh_, name, false), action_name_(name) {

		//Read PID from rosparam server for the PID constants
		double p,i,d;
		if (ros::param::get("/PID_gains/P", p)){

		}
		else{
			ROS_ERROR("/PID_gains/P was not set");
			return;
		}
		if (ros::param::get("/PID_gains/I", i))
		{
		}
		else{
			ROS_ERROR("/PID_gains/I was not set");
			return;
		}
		if (ros::param::get("/PID_gains/D", d))
		{
		}
		else{
			ROS_ERROR("/PID_gains/D was not set");
			return;
		}
		back_mby_stab_pid.initPid(p,i,d,M_PI,-M_PI);
		back_ubx_stab_pid.initPid(p,i,d,M_PI,-M_PI);
	    COM_error_client = nh_.serviceClient<C45_PostureControl::com_error>("com_error");

	    //Set callback functions
	    as_.registerGoalCallback(boost::bind(&stability_maintainer::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&stability_maintainer::preemptCB, this));

		//
		back_mby_pub = nh2_.advertise<std_msgs::Float64>("/back_mby_position_controller/command",1000,true);
		back_ubx_pub = nh3_.advertise<std_msgs::Float64>("/back_ubx_position_controller/command",1000,true);
		turn_angle = nh4_.advertise<std_msgs::Float64>("/back_lbz_position_controller/command",true);
		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
		  }
	~stability_maintainer(){}
	 void goalCB(){

		 //Init variables
		 back_mby_stab_pid.reset();
		 back_ubx_stab_pid.reset();
		 clock = ros::Time::now();
		 ROS_INFO("Current time: %f", ros::Time::now().toSec());
		 ROS_INFO("Current ok: %d", (nh_.ok())?1:0);
		 ROS_INFO("Current isactive: %d", as_.isActive());

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
		 //ROS_INFO("Current maintainPosture: %d", turn_to);
		 ROS_INFO("Current isactive: %d", as_.isActive());
		 ROS_INFO("Current time11: %f", ros::Time::now().toSec());

		 std_msgs::Float64 d;
		 d.data = direction;
		 turn_angle.publish(d);
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
		 C0_RobilTask::RobilTaskResult _res;
		 _res.success = 1;
		 as_.setSucceeded(_res);
		 ROS_INFO("ENd: Current time: %f", ros::Time::now().toSec());
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

		stability_maintainer stab_maintainer("C45_PostureControl");
		ROS_INFO("Running stability maintainer action");
		ros::spin();

	 return 0;
}
