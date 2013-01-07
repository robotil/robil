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
#include <C45_PostureControl/PostureControlAction.h>
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>

class stability_maintainer{
private:
	ros::NodeHandle nh_;
	control_toolbox::Pid back_ubx_stab_pid,back_mby_stab_pid; // PIDs for stability
	actionlib::SimpleActionServer<C45_PostureControl::PostureControlAction> as_; // NodeHandle instance must be created before this line.
	std::string action_name_;
	C45_PostureControl::PostureControlFeedback feedback_;
	C45_PostureControl::PostureControlResult result_;
	ros::ServiceClient COM_error_client;
	ros::Time clock;
	C45_PostureControl::com_error com_error_srv;
	ros::Publisher back_mby_pub,back_ubx_pub;
	std_msgs::Float64 float64_msg;

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
		back_mby_pub = nh_.advertise<std_msgs::Float64>("/back_mby_position_controller/command",1000,true);
		back_ubx_pub = nh_.advertise<std_msgs::Float64>("/back_ubx_position_controller/command",1000,true);
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
		 ROS_DEBUG("Current ok: %d", nh_.ok());
		 ROS_DEBUG("Current isactive: %d", as_.isActive());
		 uint8_t maintainPosture(as_.acceptNewGoal()->maintainPosture);
		 ROS_DEBUG("Current maintainPosture: %d", maintainPosture);
		 ROS_DEBUG("Current isactive: %d", as_.isActive());

		 //Maintain posture while not preempted and is requested to maintain posture
		 while (nh_.ok() && as_.isActive() && maintainPosture){
		      // check that preempt was not requested by the client
			 if(as_.isNewGoalAvailable()){
				 maintainPosture = as_.acceptNewGoal()->maintainPosture;
			 }
		     if (as_.isPreemptRequested() || !ros::ok())
		     {
		    	 ROS_INFO("%s: Preempted", action_name_.c_str());
		    	 // set the action state to preempted
		    	 as_.setPreempted();
		    	 maintainPosture = false;
		    	 break;
		     }
		     ROS_DEBUG("Current goal: %d", maintainPosture);

		     //Update PID
			 COM_error_client.call(com_error_srv);

			 back_mby_stab_pid.updatePid(com_error_srv.response.x_error,ros::Time::now()-clock);
			 back_ubx_stab_pid.updatePid(com_error_srv.response.y_error,ros::Time::now()-clock);
			 clock = ros::Time::now();
			 float64_msg.data = -back_mby_stab_pid.getCurrentCmd();
			 back_mby_pub.publish(float64_msg);
			 float64_msg.data = -back_ubx_stab_pid.getCurrentCmd();
			 back_ubx_pub.publish(float64_msg);
			 feedback_.stabilityQuality=sqrt(pow(com_error_srv.response.x_error,2)+pow(com_error_srv.response.y_error,2));
			 as_.publishFeedback(feedback_);
			 ROS_DEBUG("Last stability quality: %f", feedback_.stabilityQuality);
			 ros::Duration(0.01).sleep();
		 }
	     if (as_.isPreemptRequested() || !ros::ok())
	      {
	        ROS_INFO("%s: Preempted", action_name_.c_str());
	        // set the action state to preempted
	        as_.setPreempted();
	        maintainPosture = false;
	      }
		 ROS_DEBUG("ENd: Current time: %f", ros::Time::now().toSec());
		 return;
	 }
	 void preemptCB()
	   {
		 ROS_INFO("%s: Preempted", action_name_.c_str());
		 as_.setPreempted();
	   }


};

int main(int argc, char **argv) {
		ros::init(argc, argv, "stability_maintainer_action");

		stability_maintainer stab_maintainer(ros::this_node::getName());
		ROS_INFO("Running stability maintainer action");
		ros::spin();

	 return 0;
}
