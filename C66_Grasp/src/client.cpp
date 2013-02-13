/*
 * This class controls turns the robot in place.
 * It stops the stability_maintainer_action action and rotates the back_lbz controller.
 * Dependencies:
 * 		/back_lbz_position_controller/command topic
 * 		C45_PostureControl_maintain_stability action
 *
 * Subscribes:
 * 		C45_PostureControl topic
 */



#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
//#include <C45_PostureControl/C45_PostureControlAction.h>
//#include <C45_PostureControl/maintain_postureAction.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>

class turn_in_place{
private:
	ros::NodeHandle nh_;
	ros::Publisher turn_angle;
	ros::Subscriber turn_sub_;
	//actionlib::SimpleActionClient<C45_PostureControl::C45_PostureControlAction> as_;// NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	ros::Publisher back_mby_pub,back_ubx_pub;
	std::string action_name_;
	actionlib::SimpleActionClient<C0_RobilTask::RobilTaskAction> ac;


public:
	turn_in_place(std::string side):
		ac(nh_, "PickUp",true){

		ROS_INFO("Waiting.");
		ROS_INFO("Waiting for maintain_stability action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started");
		C0_RobilTask::RobilTaskGoal goal_to_maintain, goal_to_maintain2;
		ROS_INFO("S: %s", side.c_str());
		std::string param;
		param = "direction=" + side;
		goal_to_maintain.parameters = param;
		// send a goal to the action

		/*actionlib::SimpleClientGoalState state1 = ac.getState();
		ROS_INFO("Status: %s", state1.toString().c_str());*/
		ac.cancelAllGoals();
		ros::Time t1 = ros::Time::now();
		ac.sendGoal(goal_to_maintain/*, boost::bind(&turn_in_place::doneCB, this, _1, _2),
				boost::bind(&turn_in_place::activeCB, this), boost::bind(&turn_in_place::feedbackCB, this, _1)*/);
		ROS_INFO("Goal sent");
	}

	~turn_in_place(){
		nh_.shutdown();
	}

	void turn_and_maintain_stability(std_msgs::Float64 angle){

	}

	/*void goalCB(){
		// create the action client
		double direction;
		 ROS_INFO("Current time: %f", ros::Time::now().toSec());
		 ROS_INFO("Current ok: %d", (nh_.ok())?1:0);
		 ROS_INFO("Current isactive: %d", as_.isActive());
		 //uint32_t side(as_.acceptNewGoal()->turn_to);
		 direction = side*0.612;

		 ROS_INFO("Current direction: %f", direction);
		 ROS_INFO("Current isactive: %d", as_.isActive());
		 ROS_INFO("Current time11: %f", ros::Time::now().toSec());

		 //Maintain posture while not preempted and is requested to maintain posture
//		 if(as_.isNewGoalAvailable()){
//				 direction = as_.acceptNewGoal()->turn_to;
//			 }
//		     if (as_.isPreemptRequested() || !ros::ok())
//		     {
//		    	 ROS_INFO("%s: Preempted", action_name_.c_str());
//		    	 // set the action state to preempted
//		    	 as_.setPreempted();
//		    	 break;
//		     }

	}*/

//	void preemptCB(){
//		ROS_INFO("%s: Preempted", action_name_.c_str());
//	// set the action state to preempted
//		as_.setPreempted();
//		return;
//	}

	void doneCB(const actionlib::SimpleClientGoalState& state,
            const C0_RobilTask::RobilTaskActionResultConstPtr& result){
		ROS_INFO("Done Callback: %d %s", result->result.success, state.toString().c_str());

	}

	void activeCB(){
		ROS_INFO("Active Callback");
	}

	void feedbackCB(const C0_RobilTask::RobilTaskActionFeedbackConstPtr& fb){
		ROS_INFO("Feedback Callback: %f", fb->feedback.complete);
	}
};

int main (int argc, char **argv)
{
	std::string s = "";
	if(argc == 2){
		s = std::string(argv[1]);
	}
	ros::init(argc, argv, "turn_in_place");
	ROS_INFO("S: %s", s.c_str());
	turn_in_place turn_maintain_stability(s);
	ROS_INFO("Ready");
	ros::spin();

  return 0;
}
