#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <boost/thread.hpp>
#include <C0_RobilTask/RobilTaskAction.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>

class turn_in_place{
private:
	ros::NodeHandle nh_;
	ros::Publisher turn_angle;
	ros::Subscriber turn_sub_;
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	ros::Publisher back_mby_pub,back_ubx_pub;
	std::string action_name_;
	actionlib::SimpleActionClient<C0_RobilTask::RobilTaskAction> ac;


public:
	turn_in_place(std::string side):
		ac(nh_, "PickUp",true){
		ROS_INFO("Waiting.");
		ac.waitForServer();
		ROS_INFO("Action server started");
		C0_RobilTask::RobilTaskGoal goal;
		ROS_INFO("S: %s", side.c_str());
		std::string param;
		param = "pipe";
		goal.parameters = param;
		// send a goal to the action
		ac.cancelAllGoals();
		ros::Time t1 = ros::Time::now();
		ac.sendGoal(goal);
		ROS_INFO("Goal sent");
	}

	~turn_in_place(){
		nh_.shutdown();
	}

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
