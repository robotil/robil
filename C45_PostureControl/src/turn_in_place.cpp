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
<<<<<<< HEAD
#include <C45_PostureControl/C45_PostureControlAction.h>
//#include <C45_PostureControl/maintain_postureAction.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>
=======
#include <C45_PostureControl/PostureControlAction.h>
#include <std_msgs/Float64.h>
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c

class turn_in_place{
private:
	ros::NodeHandle nh_;
	ros::Publisher turn_angle;
	ros::Subscriber turn_sub_;
<<<<<<< HEAD
	//actionlib::SimpleActionClient<C45_PostureControl::C45_PostureControlAction> as_;// NodeHandle instance must be created before this line.
	C45_PostureControl::C45_PostureControlFeedback feedback_;
	C45_PostureControl::C45_PostureControlResult result_;
	ros::Publisher back_mby_pub,back_ubx_pub;
	std::string action_name_;


public:
	turn_in_place() {
		ROS_INFO("Waiting.");
		actionlib::SimpleActionClient<C45_PostureControl::C45_PostureControlAction> ac(nh_,"C45_PostureControl",true);
		ROS_INFO("Waiting for maintain_stability action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started");
		C45_PostureControl::C45_PostureControlGoal goal_to_maintain;
		goal_to_maintain.turn_to = 0;
		// send a goal to the action
		ac.sendGoal(goal_to_maintain/*, boost::bind(&turn_in_place::doneCB, this, _1, _2),
				boost::bind(&turn_in_place::activeCB, this), boost::bind(&turn_in_place::feedbackCB, this, _1)*/);

		ROS_DEBUG("Goal sent");
=======
	C45_PostureControl::PostureControlGoal goal;

public:
	turn_in_place(){
		turn_angle = nh_.advertise<std_msgs::Float64>("/back_lbz_position_controller/command",true);
		goal.maintainPosture = true;
		turn_sub_ =nh_.subscribe("C45_PostureControl",1000,&turn_in_place::turn_and_maintain_stability,this);
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c

	}

	~turn_in_place(){
	}

	void turn_and_maintain_stability(std_msgs::Float64 angle){
<<<<<<< HEAD

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
            const C45_PostureControl::C45_PostureControlActionResultConstPtr& result){
		ROS_INFO("Done Callback: %d %s", result->result.success, state.toString().c_str());

	}

	void activeCB(){
		ROS_INFO("Active Callback");
	}

	void feedbackCB(const C45_PostureControl::C45_PostureControlActionFeedbackConstPtr& fb){
		ROS_INFO("Feedback Callback: %f", fb->feedback.stabilityQuality);
	}
=======
		// create the action client
		actionlib::SimpleActionClient<C45_PostureControl::PostureControlAction> ac(nh_,"C45_PostureControl_maintain_stability",true);
		ROS_INFO("Waiting for maintain_stability action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started");
		turn_angle.publish(angle);
		// send a goal to the action
		ac.sendGoal(goal);
	  }
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c
};

int main (int argc, char **argv)
{
<<<<<<< HEAD
	ros::init(argc, argv, "turn_in_place");
	turn_in_place turn_maintain_stability = turn_in_place();
	ROS_INFO("Ready");
	ros::spin();
=======
  ros::init(argc, argv, "turn_in_place");
  turn_in_place turn_maintain_stability = turn_in_place();
  ROS_INFO("Ready");
  ros::spin();
>>>>>>> e2257af8996de269162ddd2be0c5cefb72b5fd1c

  return 0;
}
