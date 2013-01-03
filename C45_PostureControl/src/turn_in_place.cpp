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
#include <C45_PostureControl/PostureControlAction.h>
#include <std_msgs/Float64.h>

class turn_in_place{
private:
	ros::NodeHandle nh_;
	ros::Publisher turn_angle;
	ros::Subscriber turn_sub_;
	C45_PostureControl::PostureControlGoal goal;

public:
	turn_in_place(){
		turn_angle = nh_.advertise<std_msgs::Float64>("/back_lbz_position_controller/command",true);
		goal.maintainPosture = true;
		turn_sub_ =nh_.subscribe("C45_PostureControl",1000,&turn_in_place::turn_and_maintain_stability,this);

	}

	~turn_in_place(){
	}

	void turn_and_maintain_stability(std_msgs::Float64 angle){
		// create the action client
		actionlib::SimpleActionClient<C45_PostureControl::PostureControlAction> ac(nh_,"C45_PostureControl_maintain_stability",true);
		ROS_INFO("Waiting for maintain_stability action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started");
		turn_angle.publish(angle);
		// send a goal to the action
		ac.sendGoal(goal);
	  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "turn_in_place");
  turn_in_place turn_maintain_stability = turn_in_place();
  ROS_INFO("Ready");
  ros::spin();

  return 0;
}
