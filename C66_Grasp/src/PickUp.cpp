#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <move_hand/matrix.h>
#include <move_hand/move_hand.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <C23_dFind/perceptionTransform.h>
#include <boost/bind/bind.hpp>
#include <hand_grasp/grasp.h>
#include <move_hand/pelvis_move_hand.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sensor_msgs/JointState.h>
class PickUp{
private:
	ros::NodeHandle nh_;
	ros::ServiceClient pelvis_move_hand_matrix_cli_,perception_transform_cli_,pelvis_move_hand_cli_,move_hand_cli_;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	std::string action_name_;
	ros::NodeHandle* rosnode;
	ros::Publisher pub_hand_grasp_right,pub_hand_grasp_left;

public:
	PickUp(std::string name)
	:as_(nh_, name, false), action_name_(name){
	  rosnode = new ros::NodeHandle();
	  pelvis_move_hand_matrix_cli_ = nh_.serviceClient<move_hand::matrix>("C66_matrix");
	  pelvis_move_hand_cli_ = nh_.serviceClient<move_hand::pelvis_move_hand>("pelvis_move_hand");
	  move_hand_cli_ = nh_.serviceClient<move_hand::move_hand>("move_hand");
	  perception_transform_cli_ = nh_.serviceClient<C23_dFind::perceptionTransform>("perceptionTransform");

		//Set callback functions
		as_.registerGoalCallback(boost::bind(&PickUp::goalCB, this));
		//as_.registerPreemptCallback(boost::bind(&PickUp::preemptCB, this));
		pub_hand_grasp_right = rosnode->advertise<hand_grasp::grasp>("/hand_grasp_right", 100, true);
		pub_hand_grasp_left= rosnode->advertise<hand_grasp::grasp>("/hand_grasp_left", 100, true);
		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~PickUp(){}

	void goalCB(){

		ROS_INFO("Start time: %f", ros::Time::now().toSec());

		std::string g = as_.acceptNewGoal()->parameters;

		switch (g) {
		case "pipe":
			/*open hand*/
			ROS_INFO("opening right hand");
			hand_grasp::grasp grasp_msg;
			grasp_msg.strength = 0;
			pub_hand_grasp_right.publish(grasp_msg);
			ros::Duration(0.3).sleep();

			C23_dFind::perceptionTransform perception_srv_msg;
			perception_srv_msg.request.command = g;
			ROS_INFO("requesting transformation from perception");
			if (perception_transform_cli_.call(perception_srv_msg)){
			  /*init vector for pipe*/
			  move_hand::matrix move_hand_matrix_msg;
			  move_hand_matrix_msg.request.PositionDestination_right.x = 0.46;
			  move_hand_matrix_msg.request.PositionDestination_right.y = -0.071;
			  move_hand_matrix_msg.request.PositionDestination_right.z = -0.1;
			  move_hand_matrix_msg.request.AngleDestination_right.x = -3.14;
			  move_hand_matrix_msg.request.AngleDestination_right.y = 0.5;
			  move_hand_matrix_msg.request.AngleDestination_right.z = 1.442;
			  move_hand_matrix_msg.request.matrix = perception_srv_msg.response.transMat; //matrix from perception
			  if (as_.isPreemptRequested() || !ros::ok())
			                  {
			                          ROS_ERROR("%s: Preempted", action_name_.c_str());
			                          // set the action state to preempted
			                          as_.setPreempted();
			                          return;
			                  }
			  /*moving hand to requested object*/
			  ROS_INFO("moving hand to requested object");
			  if(pelvis_move_hand_matrix_cli_.call(move_hand_matrix_msg)){
			    if (!move_hand_matrix_msg.response.success){
			      as_.setAborted();
			    return;}
			  }
			else{
		                ROS_INFO("ERROR in move_hand service");
		                  as_.setAborted();
		                  return;
			}}
			else {
			  ROS_INFO("ERROR in requesting transformation from perception");
			  as_.setAborted();
			return;
			}
			/*close hand*/
			ROS_INFO("closing right hand");
			grasp_msg.strength = 15;
			pub_hand_grasp_right.publish(grasp_msg);
			ros::Duration(1.2).sleep();

			/*lifting*/

			move_hand::move_hand move_hand_msg;
			move_hand_msg.request.PositionDestination_right.z = 0.2;
			if(!move_hand_cli_.call(move_hand_msg)){
                ROS_INFO("ERROR in move_hand service");
                  as_.setAborted();
                  return;
			}

			break;



		case "insert_pipe": break;



		case "valve": break;


		default: break;
		}






                move_hand::move_hand move_hand_msg;
                move_hand_msg.request.PositionDestination_right.z = 0.3;
                ROS_INFO("lifting");
                if(move_hand_cli_.call(move_hand_msg)){
                move_hand::pelvis_move_hand pel_move_msg;
                pel_move_msg.request.PositionDestination_right.x = 0.684;
                pel_move_msg.request.PositionDestination_right.y = 0.095;
                pel_move_msg.request.PositionDestination_right.z = 0.361;
                pel_move_msg.request.AngleDestination_right.x = -1.790;
                pel_move_msg.request.AngleDestination_right.y = -0.066;
                pel_move_msg.request.AngleDestination_right.z = -0.066;
                }
                else{
                	ROS_INFO("Error in move_hand_service");
	                  as_.setAborted();
	                  return;
                }
                C0_RobilTask::RobilTaskResult _res;
		_res.success = C0_RobilTask::RobilTask::SUCCESS;
		as_.setSucceeded(_res);
		ROS_INFO("End time: %f", ros::Time::now().toSec());
		return;
	}
/*
	void preemptCB()
	{
		ROS_ERROR("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
	}*/
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C66_PickUp");

	PickUp* QuasiStaticWalker = new PickUp("PickUp");
	ROS_INFO("Running PickUp action");
	ros::spin();

	return 0;
}
