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
#include <pr2_controllers_msgs/JointControllerState.h>
#include <C45_PostureControl/back_lbz_poller_service.h>
#include <std_msgs/Float64.h>
#include <string>

class back_lbz_poller{
private:
	ros::NodeHandle nh_;
	ros::ServiceServer back_lbz_state_srv_;
	ros::Subscriber back_lbz_state_sub_;
	double back_lbz_state_;

public:
	back_lbz_poller(){

		back_lbz_state_sub_ = nh_.subscribe("/back_lbz_position_controller/state",100,&back_lbz_poller::back_lbz_state_CB,this);

		back_lbz_state_srv_ = nh_.advertiseService("back_lbz_poller_service", &back_lbz_poller::back_lbz_poller_service, this);
		ROS_INFO("back_lbz_poller_service started");
		  }
	~back_lbz_poller(){}
	void back_lbz_state_CB(const pr2_controllers_msgs::JointControllerStateConstPtr& back_lbz_state){
		back_lbz_state_=back_lbz_state->process_value;
	}

	bool back_lbz_poller_service(C45_PostureControl::back_lbz_poller_service::Request &req, C45_PostureControl::back_lbz_poller_service::Response &res){
		//ROS_INFO("back_lbz_poller service: %f", this->back_lbz_state_);
		res.state.process_value = this->back_lbz_state_;
		return true;
	}

};

int main(int argc, char **argv) {
		ros::init(argc, argv, "back_lbz_poller");

		back_lbz_poller back_lbz_poll = back_lbz_poller();
		ROS_INFO("Running back_lbz_poller service");
		ros::spin();

	 return 0;
}
