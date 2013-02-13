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
#include <sensor_msgs/JointState.h>
#include <string>
#include <map>

class back_lbz_poller{
private:
	ros::NodeHandle nh_;
	ros::ServiceServer back_lbz_state_srv_;
	ros::Subscriber back_lbz_state_sub_;
	ros::Subscriber joint_states_sub_;
	std::map <std::string, int> joints;
	double back_lbz_state_;

public:
	back_lbz_poller(){

		joint_states_sub_ = nh_.subscribe("/joint_states",100,&back_lbz_poller::joint_states_CB,this);

		//back_lbz_state_sub_ = nh_.subscribe("/back_lbz_position_controller/state",100,&back_lbz_poller::back_lbz_state_CB,this);

		back_lbz_state_srv_ = nh_.advertiseService("back_lbz_poller_service", &back_lbz_poller::back_lbz_poller_service, this);
		ROS_INFO("back_lbz_poller_service started");
		  }
	~back_lbz_poller(){}
	/*void back_lbz_state_CB(const pr2_controllers_msgs::JointControllerStateConstPtr& back_lbz_state){
		back_lbz_state_=back_lbz_state->process_value;
	}*/

	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
		for(unsigned int i=0; i < state->name.size(); i++){
			joints[state->name[i]] = i;
		}
		back_lbz_state_=state->position[joints["back_lbz"]];
		/*q1_l=state->position[joints["l_leg_lax"]];
		q2_l=state->position[joints["l_leg_lhy"]];
		q3_l=state->position[joints["l_leg_mhx"]];
		q4_l=state->position[joints["l_leg_uay"]];
		q5_l=state->position[joints["l_leg_uhz"]];
		q0_r=state->position[joints["r_leg_kny"]];
		q1_r=state->position[joints["r_leg_lax"]];
		q2_r=state->position[joints["r_leg_lhy"]];
		q3_r=state->position[joints["r_leg_mhx"]];
		q4_r=state->position[joints["r_leg_uay"]];
		q5_r=state->position[joints["r_leg_uhz"]];*/
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
