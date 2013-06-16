#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <ros/ros.h>
#include <C45_PostureControl/C45_PostureControlAction.h>
#include <atlas_msgs/AtlasSimInterfaceCommand.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>

using namespace std;
using namespace C0_RobilTask;

ostream& operator<<(ostream& o, std::vector<string>& s){
	for(size_t i=0;i<s.size()-1;i++)
		o<<s[i]<<',';
	if(s.size()!=0)
		return o<<s[s.size()-1];
	return o;
}

class C45_task_start: public RobilTask {
protected:
	ros::NodeHandle nh_;
	std::vector<string> params;


	int time;
	int retValue;
	string outputstr;

	ros::Subscriber atlas_sim_int_state_sub_;
	ros::Publisher pub_atlas_commands_;

public:
	C45_task_start(std::string name, std::vector<string> par):
        RobilTask(name), params(par)
    {
		pub_atlas_commands_ = nh_.advertise<atlas_msgs::AtlasSimInterfaceCommand>("/atlas/atlas_sim_interface_command", 1, true);
    }

    TaskResult task(const string& name, const string& uid, Arguments& args) {
		int time = this->time;
		atlas_msgs::AtlasSimInterfaceCommand cm;
		std::string behavior;
		behavior = args["Behavior"];

		if(behavior.compare("MANIPULATE") == 0){
			cm.behavior = atlas_msgs::AtlasSimInterfaceCommand::MANIPULATE;
			ROS_INFO("%s: %s", _name.c_str(), "MANIPULATE");
		}
		if(behavior.compare("STAND") == 0){
			cm.behavior = atlas_msgs::AtlasSimInterfaceCommand::STAND;
			ROS_INFO("%s: %s", _name.c_str(), "STAND");
		}
		if(behavior.compare("USER") == 0){
			cm.behavior = atlas_msgs::AtlasSimInterfaceCommand::USER;
			ROS_INFO("%s: %s", _name.c_str(), "USER");
		}

		ROS_INFO("%s: %s", _name.c_str(), "CALLED");
		ROS_INFO("%s: %s", _name.c_str(), "STARTING");
//		ROS_INFO("%s: %s", _name.c_str(), "DONE");

		pub_atlas_commands_.publish(cm);

		ros::Duration(0.3).sleep();

        return TaskResult(SUCCESS, "OK");
    }

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C45_Behavior");
	ROS_INFO("Running");
	std::vector<string> params;

	C45_task_start* posture_control = new C45_task_start("C45_Behavior",params);
	ROS_INFO("Running posture controller action");
	ros::spin();
	return 0;
}
///*
// * This class maintains the posture stability.
// * Dependencies:
// * 		com_error service
// * 		/back_mby_position_controller/command topic
// * 		/back_ubx_position_controller/command topic
// *
// * Advertises:
// * 		posture_controller_action action
// */
//
//
//#include <ros/ros.h>
////#include <C45_PostureControl/C45_PostureControlAction.h>
//#include <C0_RobilTask/RobilTaskAction.h>
//#include <C0_RobilTask/RobilTask.h>
//#include <control_toolbox/pid.h>
//#include <actionlib/server/simple_action_server.h>
//#include <pr2_controllers_msgs/JointControllerState.h>
//#include <control_msgs/FollowJointTrajectoryAction.h>
//#include <actionlib/client/simple_action_client.h>
//#include <std_srvs/Empty.h>
////#include <C45_PostureControl/back_lbz_poller_service.h>
//#include <C45_PostureControl/com_error.h>
//#include <std_msgs/Float64.h>
//#include <math.h>
//#include <boost/bind.hpp>
//#include <string>
//#include <sensor_msgs/JointState.h>
//#include <osrf_msgs/JointCommands.h>
//#include <PoseController/back_movement.h>
//
//class C45_task{
//private:
//	ros::NodeHandle nh_, nh2_, nh3_, nh4_;
//	ros::NodeHandle* rosnode;
//	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
//	C0_RobilTask::RobilTaskFeedback feedback_;
//	C0_RobilTask::RobilTaskResult result_;
//	control_toolbox::Pid back_ubx_stab_pid,back_mby_stab_pid; // PIDs for stability
//	ros::Publisher turn_angle;
//	std::string action_name_;
//	std_msgs::Float64 float64_msg;
//	ros::Time clock;
//	std::map <std::string, int> joints;
//	std::vector<double> positions;
//	ros::Publisher pub_joint_commands_;
//	ros::Subscriber joint_states_sub_;
//	ros::ServiceClient backz_cli, start_posecontroller_cli_, stop_posecontroller_cli_;
//
//public:
//	C45_task(std::string name)
//	:as_(nh_, name, false), action_name_(name){
//
//		rosnode = new ros::NodeHandle();
//		joint_states_sub_ = nh_.subscribe("/atlas/joint_states",100,&C45_task::joint_states_CB,this);
//		backz_cli = nh2_.serviceClient<PoseController::back_movement>("/PoseController/delta_back_movement");
//
//		start_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/start");
//		stop_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/stop");
//
//
//		//Set callback functions
//		as_.registerGoalCallback(boost::bind(&C45_task::goalCB, this));
//		as_.registerPreemptCallback(boost::bind(&C45_task::preemptCB, this));
//
//
//		ROS_INFO("starting");
//		as_.start();
//		ROS_INFO("started");
//	}
//	~C45_task(){}
//
//	void joint_states_CB(const sensor_msgs::JointStateConstPtr& state){
//		for(unsigned int i=0; i < state->name.size(); i++){
//			joints[state->name[i]] = i;
//		}
//		positions = state->position;
//	}
//
//	void goalCB(){
//		//Init variables
//		ros::spinOnce();
//		clock = ros::Time::now();
//
//		ROS_INFO("Start time: %f", ros::Time::now().toSec());
//
//		double direction = 0;
//		std::string goal_params = as_.acceptNewGoal()->parameters;
//		if(goal_params == "direction=1"){
//			direction = 0.612;
//		}else{
//			if(goal_params == "direction=0"){
//				direction = 0;
//			}else{
//				if(goal_params == "direction=-1"){
//					direction = -0.612;
//				}else{
//					ROS_ERROR("Bad parameter for direction");
//					return;
//				}
//			}
//		}
//
//		ROS_INFO("%s goal: direction %f", action_name_.c_str(), direction);
//
//		PoseController::back_movement back;
//		double total_time = 1.0;
//		int segments = 50;
//		//ROS_INFO("Got back_lbz %f", positions[joints["back_lbz"]]);
//		double velocity = (direction - positions[joints["back_lbz"]])/total_time;
//		std_srvs::Empty e;
//		start_posecontroller_cli_.call(e);
//		//ROS_INFO("velocity %f", velocity);
//		for(int i = 0; i < segments; i++){
//			ros::spinOnce();
//			back.request.back_lbz = velocity/segments;
//			if(!backz_cli.call(back)){
//		    	 ROS_ERROR("%s: aborted", action_name_.c_str());
//		    	 // set the action state to preempted
//		    	 as_.setAborted();
//		    	 break;
//			}
//			ros::Duration(total_time/segments).sleep();
//		}
//		stop_posecontroller_cli_.call(e);
//
//	    double dist;
//
//		C0_RobilTask::RobilTaskResult _res;
//		_res.success = C0_RobilTask::RobilTask::SUCCESS;
//		as_.setSucceeded(_res);
//		ROS_INFO("End time: %f", ros::Time::now().toSec());
//		return;
//	}
//
//	void preemptCB()
//	{
//		ROS_ERROR("%s: Preempted", action_name_.c_str());
//		as_.setPreempted();
//	}
//};
//
//int main(int argc, char **argv) {
//	ros::init(argc, argv, "C45_PostureControl_controller");
//
//	C45_task* posture_control = new C45_task("C45_PostureControl");
//	ROS_INFO("Running posture controller action");
//	ros::spin();
//
//	return 0;
//}
