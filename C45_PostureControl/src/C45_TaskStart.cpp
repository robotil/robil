#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <ros/ros.h>
#include <C45_PostureControl/C45_PostureControlAction.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <control_toolbox/pid.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <C45_PostureControl/com_error.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <PoseController/back_movement.h>
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
	ros::ServiceClient start_posecontroller_cli_, stop_posecontroller_cli_;

	int time;
	int retValue;
	string outputstr;

public:
	C45_task_start(std::string name, std::vector<string> par):
        RobilTask(name), params(par)
    {
    }

    TaskResult task(const string& name, const string& uid, Arguments& args) {
		try{
			start_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/start");
			//stop_posecontroller_cli_ = nh_.serviceClient<std_srvs::Empty>("/PoseController/stop");
			//int time = this->time;
			std_srvs::Empty e;
			ROS_INFO("%s: %s", _name.c_str(), "CALLED");
			ROS_INFO("%s: %s", _name.c_str(), "STARTING");
			if(start_posecontroller_cli_.call(e)){
				ROS_INFO("TaskStart: call ok.");
			}else{
				ROS_INFO("TaskStart: call fault.");
			}
		}catch(...){
			ROS_INFO("TaskStart: some exception catched.");
		}
//		ROS_INFO("%s: %s", _name.c_str(), "DONE");
//		stop_posecontroller_cli_.call(e);

	ROS_INFO("TaskStart: finished");
        return TaskResult(SUCCESS, "OK");
    }

};


class task_planner: public RobilTask {

public:
	task_planner(std::string name="/resetHeadAll"):
        RobilTask(name)
    {
    }

    TaskResult task(const string& name, const string& uid, Arguments& args) {
	try{
		
	}catch(...){
		ROS_INFO("resetHeadAll: some exception catched.");
	}

	ROS_INFO("resetHeadAll: finished");
        return TaskResult("<plan><seq><tsk name=\"C45_TaskStart\" id=\"IDTaskStart\" /><tsk name=\"resetHead\" id=\"IDresetHead\" /><tsk name=\"C45_TaskStop\" id=\"IDTaskStop\" /></seq></plan>", "MyPlan");
    }

};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C45_TaskStart");
	ROS_INFO("TaskStart: node Running");
	std::vector<string> params;

	C45_task_start posture_control("C45_TaskStart",params);
	task_planner tp;
	
	ROS_INFO("TaskStart: Running posture controller action");
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
