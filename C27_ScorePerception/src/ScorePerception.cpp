#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

#define VERSION "2013_06_09_12_30"

using namespace std;
using namespace C0_RobilTask;

class task_:public RobilTask{
public:
	task_(string name="/TASK"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }


            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "C27_ScorePerception");
	ros::NodeHandle node;

	ROS_INFO("VERSION: %s",VERSION);

	task_ t_;

	ros::Rate rate(2);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
