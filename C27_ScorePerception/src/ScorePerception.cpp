#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include <atlas_msgs/VRCScore.h>

#define VERSION "2013_06_11_14_30"

using namespace std;
using namespace C0_RobilTask;
using namespace atlas_msgs;

//typedef atlas_msgs::VRCScore VRCScore;

boost::mutex score_mutex;
VRCScore prev_score;
VRCScore last_score;
/*

uint32 TASK_OTHER=0
uint32 TASK_DRIVING=1
uint32 TASK_WALKING=2
uint32 TASK_MANIPULATION=3

time wall_time
time sim_time
time wall_time_elapsed
time sim_time_elapsed
int32 completion_score
int32 falls
string message
uint32 task_type

 */

void onNewScoreMessage(const VRCScoreConstPtr msg){
	boost::mutex::scoped_lock l(score_mutex);
	prev_score = last_score;
	last_score = *msg;
}

VRCScore getLastScore(){
	boost::mutex::scoped_lock l(score_mutex);
	return last_score;
}
VRCScore getPrevScore(){
	boost::mutex::scoped_lock l(score_mutex);
	return last_score;
}
void getScores(VRCScore &current, VRCScore &prev){
	boost::mutex::scoped_lock l(score_mutex);
	prev = prev_score;
	current = last_score;
}


class task_untilCompletion:public RobilTask{
public:
	task_untilCompletion(string name="/untilCompletion"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
	  int limit = 0;
	  if(args.find("number")!=args.end()){
		  std::stringstream s; s<<args["number"];
		  s>>limit;
	  }
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }

            VRCScore score = getLastScore();
            if(score.completion_score >= limit)
            	return TaskResult::SUCCESS();

            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "C27_ScorePerception");
	ros::NodeHandle node;

	ROS_INFO("VERSION: %s",VERSION);

	task_untilCompletion _task_untilCompletion;

	ros::Rate rate(2);
	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
