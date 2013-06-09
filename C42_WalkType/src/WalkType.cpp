#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

#include <C42_WalkType/mud.h>
#include <C42_WalkType/extreme_slope.h>
#include <C42_WalkType/debris.h>

#define VERSION "2013_06_09_12_30"

ros::Time last_mud_notification;
ros::Time last_extreme_slope_notification;
ros::Time last_debris_notification;

bool isMud(){
	ros::Time now = ros::Time::now();
	ros::Duration maxDuration(3.0);
	return (now - last_mud_notification) > maxDuration;
}
bool isExtremeslope(){
	ros::Time now = ros::Time::now();
	ros::Duration maxDuration(3.0);
	return (now - last_extreme_slope_notification) > maxDuration;
}
bool isDebris(){
	ros::Time now = ros::Time::now();
	ros::Duration maxDuration(3.0);
	return (now - last_debris_notification) > maxDuration;
}

enum WALKTYPE{
	DYNAMIC, DISCRETE, CRAWL_MUD
} walk_type = DYNAMIC;

void update_walk_type(){
	if(isDebris()){
		if(isMud()){
			walk_type = CRAWL_MUD;
		}else{
			walk_type = DISCRETE;
		}
	}else{
		if(isExtremeslope()){
			if(isMud()){
				walk_type = CRAWL_MUD;
			}else{
				walk_type = DISCRETE;
			}
		}else{
			if(isMud()){
				walk_type = CRAWL_MUD;
			}else{
				walk_type = DYNAMIC;
			}
		}
	}
}

void cb_mud_notifier(const C42_WalkType::mud::ConstPtr msg){
	last_mud_notification = ros::Time::now();
}

void cb_extreme_slope_notifier(const C42_WalkType::extreme_slope::ConstPtr msg){
	last_extreme_slope_notification = ros::Time::now();
}
void cb_debris_notifier(const C42_WalkType::debris::ConstPtr msg){
	last_debris_notification = ros::Time::now();
}

using namespace std;
using namespace C0_RobilTask;

class task_WalkDynamic:public RobilTask{
public:
	task_WalkDynamic(string name="/whileDynamicWalk"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }
            if (walk_type != DYNAMIC){
            	return TaskResult::FAULT();
            }
            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};
class task_WalkDiscrete:public RobilTask{
public:
	task_WalkDiscrete(string name="/whileDiscreteWalk"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }
            if (walk_type != DISCRETE){
            	return TaskResult::FAULT();
            }
            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};
class task_WalkCrawlMUD:public RobilTask{
public:
	task_WalkCrawlMUD(string name="/whileQuadWalkMUD"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }
            if (walk_type != CRAWL_MUD){
            	return TaskResult::FAULT();
            }
            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};
class task_WalkCrawlHILLS:public RobilTask{
public:
	task_WalkCrawlHILLS(string name="/whileQuadWalkHILLS"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }
            if (walk_type != DISCRETE){
            	return TaskResult::FAULT();
            }
            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};
class task_WalkCrawlDEBRIS:public RobilTask{
public:
	task_WalkCrawlDEBRIS(string name="/whileQuadWalkDEBRIS"):
      RobilTask(name)
   {   }
   TaskResult task(const string& name, const string& uid, Arguments& args){
      while(true){
            if (isPreempt()){
            	return TaskResult::Preempted();
            }
            if (walk_type != DYNAMIC && walk_type != DISCRETE ){
            	return TaskResult::FAULT();
            }
            sleep(1000);
      }
      return TaskResult::FAULT();
   }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "C42_WalkType");
	ros::NodeHandle node;

	ROS_INFO("VERSION: %s",VERSION);

	ros::Subscriber mud_notifier = node.subscribe("/walk_notification/mud", 10, cb_mud_notifier);
	ros::Subscriber extrim_slope_notifier = node.subscribe("/walk_notification/extreme_slope", 10, cb_extreme_slope_notifier);
	ros::Subscriber debris_notifier = node.subscribe("/walk_notification/debris", 10, cb_debris_notifier);

	task_WalkDynamic t_dynamic;
	task_WalkDiscrete t_discrete;
	task_WalkCrawlMUD t_crawl_mod;
	task_WalkCrawlDEBRIS t_crawl_deb;
	task_WalkCrawlHILLS t_crawl_hill;

	ros::Rate rate(2);
	while(ros::ok()){
		ros::spinOnce();
		update_walk_type();
		rate.sleep();
	}
	return 0;
}
