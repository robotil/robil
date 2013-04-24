
#include <ros/ros.h>

#include "Tasks_stub.hpp"


int main(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner_stub");
	  ros::NodeHandle node;

	  ROS_INFO("Start node : gpp_qual1.1");

	  //Main tasks
	  PathPlanning task_pathplanning;
	  PathPlanningFocus task_pathplanningfocus;

	  ROS_INFO("spin node");
	  ros::spin();
	  return 0;


}





