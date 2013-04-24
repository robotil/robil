
#include <ros/ros.h>

#include "Tasks.hpp"


int main(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner");
	  ros::NodeHandle node;

	  PathPlanning pathplanning;

	  //Main tasks
	  PathPlanningServer task_pathplanning(pathplanning);
	  PathPlanningFocusServer task_pathplanningfocus(pathplanning);

	  //utilities
	  Task_whileSolution task_whileSolution(pathplanning);

	  ros::spin();
	  return 0;


}





