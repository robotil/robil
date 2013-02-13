
#include <ros/ros.h>

#include "Tasks.hpp"


int main(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner");
	  ros::NodeHandle node;

	  PathPlanning pathplanning;
	  PathPlanningServer task_pathplanning(pathplanning);
	  PathPlanningFocusServer task_pathplanningfocus(pathplanning);

	  ros::spin();
	  return 0;

	//return cogniteam_pathplanning_test_map_inflation(argc, argv);

}





