
#include <ros/ros.h>
#include <C31_PathPlanner/PathPlan.h>

#include "cogniteam_pathplanning.h"
#include "ConvertorC22.hpp"

#include "Tasks.hpp"

bool PathPlan( C31_PathPlanner::PathPlanRequest& req, C31_PathPlanner::PathPlanResponse& res){
  ROS_INFO("START CALCULATION OF GLOBAL PATH PLANNER");

  return true;
}


int _main(int argc, char** argv){
  ros::init(argc, argv, "C31_GlobalPathPlanner");
  ros::NodeHandle node;
  ros::ServiceServer ss_PathPlan = node.advertiseService(ros::this_node::getName(),PathPlan);

  ros::spin();
  return 0;
}

int main(int argc, char** argv){
	  ros::init(argc, argv, "C31_GlobalPathPlanner");
	  ros::NodeHandle node;
	  ros::ServiceServer c31_PathPlan = node.advertiseService(ros::this_node::getName(),PathPlan);

	  PathPlanning pathplanning;
	  PathPlanningServer task_pathplanning(pathplanning);
	  PathPlanningFocusServer task_pathplanningfocus(pathplanning);

	  ros::spin();
	  return 0;

	//return cogniteam_pathplanning_test_map_inflation(argc, argv);

}





