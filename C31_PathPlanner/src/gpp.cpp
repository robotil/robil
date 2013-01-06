
#include <ros/ros.h>
#include <C31_PathPlanner/PathPlan.h>

#include "cogniteam_pathplanning.h"
#include "ConvertorC22.hpp"

#include "Tasks.hpp"

bool PathPlan( C31_PathPlanner::PathPlanRequest& req, C31_PathPlanner::PathPlanResponse& res){
  ROS_INFO("START CALCULATION OF GLOBAL PATH PLANNER");

  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "C31_GlobalPathPlanner");
  ros::NodeHandle node;
  ros::ServiceServer ss_PathPlan = node.advertiseService(ros::this_node::getName(),PathPlan);

  ros::spin();
  return 0;
}

int _main(int argc, char** argv){

	return cogniteam_pathplanning_test_map_inflation(argc, argv);

}





