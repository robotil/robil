
#include <ros/ros.h>
#include <C31_PathPlanner/PathPlan.h>

bool PathPlan( C31_PathPlanner::PathPlanRequest& req, C31_PathPlanner::PathPlanResponse& res){
  ROS_INFO("START CALCULATION OF LOCAL PATH PLANNER");

  return true;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "c31_LocalPathPlanner");
  ros::NodeHandle node;
  ros::ServiceServer ss_PathPlan = node.advertiseService(ros::this_node::getName(),PathPlan);

  ros::spin();
  return 0;
}





