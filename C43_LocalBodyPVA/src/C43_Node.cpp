#include "ros/ros.h"
#include "C43_LocalBodyPVA/C43.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "C43_Node");
  ros::NodeHandle nh_;
  ROS_INFO("made topic at %s %s \n",argv[1],argv[2]);
  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}

