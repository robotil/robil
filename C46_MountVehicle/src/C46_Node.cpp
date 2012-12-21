#include "ros/ros.h"
#include "C46_MountVehicle/C46.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "C46_Node");
  ros::NodeHandle nh_;
  ROS_INFO("made topic at %s %s \n",argv[1],argv[2]);
  while(ros::ok()){
	  ros::spin();
  }
  return 0;
}

