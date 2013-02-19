#include "C23_Node.hpp"
#include "TrackObjectServer.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c23_objectReconition");
 
  if(argc!=3){
	  printf("usage: C23_module <left camera topic> <right camera topic>");
  }
  
  C23_Node my_node(argv[1],argv[2]);
 // my_node.detectAndTrack(CAR_DRIVER);
  TrackObjectServer trackObject(my_node);
  //SearchObjectServer(my_node);
 
   ROS_INFO("C23 made topic at %s %s \n",argv[1],argv[2]);
 
  ros::spin();

  return 0;
}
