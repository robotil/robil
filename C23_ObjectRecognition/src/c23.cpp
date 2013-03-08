#include "C23_Node.hpp"

#include "LearnObjectServer.hpp"
#include "SearchObjectServer.hpp"
#include "TrackObjectServer.hpp"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c23_objectReconition");
 
  if(argc!=3){
	  printf("usage: C23_module <left camera topic> <right camera topic>");
  }
  
  C23_Node my_node(argv[1],argv[2]);

  TrackObjectServer trackObject(my_node);
  SearchObjectServer searchObject(my_node);
  LearnObjectServer learnObject(my_node);
 
   ROS_INFO("C23 made topic at %s %s \n",argv[1],argv[2]);
 
  ros::spin();

  return 0;
}
