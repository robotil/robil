#include "ros/ros.h"
#include "std_msgs/String.h"
#include "C11_Agent/C11.h"

#include <sstream>
#include <stdlib.h>



bool PathPlan(C11_Agent::C11::Request& req,
	                C11_Agent::C11::Response& res)
{
	                ROS_INFO("START CALCULATION OF LOCAL PATH PLANNER");

return true;

}


int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "C11_AG");



  ros::NodeHandle n;


   
   ros::Publisher stt_pub = n.advertise<C11_Agent::C34C11_STT>("c11_stt", 1000);

   ros::ServiceServer service = n.advertiseService("PathPlan", PathPlan);


  ros::Rate loop_rate(10);
 

  int count = 0;
  while (ros::ok())
  {


  C11_Agent::C34C11_STT  stt1;

 ROS_INFO("The status is  = %d",  1);
 

    stt_pub.publish(stt1);

    ros::spinOnce();

    loop_rate.sleep();

  }

return 0;



}
