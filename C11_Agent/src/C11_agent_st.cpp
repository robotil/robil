#include "ros/ros.h"
#include "std_msgs/String.h"
#include "C11_Agent/C11.h"
#include "C11_Agent/obstacle_map.h"
#include "C11_Agent/object_map.h"
#include "C11_Agent/override_object_properties.h"
#include "C11_Agent/override_obstacle_properties.h"
#include "C11_Agent/mission_selection.h"
#include "C11_Agent/ask_for_image.h"
#include "C11_Agent/set_forbidden_are.h"
#include "C11_Agent/override_local_path.h"
#include <sstream>
#include <stdlib.h>


bool PathPlan(C11_Agent::C11::Request& req,
	                C11_Agent::C11::Response& res)
{
	                ROS_INFO("START CALCULATION OF LOCAL PATH PLANNER");

return true;

}

bool ObstacleMap(C11_Agent::obstacle_map::Request& req,
		C11_Agent::obstacle_map::Response& res)
{

	                ROS_INFO("START CALCULATION OF Obstacle_map");
return true;

}

bool ObjectMap(C11_Agent::object_map::Request& req,
		C11_Agent::object_map::Response& res)
{

	                ROS_INFO("START CALCULATION OF object_map");
return true;

}

bool OverrideObjectProperties(C11_Agent::override_object_properties::Request& req,
		C11_Agent::override_object_properties::Response& res)
{

	                ROS_INFO("START CALCULATION OF override_object_properties");
return true;

}

bool OverrideObstacleProperties(C11_Agent::override_obstacle_properties::Request& req,
		C11_Agent::override_obstacle_properties::Response& res)
{

	                ROS_INFO("START CALCULATION OF override_obstacle_properties");
return true;

}


bool MissionSelection(C11_Agent::mission_selection::Request& req,
		C11_Agent::mission_selection::Response& res)
{

	                ROS_INFO("START CALCULATION OF MissionSelection");
return true;

}


bool AskForImage(C11_Agent::ask_for_image::Request& req,
		C11_Agent::ask_for_image::Response& res)
{

	                ROS_INFO("START CALCULATION OF AskForImage");
return true;

}

bool setForbiddenAre(C11_Agent::set_forbidden_are::Request& req,
		C11_Agent::set_forbidden_are::Response& res)
{

	                ROS_INFO("START CALCULATION OF setForbiddenAre");
return true;

}


bool overrideLocalPath(C11_Agent::override_local_path::Request& req,
		C11_Agent::override_local_path::Response& res)
{

	                ROS_INFO("START CALCULATION OF overrideLocalPath");
return true;

}

int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "C11_Agent");



  ros::NodeHandle n;


   
   ros::Publisher stt_pub = n.advertise<C11_Agent::C34C11_STT>("c11_stt", 1000);

   ros::ServiceServer service = n.advertiseService("PathPlan", PathPlan);

   ros::ServiceServer severride_obstacle_propertiesrvice_ObstacleMap = n.advertiseService("ObstacleMap", ObstacleMap);

   ros::ServiceServer service_ObjectMap = n.advertiseService("ObjectMap", ObjectMap);

   ros::ServiceServer service_OverrideObjectProperties = n.advertiseService("OverrideObjectProperties", OverrideObjectProperties);

   ros::ServiceServer service_OverrideObstacleProperties = n.advertiseService("OverrideObstacleProperties", OverrideObstacleProperties);

   ros::ServiceServer service_MissionSelection = n.advertiseService("MissionSelection", MissionSelection);

   ros::ServiceServer service_AskForImage = n.advertiseService("AskForImage", AskForImage);

   ros::ServiceServer service_setForbiddenAre = n.advertiseService("setForbiddenAre", setForbiddenAre);

   ros::ServiceServer service_overrideLocalPath = n.advertiseService("overrideLocalPath", overrideLocalPath);

   ros::Rate loop_rate(10);
 


  int count1 = 0;
  while (ros::ok())
  {


  C11_Agent::C34C11_STT  stt1;

// ROS_INFO("The status is  = %d",  1);
 

    stt_pub.publish(stt1);

    ros::spinOnce();

    loop_rate.sleep();

  }

return 0;



}
