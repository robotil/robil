#include "ros/ros.h"
#include "std_msgs/String.h"
#include "C11_Agent/C11.h"
#include "C11_Agent/obstacle_map.h"
#include "C11_Agent/object_map.h"
#include "C11_Agent/override_object_properties.h"
#include "C11_Agent/override_obstacle_properties.h"
#include "C10_Common/mission_selection.h"
#include "C11_Agent/ask_for_image.h"
#include "C11_Agent/set_forbidden_are.h"
#include "C11_Agent/override_local_path.h"
#include "C11_PushServer.hpp"
//#include "TaskProxyConnectionByActionLib.h"
#include "C34_Executer/run.h"
#include <sstream>
#include <stdlib.h>


bool PathPlan(C11_Agent::C11::Request& req,
	                C11_Agent::C11::Response& res)
{
	                ROS_INFO("START CALCULATION OF LOCAL PATH PLANNER");

return true;

}


bool MissionSelection(C10_Common::mission_selection::Request& req,
		C10_Common::mission_selection::Response& res)
{
	res.MES.mes = 1;
	int test = req.MSN.MSN;

	ros::NodeHandle _node;

   	ros::ServiceClient c34Client = _node.serviceClient<C34_Executer::run>("executer/run");

   	C34_Executer::run srv34;

   	std::string ostr;
   	std::stringstream out;
   	out << req.MSN.MSN << std::endl;
   	ostr= out.str();
   	ROS_INFO(ostr.data());
   	srv34.request.tree_id = ostr;
   	//srv34.request.tree_id << req.MSN.MSN << std::endl;

   	//ostr << "skill3.xml";
   	ostr.assign("C34_Designer//plans//skill3.xml");

   	srv34.request.filename = ostr;
//   	srv34.request.req.filename << "skill3.xml";

  	if (!c34Client.call(srv34))
  	{
  		ROS_ERROR("send of mission error, exiting\n");
  		return false;
  	}

   ROS_INFO("send of mission success");

   std::string str = srv34.response.output;
   ROS_INFO(str.data());

   return true;

}


bool AskForImage(C11_Agent::ask_for_image::Request& req,
		C11_Agent::ask_for_image::Response& res)
{

		ROS_INFO("START CALCULATION OF AskForImage");
		return true;

}



int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "C11_Agent");



  ros::NodeHandle n;


   
   ros::Publisher stt_pub = n.advertise<C11_Agent::C34C11_STT>("c11_stt", 1000);

   ros::ServiceServer service = n.advertiseService("PathPlan", PathPlan);
   ros::ServiceServer service_MissionSelection = n.advertiseService("MissionSelection", MissionSelection);
   ros::ServiceServer service_AskForImage = n.advertiseService("AskForImage", AskForImage);

   ros::Rate loop_rate(10);

   ROS_INFO("C11_Agent started!\n");

   PushHMIServer pushS;
 

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
