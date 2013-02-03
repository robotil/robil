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
#include "C34_Executer/resume.h"
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

   	ros::ServiceClient c34RunClient = _node.serviceClient<C34_Executer::run>("executer/run");

   	C34_Executer::run srv34Run;

   	std::string ostr;
   	std::stringstream out;
   	out <<"id"<< req.MSN.MSN << std::endl;
   	ostr= out.str();
   	ROS_INFO(ostr.data());
   	srv34Run.request.tree_id = ostr;
   	//srv34.request.tree_id << req.MSN.MSN << std::endl;

   	//ostr << "skill3.xml";
   	std::string filename;
   	filename.assign("git/robil/C34_Designer/plans/skill3.xml");

   	srv34Run.request.filename = filename;
//   	srv34.request.req.filename << "skill3.xml";

  	if (!c34RunClient.call(srv34Run))
  	{
  		ROS_ERROR("send of mission error, exiting\n");
  		return false;
  	}

   ROS_INFO("send of mission success");

   std::string str = srv34Run.response.output;
   ROS_INFO(str.data());

   ros::ServiceClient c34ResumeClient = _node.serviceClient<C34_Executer::resume>("executer/resume");
   C34_Executer::resume srv34Resume;
   srv34Resume.request.tree_id = ostr;
   if (!c34ResumeClient.call(srv34Resume))
	{
		ROS_ERROR("resume of mission error, exiting\n");
		return false;
	}
   ROS_INFO("resume of mission success");

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
