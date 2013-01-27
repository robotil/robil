#ifndef _C11_PUSH_SERVER_H_
#define _C11_PUSH_SERVER_H_

//tasks dependencies
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include "C21_VisionAndLidar/C21_Pan.h"
//#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include "C11_OperationControl/push_img.h"

using namespace std;
using namespace C0_RobilTask;

class PushHMIServer:public RobilTask{


public:
	PushHMIServer(string name = "/PushHMI"):
    	RobilTask(name)
    {

    }

    TaskResult task(const string& name, const string& uid, Arguments& args){

       	ros::ServiceClient c21Client = _node.serviceClient<C21_VisionAndLidar::C21_Pan>("C21/Panorama");

  	    C21_VisionAndLidar::C21_Pan srv21;

  	    srv21.request.req.cmd=C21_VisionAndLidar::C21_PANORAMA::TAKE_PICTURE;

  	    if (!c21Client.call(srv21))
  		{
  			ROS_ERROR("couldn't get a picture, exiting\n");
  			return TaskResult::FAULT();
  		}

  	    //C21_VisionAndLidar::C21_Pan srv;
  	    srv21.request.req.cmd=C21_VisionAndLidar::C21_PANORAMA::RETURN_PANORAMA;
  	    if (!c21Client.call(srv21))
  	    {
  		    ROS_ERROR("couldn't get a C21 reply, exiting\n");
	  		return TaskResult::FAULT();

  	    }

    	ros::ServiceClient c11Client = _node.serviceClient<C11_OperationControl::push_img>("C11/push_img");

    	C11_OperationControl::push_img srv11;

  	    srv11.request.req.cmd = srv21.response.res;

  	    if (!c11Client.call(srv11))
  	    {
  		    ROS_ERROR("couldn't get a C11 reply, exiting\n");
	  		return TaskResult::FAULT();

  	    }

  	     if (srv11.response.ack != 1) {
   		    ROS_ERROR("C11 ack is fault, exiting\n");
 	  		return TaskResult::FAULT();
  	     }

  	     return TaskResult::SUCCESS ();

    }


#endif //_C11_PUSH_SERVER_
