/*
 * TraceTest.cpp
 *
 *  Created on: May 9, 2013
 *      Author: michaeldavid
 */
#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <string>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Path.h>
#include <C67_CarManipulation/Trace.h>
#include <C67_CarManipulation/C67_Path.h>


int main(int argc, char** argv)
{
	std::cout.precision(6);
	std::cout.setf (std::ios::fixed , std::ios::floatfield );
	ros::init(argc, argv, "C67_Path_client");;

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<C67_CarManipulation::C67_Path>("C67_Path");
	C67_CarManipulation::C67_Path srv;
	srv.request.hand = "right";
	srv.request.q1 = 0;
	srv.request.q2 = 0;
	srv.request.q3 = 0;
	srv.request.x = 0.012;
	srv.request.y = -1;
	srv.request.z = 0.434;
	srv.request.R = -1.5;
	srv.request.P = 0.3;
	srv.request.Y = -1.5;
	if (client.call(srv))
	{
		if (srv.response.valid)
		{
			ROS_INFO("q4-q9 = %f,%f,%f,%f,%f,%f", srv.response.q4,srv.response.q5,srv.response.q6,
					srv.response.q7,srv.response.q8,srv.response.q9);
		}
		else
		{
			ROS_INFO("No Valid Solution");
		}
	}
	else
	{
		ROS_ERROR("Failed to call service C67 Path");
		return 1;
	}

	return 0;


}



