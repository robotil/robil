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

bool ScanPath(C67_CarManipulation::C67_Path::Request  &req,
		C67_CarManipulation::C67_Path::Response &res)
{
	IkSolution Ik;
	if (req.hand == "right")
		Ik = rScanRPY(req.q1,req.q2,req.q3,RPY(req.x,req.y,req.z,req.R,req.P,req.Y),0.01);
	else
		Ik = lScanRPY(req.q1,req.q2,req.q3,RPY(req.x,req.y,req.z,req.R,req.P,req.Y),0.01);

	res.valid = Ik.valid;

	if(Ik.valid)
	{
		res.q4 = Ik.m_q[0];
		res.q5 = Ik.m_q[1];
		res.q6 = Ik.m_q[2];
		res.q7 = Ik.m_q[3];
		res.q8 = Ik.m_q[4];
		res.q9 = Ik.m_q[5];
	}
	return true;
}
int main(int argc, char** argv)
{
	std::cout.precision(6);
	std::cout.setf (std::ios::fixed , std::ios::floatfield );

	ros::init(argc, argv, "C67_Path_server");

	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("C67_Path", ScanPath);
	ROS_INFO("Ready to give C67 Path.");
	ros::spin();

	return 0;

}



