/*
 * Transmit.cpp
 *
 *  Created on: May 23, 2013
 *      Author: michaeldavid
 */

#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <atlas_msgs/AtlasState.h>
#include <atlas_msgs/AtlasCommand.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Path.h>
#include <C67_CarManipulation/Transmit.h>

Transmit::Transmit()
{
	callback_called = 0;
	use_arg = false;
	callBackRun = false;


	ros::Time last_ros_time_;
	bool wait = true;
	while (wait)
	{
		last_ros_time_ = ros::Time::now();
		if (last_ros_time_.toSec() > 0)
		wait = false;
	}

	ac.position.resize(NumOfJoints);
	ac.k_effort.resize(NumOfJoints);
	ac.velocity.resize(NumOfJoints);
	ac.effort.resize(NumOfJoints);
	ac.kp_position.resize(NumOfJoints);
	ac.ki_position.resize(NumOfJoints);
	ac.kd_position.resize(NumOfJoints);
	ac.kp_velocity.resize(NumOfJoints);
	ac.i_effort_min.resize(NumOfJoints);
	ac.i_effort_max.resize(NumOfJoints);




}

void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_as)
{


	if (callBackRun == false)
	{
		callBackRun = true;

		static ros::Time startTime = ros::Time::now();
		t0 = startTime;

		// lock to copy incoming AtlasState
		{
			boost::mutex::scoped_lock lock(mutex);
			as = *_as;
		}

		ac.header.stamp = as.header.stamp;

		for (unsigned int i = 0; i < NumOfJoints; i++)
		{
			ac.kp_position[i] = as.kp_position[i];
			ac.ki_position[i] = as.ki_position[i];
			ac.kd_position[i] = as.kd_position[i];
			ac.i_effort_min[i] = as.i_effort_min[i];
			ac.i_effort_max[i] = as.i_effort_max[i];

			ac.velocity[i] = 0;
			ac.effort[i] = 0;
			ac.kp_velocity[i] = 0;

		}

		//for out car only
//		ac.k_effort[q4l]  = 255;
//		ac.k_effort[q5l]  = 255;
//		ac.k_effort[q6l]  = 255;
//		ac.k_effort[q7l]  = 255;
//		ac.k_effort[q8l]  = 255;
//		ac.k_effort[q9l]  = 255;
//		ac.k_effort[q4r]  = 255;
//		ac.k_effort[q5r]  = 255;
//		ac.k_effort[q6r]  = 255;
//		ac.k_effort[q7r]  = 255;
//		ac.k_effort[q8r]  = 255;
//		ac.k_effort[q9r]  = 255;

		// assign current joint angles
		for (unsigned int j=0; j<NumOfJoints; j++)
		{
			ac.position[j] = as.position[j];
			// for in car only
			ac.k_effort[j]  = 255;
			//std::cout << state[j] << " ";
		}

		// set cout presentation
		std::cout.precision(6);
		std::cout.setf (std::ios::fixed , std::ios::floatfield );
		// print current state
		std::cout << "Current Position:\n";
		IkSolution IkCurrent = IkSolution(as.position[q4r],as.position[q5r],
			as.position[q6r],	as.position[q7r], as.position[q8r], as.position[q9r]);
		IkCurrent.Print();
		RPY rCurrent = rPose(as.position[q1], as.position[q2],as.position[q3],IkCurrent);
		rCurrent.Print();

		// print target
		if (use_arg)
		{
			std::cout << "Target:\n";
			argTarget.Print();
		}
	}


  // uncomment to simulate state filtering
  // usleep(1000);

}


