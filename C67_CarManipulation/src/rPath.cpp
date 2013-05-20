/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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
#include "FK.h"
#include "IK.h"
#include "Path.h"

ros::Publisher pubAtlasCommand;
atlas_msgs::AtlasCommand ac;
atlas_msgs::AtlasState as;
boost::mutex mutex;
ros::Time t0;
const unsigned int numJoints = 28;

int	 callback_called = 0;
RPY argTarget;
bool use_arg = false;
bool callBackRun = false;

void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_as)
{
	static ros::Time startTime = ros::Time::now();
	t0 = startTime;

	// lock to copy incoming AtlasState
	{
		boost::mutex::scoped_lock lock(mutex);
		as = *_as;
	}
	
	ac.header.stamp = as.header.stamp;

	if (callBackRun == false)
	{
		callBackRun = true;
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

int main(int argc, char** argv)
{
	int pointsNum;
	double seconds; 
	if (argc == 9)
	{
		double dubGet[6];
		for (int i=0; i<6; i++)
		{
			dubGet[i] = boost::lexical_cast<double>(argv[i+1]);
		}
		argTarget = RPY(dubGet[0],dubGet[1], dubGet[2], dubGet[3], dubGet[4], dubGet[5]);
		seconds =  boost::lexical_cast<double>(argv[7]);
		pointsNum = boost::lexical_cast<int>(argv[8]);
		use_arg = true;
	}
   
	ros::init(argc, argv, "pub_path_command_rhand");

	ros::NodeHandle* rosnode = new ros::NodeHandle();

	ros::Time last_ros_time_;
	bool wait = true;
	while (wait)
	{
		last_ros_time_ = ros::Time::now();
		if (last_ros_time_.toSec() > 0)
		wait = false;
	}

	unsigned int n = numJoints;
	ac.position.resize(n);
	ac.k_effort.resize(n);
	ac.velocity.resize(n);
	ac.effort.resize(n);
	ac.kp_position.resize(n);
	ac.ki_position.resize(n);
	ac.kd_position.resize(n);
	ac.kp_velocity.resize(n);
	ac.i_effort_min.resize(n);
	ac.i_effort_max.resize(n);



	// ros topic subscribtions
	ros::SubscribeOptions atlasStateSo =
		ros::SubscribeOptions::create <atlas_msgs::AtlasState> (
		"/atlas/atlas_state", 100, SetAtlasState,
		ros::VoidPtr(), rosnode->getCallbackQueue());

	atlasStateSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
	  ros::Subscriber subAtlasState = rosnode->subscribe(atlasStateSo);

	// ros topic publisher
	pubAtlasCommand = rosnode->advertise<atlas_msgs::AtlasCommand>(
		"/atlas/atlas_command", 100, true);
	
	
	while (ros::ok())
	{
		ros::spinOnce();
		/*if (callback_called == 1)
		{
			subJointStates.~Subscriber();
			callback_called++;
			break;
		}*/
		if (callBackRun)
		{
			// check if no arguments			
			if (!use_arg) break;			
						
			IkSolution IkCurrent = IkSolution(as.position[q4r], as.position[q5r], as.position[q6r], as.position[q7r],
					as.position[q8r], as.position[q9r]);
			IkSolution IkNext = rSearchSolution(as.position[q1], as.position[q2], as.position[q3], argTarget);
			
			// check if solution valid
			if (IkNext.valid)
			{
				// print solution		
				std::cout << "Solution/Command:\n";			
				IkNext.Print();		
				RPY r = rPose(as.position[q1], as.position[q2], as.position[q3], IkNext);
				r.Print();	
				std::cout << "error: " << IkNext.error << std::endl;
			}
			else
			{
				std::cout << "No Solution.\n";
				break;
			}
			
			pPathPoints points = pPathPoints(IkCurrent, IkNext, pointsNum);
			
			ac.k_effort[q4l]  = 255;
			ac.k_effort[q5l]  = 255;
			ac.k_effort[q6l]  = 255;
			ac.k_effort[q7l]  = 255;
			ac.k_effort[q8l]  = 255;
			ac.k_effort[q9l]  = 255;
			ac.k_effort[q4r]  = 255;
			ac.k_effort[q5r]  = 255;
			ac.k_effort[q6r]  = 255;
			ac.k_effort[q7r]  = 255;
			ac.k_effort[q8r]  = 255;
			ac.k_effort[q9r]  = 255;
			
			for (int i=0; i<N; i++)
			{
				// ros::spinOnce();
				// assign current joint angles 
				for (unsigned int j=0; j<numJoints; j++)
				{
					ac.position[j] = as.position[j];
					//std::cout << state[j] << " ";
				}
				
				ac.position[q4r] = points.pArray[i]._q4;
				ac.position[q5r] = points.pArray[i]._q5;
				ac.position[q6r] = points.pArray[i]._q6;
				ac.position[q7r] = points.pArray[i]._q7;
				ac.position[q8r] = points.pArray[i]._q8;
				ac.position[q9r] = points.pArray[i]._q9;
				
				//ROS_INFO("");
				//std::cout << i <<": ";				
				//points.Array[i].Print();
				
				pubAtlasCommand.publish(ac);
					
				ros::Duration(seconds/pointsNum).sleep();
			}
			break;
		}
		ros::Duration(0.1).sleep();
	}

  return 0;
}
