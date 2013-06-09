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
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Path.h>
#include <C67_CarManipulation/Trace.h>

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
	int pointsNum = 0, totalPoints;
	int viasNum = 1;
	double radius = 0, seconds, dt;
	if (argc == 11)
	{
		double dubGet[8];
		for (int i=0; i<8; i++)
		{
			dubGet[i] = boost::lexical_cast<double>(argv[i+1]);
		}
		argTarget = RPY(dubGet[0],dubGet[1], dubGet[2], dubGet[3], dubGet[4], dubGet[5]);
		radius = dubGet[6];
		// first second will be for first point
		seconds =  dubGet[7]-1;
		viasNum = boost::lexical_cast<int>(argv[9]);
		totalPoints = boost::lexical_cast<int>(argv[10]);
		pointsNum = totalPoints/viasNum;
		dt = seconds/totalPoints;
		use_arg = true;
	}
   
	ros::init(argc, argv, "pub_trace_command_rhand");

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
			IkSolution IkNext;
			pPathPoints points = pPathPoints(pointsNum);
			//std::cout<<"print 1\n";
			Trace wheelTrace  = Trace(argTarget, radius, 0, M_PI/2, viasNum);
			// check if no arguments			
			if (!use_arg) break;

			for (unsigned int i = 0; i < n; i++)
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

			// assign current joint angles
			for (unsigned int j=0; j<numJoints; j++)
			{
				ac.position[j] = as.position[j];
				//std::cout << state[j] << " ";
			}

			for (int i = 0; i < viasNum; i++)
			{
				//std::cout<<"print 2,"<<i<<"\n";
				IkNext = rSearchSolution(as.position[q1], as.position[q2], as.position[q3], wheelTrace.pArray[i]);
				std::cout << "error:" << IkNext.error << "\n";

				if (!IkNext.valid) break;
				if (i==0)
					points.Update(IkSolution(as.position[q4r], as.position[q5r], as.position[q6r], as.position[q7r],
							as.position[q8r], as.position[q9r])
							, IkNext, NoEnd);
				else if (i==(viasNum-1))
					points.Update(IkSolution(ac.position[q4r], ac.position[q5r], ac.position[q6r], ac.position[q7r],
						ac.position[q8r], ac.position[q9r])
						, IkNext, NoStart);
				else
					points.Update(IkSolution(ac.position[q4r], ac.position[q5r], ac.position[q6r], ac.position[q7r],
						ac.position[q8r], ac.position[q9r])
						, IkNext, NoStartEnd);

				//std::cout<<"print 3,"<<i<<"\n";

				for (int j=0; j<pointsNum; j++)
				{
					// ros::spinOnce();


					ac.position[q4r] = points.pArray[j]._q4;
					ac.position[q5r] = points.pArray[j]._q5;
					ac.position[q6r] = points.pArray[j]._q6;
					ac.position[q7r] = points.pArray[j]._q7;
					ac.position[q8r] = points.pArray[j]._q8;
					ac.position[q9r] = points.pArray[j]._q9;

					//ROS_INFO("");
					//std::cout << i <<": ";
					//points.Array[i].Print();
					//std::cout<<"print 3,"<<i<<","<<j<<"\n";

					pubAtlasCommand.publish(ac);
					if (i)
						ros::Duration(dt).sleep();
					else
						ros::Duration(1.0/pointsNum).sleep();
				}
				//std::cout<<"print 4,"<<i<<"\n";
			} //end for
			std::cout<<"print 5\n";
			
			// check if solution valid
//			if (IkNext.valid)
//			{
//				// print solution
//				std::cout << "Solution/Command:\n";
//				IkNext.Print();
//				RPY r = rPose(as.position[q1], as.position[q2], as.position[q3], IkNext);
//				r.Print();
//				std::cout << "error: " << IkNext.error << std::endl;
//			}
//			else
//			{
//				std::cout << "No Solution.\n";
//				break;
//			}
			
			
			

			break;
		} //end if
		ros::Duration(0.1).sleep();
		//std::cout<<"print 6\n";
	}// end while
	//std::cout<<"print 7\n";
  return 0;
}
