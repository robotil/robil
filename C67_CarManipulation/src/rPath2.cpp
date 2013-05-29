/*
 * rPath2.cpp
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
#include "FK.h"
#include "IK.h"
#include "Path.h"
#include "Transmit.h"

int main(int argc, char** argv)
{
	int pointsNum;
	double seconds;

	ros::init(argc, argv, "pub_path2_command_rhand");

	ros::NodeHandle* rosnode = new ros::NodeHandle();
	Transmit transmit = Transmit();

	// ros topic subscriptions
	ros::SubscribeOptions atlasStateSoatlasStateSo =
			ros::SubscribeOptions::create <atlas_msgs::AtlasState> (
			"/atlas/atlas_state", 100, Transmit::SetAtlasState,
			ros::VoidPtr(), rosnode->getCallbackQueue());

		atlasStateSo.transport_hints = ros::TransportHints().reliable().tcpNoDelay(true);
		  ros::Subscriber subAtlasState = rosnode->subscribe(atlasStateSo);

		// ros topic publisher
		pubAtlasCommand = rosnode->advertise<atlas_msgs::AtlasCommand>(
			"/atlas/atlas_command", 100, true);

	if (argc == 9)
	{
		double dubGet[6];
		for (int i=0; i<6; i++)
		{
			dubGet[i] = boost::lexical_cast<double>(argv[i+1]);
		}
		transmit.argTarget = RPY(dubGet[0],dubGet[1], dubGet[2], dubGet[3], dubGet[4], dubGet[5]);
		seconds =  boost::lexical_cast<double>(argv[7]);
		pointsNum = boost::lexical_cast<int>(argv[8]);
		transmit.use_arg = true;
	}

	while (ros::ok())
	{
		ros::spinOnce();

		if (transmit.callBackRun)
		{
			// check if no arguments
			if (!use_arg) break;



			IkSolution IkCurrent = IkSolution(transmit.as.position[q4r], transmit.as.position[q5r], transmit.as.position[q6r], transmit.as.position[q7r],
					transmit.as.position[q8r], transmit.as.position[q9r]);
			IkSolution IkNext = rSearchSolution(transmit.as.position[q1], transmit.as.position[q2], transmit.as.position[q3], transmit.argTarget);

			// check if solution valid
			if (IkNext.valid)
			{
				// print solution
				std::cout << "Solution/Command:\n";
				IkNext.Print();
				RPY r = rPose(transmit.as.position[q1], transmit.as.position[q2], transmit.as.position[q3], IkNext);
				r.Print();
				std::cout << "error: " << IkNext.error << std::endl;
			}
			else
			{
				std::cout << "No Solution.\n";
				break;
			}

			pPathPoints points = pPathPoints(IkCurrent, IkNext, pointsNum);

			for (int i=0; i<pointsNum; i++)
			{
				// ros::spinOnce();


				transmit.ac.position[q4r] = points.pArray[i]._q4;
				transmit.ac.position[q5r] = points.pArray[i]._q5;
				transmit.ac.position[q6r] = points.pArray[i]._q6;
				transmit.ac.position[q7r] = points.pArray[i]._q7;
				transmit.ac.position[q8r] = points.pArray[i]._q8;
				transmit.ac.position[q9r] = points.pArray[i]._q9;

				//ROS_INFO("");
				//std::cout << i <<": ";
				//points.Array[i].Print();

				pubAtlasCommand.publish(transmit.ac);

				ros::Duration(seconds/pointsNum).sleep();
			}
		}
		ros::Duration(0.1).sleep();
		break;

	}
	return 0;

}


