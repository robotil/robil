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
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include "FK.h"
#include "IK.h"

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
int	 callback_called = 0;
RPY argTarget;
bool use_arg = false;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
	static ros::Time startTime = ros::Time::now();
  	static int count = 0;
	// for testing round trip time
	jointcommands.header.stamp = _js->header.stamp;

	// assign sinusoidal joint angle targets
	for (unsigned int i = 0; i < jointcommands.name.size(); i++)
		jointcommands.position[i] = _js->position[i];
	
	std_msgs::String msg;	
	std::stringstream ss;
	//ss << "hello world " << count++;
	//msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());
	
	callback_called += 1;
	// target from pelvis
	RPY Target;
	if (use_arg)
		Target = argTarget;
	else
		Target = RPY(0.6, -0.5, 0.3, 0.142, -0.061, 1.28);
	
	
	IkSolution Ik = SearchSolution(_js->position[q1], _js->position[q2], 
		_js->position[q3], Target);
	
	if (Ik.valid)
	{
		std::cout.precision(6);
		std::cout.setf (std::ios::fixed , std::ios::floatfield ); 
		std::cout << "q5-q9 old:"<< _js->position[q5]<< ", "<< _js->position[q6]<< ", "<< _js->position[q7]
			<< ", "<< _js->position[q8]<< ", "<< _js->position[q9]<< std::endl;
		Ik.Print();
	
		jointcommands.position[q4] = Ik._q4;
		jointcommands.position[q5] = Ik._q5;
		jointcommands.position[q6] = Ik._q6;
		jointcommands.position[q7] = Ik._q7;
		jointcommands.position[q8] = Ik._q8;
		jointcommands.position[q9] = Ik._q9;
		pub_joint_commands_.publish(jointcommands);
	}
	else
	{
		std::cout << "No solution found.\n";
	}
	
  
}

int main(int argc, char** argv)
{
	
  if (argc == 7)
  {
  	double dubGet[6];
  	for (int i=0; i<6; i++)
  	{
  		dubGet[i] = boost::lexical_cast<double>(argv[i+1]);
  	}
  	argTarget = RPY(dubGet[0],dubGet[1], dubGet[2], dubGet[3], dubGet[4], dubGet[5]);
  	argTarget.Print();
  	use_arg = true;
  }
   
  ros::init(argc, argv, "pub_joint_command_rhand");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  ros::Time last_ros_time_;
  bool wait = true;
  while (wait)
  {
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }

  // must match those inside AtlasPlugin
  jointcommands.name.push_back("atlas::back_lbz");
  jointcommands.name.push_back("atlas::back_mby");
  jointcommands.name.push_back("atlas::back_ubx");
  jointcommands.name.push_back("atlas::neck_ay");
  jointcommands.name.push_back("atlas::l_leg_uhz");
  jointcommands.name.push_back("atlas::l_leg_mhx");
  jointcommands.name.push_back("atlas::l_leg_lhy");
  jointcommands.name.push_back("atlas::l_leg_kny");
  jointcommands.name.push_back("atlas::l_leg_uay");
  jointcommands.name.push_back("atlas::l_leg_lax");
  jointcommands.name.push_back("atlas::r_leg_uhz");
  jointcommands.name.push_back("atlas::r_leg_mhx");
  jointcommands.name.push_back("atlas::r_leg_lhy");
  jointcommands.name.push_back("atlas::r_leg_kny");
  jointcommands.name.push_back("atlas::r_leg_uay");
  jointcommands.name.push_back("atlas::r_leg_lax");
  jointcommands.name.push_back("atlas::l_arm_usy");
  jointcommands.name.push_back("atlas::l_arm_shx");
  jointcommands.name.push_back("atlas::l_arm_ely");
  jointcommands.name.push_back("atlas::l_arm_elx");
  jointcommands.name.push_back("atlas::l_arm_uwy");
  jointcommands.name.push_back("atlas::l_arm_mwx");
  jointcommands.name.push_back("atlas::r_arm_usy");
  jointcommands.name.push_back("atlas::r_arm_shx");
  jointcommands.name.push_back("atlas::r_arm_ely");
  jointcommands.name.push_back("atlas::r_arm_elx");
  jointcommands.name.push_back("atlas::r_arm_uwy");
  jointcommands.name.push_back("atlas::r_arm_mwx");

  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointcommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointcommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointcommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_min[i]);
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_max[i]);

    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    jointcommands.kp_velocity[i]  = 0;
  }

  // ros topic subscribtions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

	while (ros::ok())
	{
		ros::spinOnce();
		if (callback_called == 1)
		{
			subJointStates.~Subscriber();
			callback_called++;
			break;
		}
		sleep(0.1);
	}

  return 0;
}
