
/*
 * ComponentMain.cpp
 *
 *  Created on: Thursday, 27. February 2014 12:29PM
 *      Author: autogenerated
 */
#include "ComponentMain.h"
#include "../roscomm/RosComm.h"
#include "WsmTask.h"
#include <std_msgs/Float64.h>


ComponentMain::ComponentMain(int argc,char** argv)
{
	_roscomm = new RosComm(this,argc, argv);
	//cur_mission = new WsmTask(this);
	this->cur_mission = NULL ;
	this->jointStates = NULL;
	this->receivedBladePosition = NULL;
	this->receivedWorkSeqData = NULL;
	this->receivedLocation = NULL;
	this->receivedPerVelocity = NULL;
    this->recivedMap = NULL;
    this->ground_heigth = 0 ;
    this->z_offset = 0;
}
ComponentMain::~ComponentMain() {
	if(_roscomm) delete _roscomm; _roscomm=0;
	if(cur_mission) delete cur_mission;
}

void ComponentMain::handleWorkSeqData(const config::WSM::sub::WorkSeqData& msg)
{
	if(this->cur_mission != NULL)
	{
		ROS_INFO("get_new_problem");
		if((this->cur_mission->Get_Task_id()) == atoi(msg.task_id.c_str()))
				{
					ROS_ERROR("Rejected Task %s, WSM already has Task %d , at state '%s'",msg.task_id.c_str(),this->cur_mission->Get_Task_id(),this->cur_mission->Get_status().c_str());
					return;
				}
		else
		{
			/*
			if(this->cur_mission->Get_status() == "paused"){
				ROS_ERROR("Rejected Task %s, WSM already has Task %d , at state %s",msg.task_id.c_str(),this->cur_mission->Get_Task_id(),this->cur_mission->Get_status().c_str());
				return;
			}
			*/
			ROS_INFO("Delete old , allocate mew!");
				delete this->cur_mission ;
				this->cur_mission = new WsmTask(atoi(msg.task_id.c_str()),1,msg,this);
				return;
		}
	}
	else
	{
		ROS_INFO("allocate mew!");
		this->cur_mission = new WsmTask(atoi(msg.task_id.c_str()),0,msg,this);
		return;
	}
}
	
void ComponentMain::handleBladePosition(const config::WSM::sub::BladePosition& msg)
{
	if(this->receivedBladePosition != NULL)
		delete this->receivedBladePosition;
	this->receivedBladePosition = new config::WSM::sub::BladePosition(msg);
	//std::cout<< "WSM say:" << msg << std::endl;
}

void ComponentMain::handleLocation(const config::LLC::sub::Location& msg)
{

	if(this->receivedLocation != NULL)
		delete this->receivedLocation;
	this->receivedLocation = new config::LLC::sub::Location(msg);
	//std::cout<< "LLC say:" << msg.pose.pose.position.x << std::endl;
}

void ComponentMain::handlePerVelocity(const config::LLC::sub::PerVelocity& msg)
{

	if(this->receivedPerVelocity != NULL)
		delete this->receivedPerVelocity;
	this->receivedPerVelocity = new config::LLC::sub::PerVelocity(msg);

//	std::cout<< "LLC say:" << msg << std::endl;
}

void ComponentMain::handleMiniMapWSM(const config::WSM::sub::MiniMap& msg)
{

	if(this->recivedMap != NULL){
		delete this->recivedMap ;
	}
		this->recivedMap = new config::WSM::sub::MiniMap(msg);

			//double max = 0 ;
	//	for(int i = 12 ; i < 18 ; i++)
	//	{
		//	std::cout << "[" << this->recivedMap->data[45*30 + i].height << "]" ;
	//		if(this->recivedMap->data[45*30 + i].height > max)
	//		{
		//		max = this->recivedMap->data[3*30 + i].height;
	///		}
	//	}
//	std::cout << std::endl ;
//	this->ground_heigth = max;

}

void ComponentMain::publish_monitor_time(const std_msgs::Header& msg)
{
	_roscomm->publish_monitor_time(msg);
}

void ComponentMain::publish_h(const std_msgs::Float64 &msg)
{
	_roscomm->publish_h(msg);
}

void ComponentMain::publish_m(const std_msgs::Float64 &msg)
{
	_roscomm->publish_m(msg);
}

void ComponentMain::publishWSMVelocity(config::WSM::pub::WSMVelocity& msg)
{
	_roscomm->publishWSMVelocity(msg);
}
	
void ComponentMain::publishBladePositionCommand(config::WSM::pub::BladePositionCommand& msg)
{
	_roscomm->publishBladePositionCommand(msg);
}
	
void ComponentMain::publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame){
	_roscomm->publishTransform(_tf, srcFrame, distFrame);
}
tf::StampedTransform ComponentMain::getLastTrasform(std::string srcFrame, std::string distFrame){
	return _roscomm->getLastTrasform(srcFrame, distFrame);
}
void ComponentMain::publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report){
	_roscomm->publishDiagnostic(_report);
}
void ComponentMain::publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report){
	_roscomm->publishDiagnostic(header, _report);
}
