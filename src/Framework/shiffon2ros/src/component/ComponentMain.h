
/*
 * ComponentMain.h
 *
 *  Created on: Monday, 07. September 2015 03:23PM
 *      Author: autogenerated
 */
#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_
#include <std_msgs/String.h>
#include <ParameterTypes.h>
#include <tf/tf.h>
#include "../Shiphon_IO/Shiphon_IO.h"
#include "../roscomm/RosComm.h"

class RosComm;
class ComponentMain {
	RosComm* 		_roscomm;
	Shiphon_Ctrl * 	_shiphonCtrl;
public:
	ComponentMain(int argc,char** argv);
	virtual ~ComponentMain();

	void publishGPS(config::SHIFFON2ROS::pub::GPS& msg);
	void publishINS(config::SHIFFON2ROS::pub::INS& msg);
	void publishINS2(std_msgs::Float64& msg);
	void publishGpsSpeed(config::SHIFFON2ROS::pub::GpsSpeed& msg);
	void publishGpsSpeed2(std_msgs::Float64& msg);
	void publishTransform(const tf::Transform& _tf, std::string srcFrame, std::string distFrame);
	tf::StampedTransform getLastTrasform(std::string srcFrame, std::string distFrame);
	void publishDiagnostic(const diagnostic_msgs::DiagnosticStatus& _report);
	void publishDiagnostic(const std_msgs::Header& header, const diagnostic_msgs::DiagnosticStatus& _report);

	void InitShiphonConection();
	void ReadAndPub_ShiphonGPS();
	void ReadAndPub_ShiphonINS();
	void ReadAndPub_ShiphonGpsSpeed();

};
#endif /* COMPONENTMAIN_H_ */