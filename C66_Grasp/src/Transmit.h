/*
 * Transmit.h
 *
 *  Created on: May 23, 2013
 *      Author: michaeldavid
 */

#ifndef TRANSMIT_H_
#define TRANSMIT_H_
enum TransmitConst{ NumOfJoints = 28 };
class Transmit{
	bool callBackRun;
	int	 callback_called;

public:
	ros::Publisher pubAtlasCommand;
	//ros::SubscribeOptions atlasStateSo;
	atlas_msgs::AtlasCommand ac;
	atlas_msgs::AtlasState as;
	boost::mutex mutex;
	ros::Time t0;

	bool use_arg;
	RPY argTarget;


	Transmit();
	//~Trace();

};
void SetAtlasState(const atlas_msgs::AtlasState::ConstPtr &_as);

#endif /* TRANSMIT_H_ */
