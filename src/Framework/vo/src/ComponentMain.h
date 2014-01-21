/*
 * ComponentMain.h
 *
 *  Created on: Jan 16, 2014
 *      Author: yuval
 */

#ifndef COMPONENTMAIN_H_
#define COMPONENTMAIN_H_

#include <std_msgs/String.h>

class RosComm;

class ComponentMain {
	RosComm *_roscomm;
public:
	ComponentMain(int argc,char** argv);
	virtual ~ComponentMain();

        void handleCamera(std_msgs::String msg);
        void handleINS(std_msgs::String msg);
        void handleTF(std_msgs::String msg);

        void publishPosAttVel(std_msgs::String &msg);

};

#endif /* COMPONENTMAIN_H_ */