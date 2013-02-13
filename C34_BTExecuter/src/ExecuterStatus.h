/*
 * ExecuterStatus.h
 *
 *  Created on: Nov 28, 2012
 *      Author: dan
 */

#ifndef EXECUTERSTATUS_H_
#define EXECUTERSTATUS_H_

#include "Info.h"
#include "boost/shared_ptr.hpp"
#include "boost/thread.hpp"
#include "Logger.h"

class ExecuterStatus {
public:
	typedef boost::shared_ptr<ExecuterStatus> Ref;
	enum STATE{
		ST_RUNNING,
		ST_PAUSED,
		ST_FINISHED,
		ST_STOPPED,
		ST_ERROR
	};

private:
	std::string bt_source;
	std::string lookup_source;
	std::string address_source;
	STATE state;
	boost::mutex mtx;

	std::string stateStr();

public:
	ExecuterStatus();
	virtual ~ExecuterStatus();

	void setBTSource(std::string fname);
	void setLookupSource(std::string fname);
	void setAddressSource(std::string fname);
	void updateState(STATE st);
	STATE getState();
	virtual std::string str();

};

#endif /* EXECUTERSTATUS_H_ */
