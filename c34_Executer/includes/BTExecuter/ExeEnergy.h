/*
 * ExeEnergy.h
 *
 *  Created on: Aug 21, 2012
 *      Author: dan
 */

#ifndef EXEENERGY_H_
#define EXEENERGY_H_

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/shared_ptr.hpp>
#include "Unlock.h"

class ExeEnergy {
public:
	typedef boost::shared_ptr<ExeEnergy> Ref;

private:

	boost::mutex m;
	boost::condition_variable stepReady;
	long cont_time;
	bool step;
	boost::system_time timeout;

public:
	ExeEnergy();
	virtual ~ExeEnergy();

	void getStep();
	void getStep(boost::mutex::scoped_lock& l){ Unlock ul(l); getStep(); }

	void setStep();
	void setStepMode();
	void wake();
	void setContinuously(long milsec);

	static ExeEnergy& global(){ static ExeEnergy e; return e; }
	bool stepByStep()const { return cont_time<0; }

};

#endif /* EXEENERGY_H_ */
