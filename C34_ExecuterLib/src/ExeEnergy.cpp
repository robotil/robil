/*
 * ExeEnergy.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: dan
 */

#include "ExeEnergy.h"
#include "Logger.h"

ExeEnergy::ExeEnergy():cont_time(-1),step(false) {
}

ExeEnergy::~ExeEnergy() {
	boost::mutex::scoped_lock l(m);
	stepReady.notify_all();
}

void ExeEnergy::getStep(){
	boost::mutex::scoped_lock l(m);
	while(true){
		if(cont_time < 0){
			//log<<"ExeEnergy::getStep > step mode";
			if(step){ step=false; return; }
			//log<<"wait for step energy ...";
			stepReady.wait(l);
		}else{
			//log<<"ExeEnergy::getStep > continues mode";
			stepReady.timed_wait(l, timeout);
			timeout = boost::get_system_time()+ boost::posix_time::milliseconds(cont_time);
			return;
		}
	}
}
void ExeEnergy::setStep(){
	boost::mutex::scoped_lock l(m);
	cont_time = -1;
	step=true;
	stepReady.notify_one();
}
void ExeEnergy::setStepMode(){
	boost::mutex::scoped_lock l(m);
	cont_time = -1;
	step=false;
}
void ExeEnergy::wake(){
	boost::mutex::scoped_lock l(m);
	stepReady.notify_one();
}
void ExeEnergy::setContinuously(long milsec){
	boost::mutex::scoped_lock l(m);
	cont_time = milsec;
	step=false;
	timeout = boost::get_system_time()+ boost::posix_time::milliseconds(cont_time);
}
