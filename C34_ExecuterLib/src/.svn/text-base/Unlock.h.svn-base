/*
 * Unlock.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef UNLOCK_H_
#define UNLOCK_H_

#include <boost/thread/mutex.hpp>

class Unlock{
	boost::mutex::scoped_lock& plocker;
public:
	Unlock(boost::mutex::scoped_lock& plocker):plocker(plocker){plocker.unlock();}
	~Unlock(){plocker.lock();}
};
class Lock{
	boost::mutex::scoped_lock& plocker;
public:
	Lock(boost::mutex::scoped_lock& plocker):plocker(plocker){plocker.lock();}
	~Lock(){plocker.unlock();}
};

#endif /* UNLOCK_H_ */
