/*
 * Logger.h
 *
 *  Created on: Sep 19, 2012
 *      Author: dan
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <boost/thread/mutex.hpp>
#include <iostream>
#include <sstream>

class Logger : public std::ostream{
	static std::stringstream cout;
	static boost::mutex mtx;
	boost::mutex::scoped_lock locker;
	std::string source;
	bool first_print;

	std::stringstream _global_log;

public:
	typedef void (*LOG_CALLBACK)( const std::string& str );
protected:
	static LOG_CALLBACK global_log;
	#define PUSH_TO_GLOG(X) if(global_log){_global_log<<X;}
	#define FLASH_GLOG if(global_log){global_log(_global_log.str());_global_log.str("");}

public:
	Logger(std::string source):
		locker(mtx),
		source(source),
		first_print(true)
	{
	}
	virtual ~Logger(){
		cout<<std::endl;
		std::cout<<std::endl;
		PUSH_TO_GLOG(std::endl)
		FLASH_GLOG
	}
	template<class T>
	Logger& operator<<(const T& t){
		printHeader();
		cout<<t;
		std::cout<<t;
		PUSH_TO_GLOG(t)
		return *this;
	}
	void printHeader(){
		if(!first_print) return;
		first_print = false;
		std::stringstream t; t<<"["<<source<<"] ";
		cout<<t.str();
		std::cout<<t.str();
		PUSH_TO_GLOG(t.str())
	}
	std::string get(){
		std::string res = cout.str();
		cout.str("");
		std::cout<<"--- get ---";
		return res;
	}
	void setLogCallback(LOG_CALLBACK logcb){
		global_log = logcb;
	}
};
#define log Logger(_executer_id)
#define logSystem Logger("system")
#define logID(X) Logger(X)
#undef PUSH_TO_GLOG
#undef FLASH_GLOG


#endif /* LOGGER_H_ */
