/*
 * Result.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef RESULT_H_
#define RESULT_H_

#include "Info.h"
#include "boost/shared_ptr.hpp"
#include "Logger.h"

class Result{
public:
	typedef boost::shared_ptr<Result> Ref;
	static const int SYSTEM_ERROR=1100;
	static const int SYSTEM_ERROR_TERMINATED=1000;
private:
	Result::Ref _next;
	bool _val;
	int _error_code;
	Info _info;
public:
	bool value(){ return _val; }
	int error_code(){ return _error_code; }
	Info info(){ return _info; }
	bool hasNext(){ return _next.get()!=NULL; }
	Result::Ref next(){ return _next; }
	static Result::Ref New(bool v, int error_code, Info i){ return Ref(new Result(v, error_code, i)); }
	static Result::Ref New(bool v, int error_code, Info i, Result::Ref r){ return Ref(new Result(v, error_code, i, r)); }

	void print(Logger& cout);
	void print(std::ostream& cout);

	void printTop(Logger& cout);
	void printTop(std::ostream& cout);

private:
	Result(bool v, int error_code, Info i):_val(v), _error_code(error_code), _info(i){ }
	Result(bool v, int error_code, Info i, Result::Ref r):_next(r),_val(v), _error_code(error_code), _info(i){ }
};

#endif /* RESULT_H_ */
