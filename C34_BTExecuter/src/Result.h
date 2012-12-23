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
private:
	Result::Ref _next;
	bool _val;
	Info _info;
public:
	bool value(){ return _val; }
	Info info(){ return _info; }
	bool hasNext(){ return _next.get()!=NULL; }
	Result::Ref next(){ return _next; }
	static Result::Ref New(bool v, Info i){ return Ref(new Result(v, i)); }
	static Result::Ref New(bool v, Info i, Result::Ref r){ return Ref(new Result(v, i, r)); }

	void print(Logger& cout);
	void print(std::ostream& cout);
private:
	Result(bool v, Info i):_val(v), _info(i){ }
	Result(bool v, Info i, Result::Ref r):_next(r),_val(v), _info(i){ }
};

#endif /* RESULT_H_ */
