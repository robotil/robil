/*
 * Info.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef INFO_H_
#define INFO_H_

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "BT.h"

class Info {
	std::string _node_type;
	std::string _node_name;
	std::string _error_description;
	BT bt;
public:
	Info(std::string node_type,std::string node_name,std::string error_description);
	Info(BT bt,std::string error_description);
	Info(std::string node_type,BT bt,std::string error_description);
	Info(const Info& i):_node_type(i._node_type),_node_name(i._node_name),_error_description(i._error_description),bt(i.bt){}
	virtual ~Info();

	void printFullName(Logger& cout);
	void printShortDescription(Logger& cout, int len);
	void printFullName(std::ostream& cout);
	void printShortDescription(std::ostream& cout, int len);
	std::string node_name()const{return _node_name;}
	std::string node_type()const{return _node_type;}
	std::string error_desc()const{return _error_description;}
};

#endif /* INFO_H_ */
