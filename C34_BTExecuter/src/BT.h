/*
 * BT.h
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#ifndef BT_H_
#define BT_H_

#include <string>
#include <vector>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include "Logger.h"
#include "StringOperations.h"

class BT{

	typedef boost::property_tree::ptree ptree;
public:
	ptree pt;
	std::string type;
	std::string source;

	static BT load(std::string fname){
		return BT(fname);
	}
	BT(std::istream& stream){
		read_xml(stream, pt);
		pt = pt.get_child("plan");
		type = "plan";
		source = "---";
	}
	BT(std::string fname){
		if(
			string_operations::startWith(fname,"<plan") ||
			string_operations::startWith(fname,"<?xml")
		){
			std::cout<<"BT: run xml : "<<fname<<std::endl;
			std::stringstream xml; xml << fname;
			read_xml(xml, pt);
			std::cout<<"BT created"<<std::endl;
			source = fname;
		}else{
			std::cout<<"BT: load and run xml from file "<<fname<<std::endl;
			read_xml(fname, pt);
			std::cout<<"BT created"<<std::endl;
			source = fname;
		}
		pt = pt.get_child("plan");
		type = "plan";
	}
	BT(std::string type, ptree pt):pt(pt),type(type),source("---"){

	}
	BT(){

	}
	bool empty(){
		return pt.size()<1;
	}
	std::vector<BT> getSubtree()const;

	std::string getRootName()const;
	std::string getID()const;
	bool hasID()const;
	int getDBGTimeInterval()const;
	bool getDBGResult()const;
	std::string getRootType()const;

	void print(Logger& std_cout, std::string tab)const;
};

#endif /* BT_H_ */
