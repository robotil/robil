/*
 * MapReader.cpp
 *
 *  Created on: Oct 6, 2012
 *      Author: dan
 */

#include "MapReader.h"

#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

typedef boost::property_tree::ptree ptree;
using namespace std;

MapReader::MapReader(std::string fname, std::string root, std::string items, std::string key, std::string value) {
	ptree pt;
	read_xml(fname, pt);
	pt = pt.get_child(root);
	BOOST_FOREACH(const ptree::value_type& v, pt){
		if(v.first==items){
			string _key = v.second.get<std::string>(string("<xmlattr>.")+key);
			string _val = v.second.get<std::string>(string("<xmlattr>.")+value);
			map[_key]=_val;
		}
	}
}
MapReader::MapReader(std::istream& fname, std::string root, std::string items, std::string key, std::string value) {
	ptree pt;
	read_xml(fname, pt);
	pt = pt.get_child(root);
	BOOST_FOREACH(const ptree::value_type& v, pt){
		if(v.first==items){
			string _key = v.second.get<std::string>(string("<xmlattr>.")+key);
			string _val = v.second.get<std::string>(string("<xmlattr>.")+value);
			map[_key]=_val;
		}
	}
}

MapReader::~MapReader() {

}

