/*
 * BT.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "BT.h"


	std::vector<BT> BT::getSubtree()const{
		std::vector<BT> res;
		BOOST_FOREACH(const ptree::value_type& v, pt){
			if(v.first[0]=='<')continue;
			res.push_back(BT(v.first, v.second));
		}
		return res;
	}

	std::string BT::getRootName()const{
		try{
			return pt.get<std::string>("<xmlattr>.name");
		}catch(boost::property_tree::ptree_bad_path& e){}
		return "";
	}
	std::string BT::getID()const{
		try{
			return pt.get<std::string>("<xmlattr>.id");
		}catch(boost::property_tree::ptree_bad_path& e){}
		return "";
	}
	bool BT::hasID()const{
		try{
			return pt.get<std::string>("<xmlattr>.id")!="";
		}catch(boost::property_tree::ptree_bad_path& e){}
		return false;
	}
	int BT::getDBGTimeInterval()const{
		try{
			return pt.get<int>("<xmlattr>.dbg_time");
		}catch(boost::property_tree::ptree_bad_path& e){}
		return 0;
	}
	bool BT::getDBGResult()const{
		try{
			std::string v = pt.get<std::string>("<xmlattr>.dbg_result");
			std::transform(v.begin(), v.end(), v.begin(), ::tolower);
			return v=="true";
		}catch(boost::property_tree::ptree_bad_path& e){}
		return true;
	}
	std::string BT::getRootType()const{
		return type;
	}

	void BT::print(Logger& std_cout, std::string tab)const{
		std::string rtype = getRootType();
		std::string rname = getRootName();
		std_cout<<tab<<rtype<<":"<<rname;
		std::vector<BT> st = getSubtree();
		if(st.size()>0){
			std_cout<<"{"<<'\n';
			BOOST_FOREACH(const BT& b, st){
				b.print(std_cout, tab+"   ");
			}
			std_cout<<tab<<"}"<<'\n';
		} else {
			std_cout<<'\n';
		}
	}


