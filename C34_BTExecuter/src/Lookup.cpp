/*
 * Lookup.cpp
 *
 *  Created on: Aug 22, 2012
 *      Author: dan
 */

#include "Lookup.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
typedef boost::property_tree::ptree ptree;

Lookup::Planner* createPlanner(std::string planner, ptree& pt){
	if(planner=="file"){
		std::string file = pt.get<std::string>("<xmlattr>.file-name");
		return new Lookup::Planer_LoadFromFile(file);
	}
	return NULL;
}

Lookup::Item* createItem(std::string name, std::string type, std::string planner, ptree& pt){
	Lookup::Planner* p = createPlanner(planner, pt);
	if(!p) return NULL;
	Lookup::Item* i=NULL;
	if(type == "synch") i= new Lookup::ItemGlobalSynch(name,p);
	if(type == "asunch") i= new Lookup::ItemGlobalAsynch(name,p);
	if(type == "local") i= new Lookup::ItemLocal(name,p);
	if(!i){ delete p; }
	return i;
}

Lookup::Lookup() {

}

Lookup::Lookup(std::string fname){
	ptree pt;
	read_xml(fname, pt);
	_parseXml(pt);
}

Lookup::Lookup(std::istream& stream){
	ptree pt;
	read_xml(stream, pt);
	_parseXml(pt);
}

void Lookup::_parseXml(boost::property_tree::ptree& pt){
	pt = pt.get_child("lookup");
	BOOST_FOREACH(ptree::value_type& v, pt)if(v.first=="parent"){
		addParent(Lookup::Ref(new Lookup(v.second.get<std::string>("<xmlattr>.file"))));
	}
	BOOST_FOREACH(ptree::value_type& v, pt)if(v.first=="task"){
		std::string name = v.second.get<std::string>("<xmlattr>.name");
		std::string type = v.second.get<std::string>("<xmlattr>.type");
		std::string planner = v.second.get<std::string>("<xmlattr>.planner");
		Lookup::Item* item = createItem(name,type, planner ,v.second);
		if(item){
			//log<<"Lookup item : "<<name<<", "<<type<<", "<<planner;
			addPlanner(ItemRef(item));
		}
	}
}

void Lookup::addParent(Lookup::Ref parent){
	boost::mutex::scoped_lock l(mtx);
	parents.insert(parent);
}

void Lookup::addPlanner(Lookup::ItemRef item){
	planners[item->taskName]=item;
}

Lookup::~Lookup() {
}

void Lookup::_analize(const BT& bt, bool thisIsRoot){

	//----------- current node
	if(bt.getRootType()=="tsk"){
		std::string name = bt.getRootName();
		if(_contains(name)){
			ItemRef item = _getPlanner(name);
			item->create(thisIsRoot);
		}
	}
	//----------- childrens
	BOOST_FOREACH(const BT& i, bt.getSubtree()){
		_analize(i, false);
	}
}
BT Lookup::createNode(const BT& bt){
	boost::mutex::scoped_lock l(mtx);

	std::string name = bt.getRootName();
	if(_contains(name)==false) return BT();
	ItemRef item = _getPlanner(name);
	//log<<"LOOKUP::NODE_CREATED "<<name;
	std::string pid = bt.getID();
	std::string cid = item->bt.getID();
	std::string id = "";
	if(cid!="") id += cid + (pid!=""?",":"");
	if(pid!="") id += pid;
	item->bt.pt.put("<xmlattr>.id",id);
	//log<<"NEW ID IS ["<<id<<"] pid=["<<pid<<"], cid=["<<cid<<"]";
	return item->bt;
}


bool Lookup::_contains(std::string name){
	bool r = planners.find(name)!=planners.end();
	if(r) return true;
	for(std::set<Lookup::Ref>::iterator i=parents.begin(); i!=parents.end(); i++){
		r = contains(name);
		if(r) return true;
	}
	return false;
}

Lookup::ItemRef Lookup::_getPlanner(std::string name){
	bool r = planners.find(name)!=planners.end();
	if(r) return planners[name];
	for(std::set<Lookup::Ref>::iterator i=parents.begin(); i!=parents.end(); i++){
		r = (*i)->contains(name);
		if(r) return (*i)->getPlanner(name);
	}
	return Lookup::ItemRef();
}

//--------------- ITEMS ----------------------------

void Lookup::ItemGlobalSynch::create(bool active){
	if(status==ItemStatus_Created) return;
	bt = planner->plan();
	status = ItemStatus_Created;
}
/*
* void Lookup::ItemGlobalAsynch::create(bool active){
* 	if(active){
* 		boost::mutex::scoped_lock l(m);
* 		if(status==ItemStatus_notCreated)
* 			startCreate();
* 		while(status!=ItemStatus_InProgress)
* 			cond.wait(l);
* 	}else{
* 		boost::mutex::scoped_lock l(m);
* 		if(status==ItemStatus_notCreated)
* 			startCreate();
* 	}
* }
*/
void Lookup::ItemLocal::create(bool active){
	if(status==ItemStatus_Created) return;
	if(active){
		bt = planner->plan();
		status = ItemStatus_Created;
	}
}

//----------------- PLANNERS ---------------------

BT Lookup::Planer_LoadFromFile::plan(){
		//log<<"PLANNER: LOAD NEW PLAN FROM "<<fname;
		return BT(fname);
}
