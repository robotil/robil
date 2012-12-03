/*
 * Lookup.h
 *
 *  Created on: Aug 22, 2012
 *      Author: dan
 */

#ifndef LOOKUP_H_
#define LOOKUP_H_

#include "BT.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/foreach.hpp>
#include <map>
#include <set>
#include <sstream>

class Lookup {
public:
	enum ItemStatus{
		ItemStatus_notCreated,
		ItemStatus_Created,
		ItemStatus_InProgress
	};
	class Planner{
	public:
		virtual BT plan()=0;
		virtual ~Planner(){}
	};
	class Item{
	public:
		std::string taskName;
		ItemStatus status;
		Planner* planner;
		BT bt;
		Item(std::string name, Planner* p):
			taskName(name),
			status(ItemStatus_notCreated),
			planner(p)
		{  }
		virtual ~Item(){
			if(planner) delete planner; planner=0;
		}
		virtual void create(bool active)=0;
	};
	typedef boost::shared_ptr<Item> ItemRef;


	//------------- ITEMS -------------------------

	class ItemGlobalSynch:public Item{
	public:
		ItemGlobalSynch(std::string name, Planner* p):Item(name,p){}
		void create(bool active);
	};
	typedef ItemGlobalSynch ItemGlobalAsynch;

//	class ItemGlobalAsynch:public Item{
//		boost::mutex m;
//		boost::condition_variable cont;
//	public:
//		ItemGlobalAsynch(std::string name, Planner* p):Item(name,p){}
//		void create(bool active);
//	};

	class ItemLocal:public Item{
	public:
		ItemLocal(std::string name, Planner* p):Item(name,p){}
		void create(bool active);
	};

	//------------ PLANNERS --------------------

	class Planer_LoadFromFile:public Planner{
		std::string fname;
	public:
		Planer_LoadFromFile(std::string filename){
			fname = filename;
		}
		BT plan();
	};


public:
	typedef boost::shared_ptr<Lookup> Ref;


private:

	typedef std::set<Lookup::Ref> parents_t;
	parents_t parents;
	typedef std::map<std::string, ItemRef> planners_t;
	planners_t planners;
	boost::mutex mtx;
public:

	Lookup();
	Lookup(std::string fname);
	Lookup(std::istream& stream);
	virtual ~Lookup();

	void addParent(Lookup::Ref parent);

	void analize(const BT& bt, bool thisIsRoot=true){
		boost::mutex::scoped_lock l(mtx);
		_analize(bt, thisIsRoot);
	}
	BT createNode(const BT& bt);

	void addPlanner(ItemRef item);

	bool contains(std::string name){
		boost::mutex::scoped_lock l(mtx);
		return _contains(name);
	}
	bool contains(BT bt){
		if(bt.getRootType()!="tsk") return false;
		return contains(bt.getRootName());
	}
	ItemRef getPlanner(std::string name){
		boost::mutex::scoped_lock l(mtx);
		return _getPlanner(name);
	}

	std::string str()const{
		std::stringstream s;
		s<<"{";
		for(parents_t::iterator i=parents.begin();i!=parents.end();i++){
			s<<" "<<(*i)->str();
		}
		s<<" }";
		s<<"";
		for(planners_t::const_iterator i=planners.begin();i!=planners.end();i++){
			s<<" "<<i->first<<":"<<i->second->taskName<<":"<<i->second->status;
		}
		s<<" ";
		return s.str();
	}

private:
	void _parseXml(boost::property_tree::ptree& pt);
	void _analize(const BT& bt, bool thisIsRoot=true);
	bool _contains(std::string name);
	ItemRef _getPlanner(std::string name);
};

#endif /* LOOKUP_H_ */
