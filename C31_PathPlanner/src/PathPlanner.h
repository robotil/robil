#ifndef ____PATH_PLANNER_IMPL_H_
#define ____PATH_PLANNER_IMPL_H_

//common utils
#include <sstream>
#include <iostream>
#include <time.h>

//local pathplanner dependencies
#include "Gps.h"
#include "cogniteam_pathplanning.h"
#include "ConvertorC22.hpp"
#include "ConvertorC23.hpp"

#include "Events.hpp"


using namespace std;
using namespace C0_RobilTask;


typedef GPSPoint TargetPosition;
typedef GPSPoint Localization;
typedef std::string TargetGoal;

struct Editable_Constraints{
	RobotDimentions& dimentions;
	Transits& transits;
	Attractors& attractors;
	Editable_Constraints(RobotDimentions& dimentions, Transits& transits, Attractors& attractors)
	:dimentions(dimentions),transits(transits),attractors(attractors){}
};

class PlanningArguments{
public:
	const Map& map;
	const Waypoint& start;
	const Waypoint& finish;
	const TargetPosition& targetPosition;
	const TargetGoal& targetGoal;
	const GPSPoint& selfLocation;
	const MapProperties& mapProperties;
	PlanningArguments(
		const Map& map, 
		const Waypoint& start, 
		const Waypoint& finish, 
		const TargetPosition& targetPos, 
		const TargetGoal& targetGoal, 
		const GPSPoint& sloc, 
		const MapProperties& mapProperties)
	:
		map(map), 
		start(start), 
		finish(finish), 
		targetPosition(targetPos), 
		targetGoal(targetGoal), 
		selfLocation(sloc), 
		mapProperties(mapProperties)
	{  }
};
class Editable_PlanningArguments{
public:
	Map& map;
	Waypoint& start;
	Waypoint& finish;
	TargetPosition& targetPosition;
	TargetGoal& targetGoal;
	GPSPoint& selfLocation;
	MapProperties& mapProperties;
	Editable_PlanningArguments(
		Map& map, 
		Waypoint& start, 
		Waypoint& finish, 
		TargetPosition& targetPos, 
		TargetGoal& targetGoal,
		GPSPoint& sloc, 
		MapProperties& mapProperties
	)
	:
		map(map), 
		start(start), 
		finish(finish), 
		targetPosition(targetPos), 
		targetGoal(targetGoal),
		selfLocation(sloc), 
		mapProperties(mapProperties)
	{   }
};

class PlanningResult{
public:
	const SmoothedPath& path;
	PlanningResult(const SmoothedPath& path)
	:path(path){}
};


class PathPlanning{
	boost::mutex _mtx;
#define SYNCHRONIZED boost::mutex::scoped_lock l(_mtx);
#define LOCK( X ) boost::shared_ptr<boost::mutex::scoped_lock> X(new boost::mutex::scoped_lock(_mtx));
#define UNLOCK( X ) X = boost::shared_ptr<boost::mutex::scoped_lock>();


	class PlanningInputData{
	public:
		Map map;
		Waypoint start;
		Waypoint finish;
		RobotDimentions dimentions;
		Transits transits;
		Attractors attractors;
		PlanningInputData():map(0,0){}
		PlanningInputData(const PlanningInputData& p):
			 map(p.map)
			,start(p.start)
			,finish(p.finish)
			,dimentions(p.dimentions)
			,transits(p.transits)
			,attractors(p.attractors)
		{}
	};

	class ChangesNotification{
	public:
		virtual ~ChangesNotification(){}
		virtual void notify()=0;
	};
	template <typename CALLBACK>
	class _ChangesNotification:public ChangesNotification{
	public:
		CALLBACK callback;
		_ChangesNotification(CALLBACK cb):callback(cb){}
		virtual void notify(){callback();}
	};

	//-------------- DATA ------------------------------
	TargetPosition targetPosition;
	TargetGoal targetGoal;
	GPSPoint selfLocation;
	MapProperties mapProperties;

	PlanningInputData data;
	SmoothedPath path;
	//--------------------------------------------------

	//-------------- INTERFACES ------------------------
	PlanningArguments arguments;
	Constraints constraints;
	PlanningResult results;

	Editable_PlanningArguments ed_arguments;
	Editable_Constraints ed_constraints;
	//--------------------------------------------------

	//-------------- EVENTS ----------------------------
	boost::shared_ptr<ChangesNotification> changeNotification;
	Events _events;
	//--------------------------------------------------

public:

	PathPlanning():
		//temporal data
		targetPosition(0,0), 
		targetGoal(""),
		selfLocation(0,0), 
		mapProperties(0.25, GPSPoint(0,0), Waypoint(0,0)),
		//input data 
		data(),
		//output data
		path(),
		//parameters interfaces
		//...read-only
		   arguments(data.map, data.start, data.finish, targetPosition, targetGoal, selfLocation, mapProperties),
		   constraints(data.dimentions, data.transits, data.attractors), results(path),
		//...read-write
		ed_arguments(data.map, data.start, data.finish, targetPosition, targetGoal, selfLocation, mapProperties),
		ed_constraints(data.dimentions, data.transits, data.attractors)
	{

	}

	template <typename CALLBACK>
	void setChangeNotifier(CALLBACK cb){ changeNotification = boost::shared_ptr<ChangesNotification>(new _ChangesNotification<CALLBACK>(cb)); }

	bool plan();
	Events& events(){ return _events; }

	class EditSession{
		boost::shared_ptr<boost::mutex::scoped_lock> l;
		boost::shared_ptr<ChangesNotification> changeNotification;
	public:
		bool _aborted;
		Editable_Constraints& constraints;
		Editable_PlanningArguments& arguments;
		EditSession(boost::mutex& _mtx, Editable_Constraints& cons, Editable_PlanningArguments& arguments, boost::shared_ptr<ChangesNotification> ch_notify):
			l(new boost::mutex::scoped_lock(_mtx)), changeNotification(ch_notify), _aborted(false), constraints(cons),arguments(arguments)  {}
		EditSession(const EditSession& e):
			l(e.l), changeNotification(e.changeNotification), _aborted(false), constraints(e.constraints),arguments(e.arguments)  {}
		~EditSession(){ 
			if(!isAborted()){
				notify(); 
			}
		}
		void aborted(){ _aborted = true; }
		bool isAborted()const{ return _aborted; }
		void notify(){if(changeNotification.get())changeNotification->notify();}
	};
	class ReadSession{
		boost::shared_ptr<boost::mutex::scoped_lock> l;
	public:
		const Constraints& constraints;
		const PlanningArguments& arguments;
		const PlanningResult& results;
		ReadSession(boost::mutex& _mtx, const Constraints& cons, const PlanningArguments& arguments, const PlanningResult& results):
			l(new boost::mutex::scoped_lock(_mtx)),constraints(cons),arguments(arguments), results(results) {}
		ReadSession(const ReadSession& e):
			l(e.l),constraints(e.constraints),arguments(e.arguments), results(e.results)  {}
	};

	EditSession startEdit(){
		return EditSession(_mtx, ed_constraints, ed_arguments, changeNotification);
	}
	ReadSession startReading(){
		return ReadSession(_mtx, constraints, arguments, results);
	}

	bool isMapReady()const{
		return data.map.w()>0 && data.map.h()>0;
	}

	long cast(double gps)const;
	long castLength(double gps)const;
	double cast(long cell)const;
	double castLength(long cell)const;
	double castWP(double cell)const;
	double castWPLength(double cell)const;
	Waypoint cast(const GPSPoint& gps)const;
	GPSPoint cast(const Waypoint& wp)const;
	GPSPoint cast(const Vec2d& wp)const;

#undef SYNCHRONIZED
#undef LOCK
#undef UNLOCK

};


#endif
