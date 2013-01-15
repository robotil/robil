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
	const Path& path;
	PlanningResult(const Path& path)
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

	TargetPosition targetPosition;
	TargetGoal targetGoal;
	GPSPoint selfLocation;
	MapProperties mapProperties;

	PlanningInputData data;
	Path path;

	PlanningArguments arguments;
	Constraints constraints;
	PlanningResult results;

	Editable_PlanningArguments ed_arguments;
	Editable_Constraints ed_constraints;

	boost::shared_ptr<ChangesNotification> changeNotification;

public:

	PathPlanning():
		//temporal data
		targetPosition(0,0), 
		targetGoal(""),
		selfLocation(0,0), 
		mapProperties(0.12, GPSPoint(0,0), Waypoint(0,0)),
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

	void plan();

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

// 	#define TRANSLATE_GPS_TO_GRID(x, v) ( mapProperties.anchor.x + round( ( v - mapProperties.gps.x) / mapProperties.resolution ) )
// 	#define TRANSLATE_GRID_TO_GPS(x, v) ( mapProperties.gps.x +  (v - mapProperties.anchor.x) * mapProperties.resolution  )
// 	#define TRANSLATE_POINT_GPS_TO_GRID(x) TRANSLATE_GPS_TO_GRID(x, gps.x)
// 	#define TRANSLATE_POINT_GRID_TO_GPS(x) TRANSLATE_GRID_TO_GPS(x, wp.x)

	size_t cast(double gps)const;
// 	{
// 		if(isMapReady()==false){
// 			return 0;
// 		}
// 		size_t cell = TRANSLATE_GPS_TO_GRID(x, gps);
// 		return cell;
// 	}
	size_t castLength(double gps)const;
// 	{
// 		if(isMapReady()==false){
// 			return 0;
// 		}
// 		size_t cell = TRANSLATE_GPS_TO_GRID(x, gps+mapProperties.gps.x) - mapProperties.anchor.x;
// 		return cell;
// 	}
	double cast(size_t cell)const;
// 	{
// 		if(isMapReady()==false){
// 			return 0.0;
// 		}
// 		size_t gps = TRANSLATE_GRID_TO_GPS(x, cell);
// 		return gps;
// 	}
	double castLength(size_t cell)const;
// 	{
// 		if(isMapReady()==false){
// 			return 0.0;
// 		}
// 		double gps = TRANSLATE_GRID_TO_GPS(x+mapProperties.anchor.x, cell) - mapProperties.gps.x;
// 		return gps;
// 	}
	Waypoint cast(const GPSPoint& gps)const;
// 	{
// 		#define TRANSLATE(x) TRANSLATE_POINT_GPS_TO_GRID(x)
// 
// 		if(isMapReady()==false){
// 			return Waypoint(0,0);
// 		}
// 
// 		long x ( TRANSLATE(x) );
// 		long y ( TRANSLATE(y) );
// 
// 		if(data.map.inRange(x, y)==false){
// 			data.map.approximate(x, y);
// 		}
// 
// 		return Waypoint((size_t)x, (size_t)y);
// 
// 		#undef TRANSLATE
// 	}
	GPSPoint cast(const Waypoint& wp)const;
// 	{
// 		#define TRANSLATE(x) TRANSLATE_POINT_GRID_TO_GPS(x)
// 
// 		if(isMapReady()==false){
// 			return GPSPoint(0,0);
// 		}
// 
// 		double x ( TRANSLATE(x) );
// 		double y ( TRANSLATE(y) );
// 
// 		return GPSPoint(x, y);
// 
// 		#undef TRANSLATE
// 	}

// 	#undef TRANSLATE_POINT_GPS_TO_GRID
// 	#undef TRANSLATE_POINT_GRID_TO_GPS
// 	#undef TRANSLATE_GPS_TO_GRID
// 	#undef TRANSLATE_GRID_TO_GPS



#undef SYNCHRONIZED
#undef LOCK
#undef UNLOCK

};


#endif