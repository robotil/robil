#ifndef _PATH_PLANNING_SERVER_H_
#define _PATH_PLANNING_SERVER_H_

//tasks dependencies
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

//common utils
#include <sstream>
#include <iostream>
#include <time.h>

//local pathplanner dependencies
#include "Gps.h"
#include "cogniteam_pathplanning.h"
#include "ConvertorC22.hpp"


//messages
#include <C31_PathPlanner/C31_PlanPath.h>
#include <C31_PathPlanner/C31_GetPath.h>



using namespace std;
using namespace C0_RobilTask;
using namespace RobilTask;

#define DURATION(FTIME, STIME) ( difftime(STIME, FTIME) )
#define NOW time(NULL)
#define TIME_T time_t
#define TIME_STR(T) std::string(ctime(&T)).substr(0,std::string(ctime(&T)).size()-1)


typedef GPSPoint TargetPosition;
typedef GPSPoint Localization;

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
	const GPSPoint& selfLocation;
	const MapProperties& mapProperties;
	PlanningArguments(const Map& map, const Waypoint& start, const Waypoint& finish, const TargetPosition& targetPos, const GPSPoint& sloc, const MapProperties& mapProperties)
	:map(map), start(start), finish(finish), targetPosition(targetPos), selfLocation(sloc), mapProperties(mapProperties){}
};
class Editable_PlanningArguments{
public:
	Map& map;
	Waypoint& start;
	Waypoint& finish;
	TargetPosition& targetPosition;
	GPSPoint& selfLocation;
	MapProperties& mapProperties;
	Editable_PlanningArguments(Map& map, Waypoint& start, Waypoint& finish, TargetPosition& targetPos, GPSPoint& sloc, MapProperties& mapProperties)
	:map(map), start(start), finish(finish), targetPosition(targetPos), selfLocation(sloc), mapProperties(mapProperties){}
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
		targetPosition(0,0), selfLocation(0,0), mapProperties(0.12, GPSPoint(0,0), Waypoint(0,0)),
		//input data
		data(),
		//output data
		path(),
		//parameters interfaces
		//...read-only
		   arguments(data.map, data.start, data.finish, targetPosition, selfLocation, mapProperties),
		   constraints(data.dimentions, data.transits, data.attractors), results(path),
		//...read-write
		ed_arguments(data.map, data.start, data.finish, targetPosition, selfLocation, mapProperties),
		ed_constraints(data.dimentions, data.transits, data.attractors)
	{

	}

	template <typename CALLBACK>
	void setChangeNotifier(CALLBACK cb){ changeNotification = boost::shared_ptr<ChangesNotification>(new _ChangesNotification<CALLBACK>(cb)); }

	void plan(){

		LOCK( locker_bfr )
			PlanningInputData _data(data);
		UNLOCK( locker_bfr )

		stringstream out;
		out<<"searchPath(map="<<_data.map.w()<<"x"<<_data.map.h()
				<<", start="<<_data.start.x<<","<<_data.start.y
				<<", finish="<<_data.finish.x<<","<<_data.finish.y
				<<", const.dim="<<_data.dimentions.radius
				<<", const.trans#="<<_data.transits.size()
				<<", const.attractors#="<<_data.attractors.size()
		<<")";
		ROS_INFO("PathPlanning::plan : %s",out.str().c_str());

		Constraints constraints(_data.dimentions, _data.transits, _data.attractors);
		Path _path ;//= searchPath(_data.map, _data.start, _data.finish, constraints);

		LOCK( locker_aft )
			path = _path;
		UNLOCK( locker_aft )
	}

	class EditSession{
		boost::shared_ptr<boost::mutex::scoped_lock> l;
		boost::shared_ptr<ChangesNotification> changeNotification;
	public:
		Editable_Constraints& constraints;
		Editable_PlanningArguments& arguments;
		EditSession(boost::mutex& _mtx, Editable_Constraints& cons, Editable_PlanningArguments& arguments, boost::shared_ptr<ChangesNotification> ch_notify):
			l(new boost::mutex::scoped_lock(_mtx)), changeNotification(ch_notify),constraints(cons),arguments(arguments)  {}
		EditSession(const EditSession& e):
			l(e.l), changeNotification(e.changeNotification),constraints(e.constraints),arguments(e.arguments)  {}
		~EditSession(){ if(changeNotification.get()) changeNotification->notify(); }
	};
	class ReadSession{
		boost::shared_ptr<boost::mutex::scoped_lock> l;
	public:
		const Constraints& constraints;
		const PlanningArguments& arguments;
		const PlanningResult& results;
		ReadSession(boost::mutex& _mtx, const Constraints& cons, const PlanningArguments& argumentsc, const PlanningResult& results):
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


	Waypoint cast(const GPSPoint& gps)const{
		#define TRANSLATE(x) ( mapProperties.anchor.x + round( ( gps.x - mapProperties.gps.x) / mapProperties.resolution ) )

		long x ( TRANSLATE(x) );
		long y ( TRANSLATE(y) );

		if(data.map.inRange(x, y)==false){
			data.map.approximate(x, y);
		}

		return Waypoint((size_t)x, (size_t)y);

		#undef TRANSLATE
	}
	GPSPoint cast(const Waypoint& wp)const{
		#define TRANSLATE(x) ( mapProperties.gps.x +  (wp.x - mapProperties.anchor.x) * mapProperties.resolution  )

		double x ( TRANSLATE(x) );
		double y ( TRANSLATE(y) );

		return GPSPoint(x, y);

		#undef TRANSLATE
	}


#undef SYNCHRONIZED
#undef LOCK
#undef UNLOCK

};


class PathPlanningServer:public AbstractTask{
	PathPlanning& _planner;

	bool new_map_or_location;
	boost::mutex _mtx;
	//boost::condition_variable new_map_or_location_gotten;
#define LOCK( X ) boost::shared_ptr<boost::mutex::scoped_lock> X(new boost::mutex::scoped_lock(_mtx));
#define UNLOCK( X ) X = boost::shared_ptr<boost::mutex::scoped_lock>();

	class Statistic{
		friend class PathPlanningServer;
		TIME_T time_map_lastRequest;
		TIME_T time_map_lastReceive;
		TIME_T time_plan_start_planning;
		TIME_T time_plan_stop_planning;
		TIME_T time_location_lastRequest;
		TIME_T time_location_lastReceive;
		//TIME_T time_target_lastReceive;
		Statistic(){
			::memset(this, 0, sizeof(Statistic));
		}
	};
	Statistic statistic;

public:
    PathPlanningServer(PathPlanning& planner, string name = "/PathPlanning"):
    	AbstractTask(name), _planner(planner), new_map_or_location(false)
    {
    	ROS_INFO("=========== TEST STR: %s ===========", STR("1"<<","<<2<<","<<3.0<<"!!!") );
    	_planner.setChangeNotifier(boost::bind(&PathPlanningServer::dataChanged, this));

    }

    bool srv_PlanPath( C31_PathPlanner::C31_PlanPathRequest& req, C31_PathPlanner::C31_PlanPathResponse& res){
		ROS_INFO("START CALCULATION OF GLOBAL PATH PLANNER");

		PathPlanning planner;

		{ PathPlanning::EditSession session = planner.startEdit();
		  //TODO: fill data of planner from message
		}

		planner.plan();

		{ PathPlanning::ReadSession session = planner.startReading();
			GPSPath gpspath;
			for(size_t i=0;i<session.results.path.size();i++){
				const Waypoint& wp = session.results.path[i];
				gpspath.push_back(_planner.cast(wp));
			}
			//TODO: fill message by planner results
		}
		return true;
    }

    bool srv_GetPath( C31_PathPlanner::C31_GetPathRequest& req, C31_PathPlanner::C31_GetPathResponse& res){
      ROS_INFO("RETURN CALCULATED GLOBAL PATH");
      GPSPath path = get_calculated_path();
      //TODO: fill message by planner results
      return true;
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){

    	//ros::this_node::getName()
    	ros::ServiceServer c31_PlanPath =
    			_node.advertiseService<C31_PathPlanner::C31_PlanPathRequest, C31_PathPlanner::C31_PlanPathResponse>(
    					STR(ros::this_node::getName()<<"/planPath"),boost::bind(&PathPlanningServer::srv_PlanPath,this,_1,_2)
    			);
    	ros::ServiceServer c31_GetPath =
    			_node.advertiseService<C31_PathPlanner::C31_GetPathRequest, C31_PathPlanner::C31_GetPathResponse>(
    					STR(ros::this_node::getName()<<"/getPath"),boost::bind(&PathPlanningServer::srv_GetPath,this,_1,_2)
    			);

    	ros::ServiceClient c22Client = _node.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");

    	/* NUMBER OF ITERATIONS IN TASK LOOP */
        while(true){
            if (isPreempt()){

                /* HERE PROCESS PREEMPTION OR INTERAPT */

            	return TaskResult::Preempted();
            }

            /* HERE PROCESS TASK */

            LOCK( locker_nds )
            	bool needNewMap = requestNewMapNeeded();
            	bool needNewLoc = requestNewLocationNeeded();
            UNLOCK( locker_nds )

            if(needNewMap) requestNewMap(c22Client);
            if(needNewLoc) requestNewLocation();

            LOCK( locker )
            if(exists_new_map_or_location()){

					map_and_location_gotten();

			UNLOCK( locker )

					statistic.time_plan_start_planning = NOW;
					ROS_INFO("%s: plan path", _name.c_str());
					_planner.plan();
					statistic.time_plan_stop_planning = NOW;

            }else{

            UNLOCK( locker )

            		ROS_INFO("%s: wait for new data (map, location, target, constraints, etc.)", _name.c_str());

            }



            /* SLEEP BETWEEN LOOP ITERATIONS */
            sleep(1000);
        }

        return TaskResult::FAULT();
    }


    //==================== NEW DATA CONVERTING FROM GRID TO GPS WORLD =====================
    GPSPath get_calculated_path(){
    	PathPlanning::ReadSession session = _planner.startReading();
    	GPSPath gpspath;
    	for(size_t i=0;i<session.results.path.size();i++){
    		const Waypoint& wp = session.results.path[i];
    		gpspath.push_back(_planner.cast(wp));
    	}
    	return gpspath;
    }

    //=================== NEW DATA REQUESTS ===============================================
    bool requestNewMapNeeded(){
    	TIME_T now = NOW;
    	double duration = DURATION(statistic.time_map_lastReceive, now);
    	//ROS_INFO("requestNewMapNeeded : %s",STR("Now="<<TIME_STR(now)<<", lastReceive="<<TIME_STR(statistic.time_map_lastReceive)<<", duration="<<duration<<"s"));
    	return duration > 1;
    }
    void requestNewMap(ros::ServiceClient & c22Client){

		C22_GroundRecognitionAndMapping::C22 c22;
		/*
			C0C22_AZI azimuth_msg
				float32 azimuth
			C0C22_CAM camera_sample_rate_msg
				int32 frameRatePerSec
			C0C22_LAZ laser_sample_rate_msg
				int32 sampleRatePerSec
			C0C22_SAF safety_requirements
				int32 safety_req
			---
			C22C0_PATH drivingPath
				C22_ROW_TYPE[] row
					C22_MAP_SQUARE[] column
						int32 status
						int32 AVAILABLE=0
						int32 BLOCKED=1
						int32 UNCHARTED=2
						C22_PLANE_TYPE[] planes
							float32 x
							float32 y
							float32 z
							float32 d
		*/
		statistic.time_map_lastRequest = NOW;
		if (c22Client.call(c22)){
			statistic.time_map_lastReceive = NOW;
			MapProperties mprop = extractMapProperties(c22.response);
			Map map = extractMap(c22.response);
			Gps2Grid gps_grid = extractLocation(c22.response);
			if(map.w()>0 && map.h()>0){
				onNewMap( map , mprop );
				onNewLocation( gps_grid.gps, gps_grid.cell );
			}else{
				ROS_ERROR("Map gotten from C22_GroundRecognitionAndMapping::C22 is EMPTY (size=0x0)");
			}
		}else{
			ROS_ERROR("Failed to call service C22_GroundRecognitionAndMapping::C22");
		}
    }
    bool requestNewLocationNeeded(){
    	return false;
    }
    void requestNewLocation(){
    	statistic.time_location_lastRequest = NOW;
    	//statistic.time_location_lastReceive = NOW;
    }

    //=================== NEW DATA INPUT ==================================================
    void dataChanged(){
    	ROS_INFO("%s: data changed", _name.c_str());
    	boost::mutex::scoped_lock locker(_mtx);
    	new_map_or_location = true;
    }

    bool exists_new_map_or_location(){
    	return new_map_or_location;
    }
    void map_and_location_gotten(){
    	new_map_or_location = false;
    }

    void onNewMap(const Map map, const MapProperties& prop){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.map = map;
    	session.arguments.mapProperties = prop;
    }
    void onNewLocation(const GPSPoint& pos, const Waypoint& wp){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = wp;
    	session.arguments.selfLocation = pos;
    }
    void onNewLocation(const GPSPoint& pos){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = _planner.cast(pos);
    	session.arguments.selfLocation = pos;
    }
    void onNewConstraints(const Constraints& constr){
    	PathPlanning::EditSession session = _planner.startEdit();
    }

#undef LOCK
#undef UNLOCK
};

class PathPlanningFocusServer:public AbstractTask{
	PathPlanning& _planner;
public:
    PathPlanningFocusServer(PathPlanning& planner, string name = "/PathPlanningFocus"):
    	AbstractTask(name), _planner(planner)
    {  }

    TaskResult task(const string& name, const string& uid, Arguments& args){

		if( args.find("x")!=args.end() && args.find("y")!=args.end() ){
			std::stringstream numbers; numbers<<(args["x"])<<','<<(args["y"]);
			char c; double x, y;
			numbers>>x>>c>>y;
			ROS_INFO("%s: set planning goal to [x,y] = %f, %f", _name.c_str(), x, y);
			PathPlanning::EditSession session = _planner.startEdit();
			session.arguments.targetPosition.x=x;
			session.arguments.targetPosition.y=y;
			session.arguments.finish = _planner.cast(session.arguments.targetPosition);

			return TaskResult(SUCCESS, "OK");
		}else{
			string desc = "I don't know to set path planner goal from current parameters ";
			ROS_INFO("%s: ERROR: %s", _name.c_str(), desc.c_str());

			return TaskResult(FAULT, desc);
		}
//        /* NUMBER OF ITERATIONS IN TASK LOOP */
//        while(true){
//            if (isPreempt()){
//
//                /* HERE PROCESS PREEMPTION OR INTERAPT */
//
//
//                return TaskResult::Preempted();
//            }
//
//            /* HERE PROCESS TASK */
//
//
//
//            /* SLEEP BETWEEN LOOP ITERATIONS */
//            sleep(100);
//        }

        return TaskResult::FAULT();
    }

};

#endif //_PATH_PLANNING_SERVER_H_
