#ifndef _PATH_PLANNING_SERVER_H_
#define _PATH_PLANNING_SERVER_H_

//tasks dependencies
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

#include "PathPlanner.h"

//messages
#include <C31_PathPlanner/C31_PlanPath.h>
#include <C31_PathPlanner/C31_GetPath.h>

using namespace std;
using namespace C0_RobilTask;

#define DURATION(FTIME, STIME) ( difftime(STIME, FTIME) )
#define NOW time(NULL)
#define TIME_T time_t
#define TIME_STR(T) std::string(ctime(&T)).substr(0,std::string(ctime(&T)).size()-1)
#define SET_CURRENT_TIME(V) {V = NOW;}

class PathPlanningServer:public RobilTask{
	PathPlanning& _planner;

	//mutex needed for safe update of internal state of task (flag about new map or locations and statistic information)
	boost::mutex _mtx;
	//boost::condition_variable new_map_or_location_gotten;
	#define LOCK( X ) boost::shared_ptr<boost::mutex::scoped_lock> X(new boost::mutex::scoped_lock(_mtx));
	#define UNLOCK( X ) X = boost::shared_ptr<boost::mutex::scoped_lock>();
	#define SYNCH(V) {boost::mutex::scoped_lock locker(_mtx); V}

	bool new_map_or_location;

	class Statistic{
		friend class PathPlanningServer;
		TIME_T time_map_lastRequest;
		TIME_T time_map_lastReceive;
		TIME_T time_plan_startPlanning;
		TIME_T time_plan_stopPlanning;
		TIME_T time_location_lastRequest;
		TIME_T time_targetLocation_lastRequest;
		TIME_T time_location_lastReceive;
		//TIME_T time_target_lastReceive;
		Statistic(){
			::memset(this, 0, sizeof(Statistic));
		}
	};
	Statistic statistic;

public:
    PathPlanningServer(PathPlanning& planner, string name = "/PathPlanning"):
    	RobilTask(name), _planner(planner), new_map_or_location(false)
    {
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
				const Vec2d& wp = session.results.path[i];
				gpspath.push_back(planner.cast(wp));
			}
			//TODO: fill message by planner results
		}
		return true;
    }

    bool srv_GetPath( C31_PathPlanner::C31_GetPathRequest& req, C31_PathPlanner::C31_GetPathResponse& res){
      ROS_INFO("RETURN CALCULATED GLOBAL PATH");
      GPSPath path = get_calculated_path();
	  for( size_t i=0;i<path.size();i++ ){
		  C31_PathPlanner::C31_Location loc; loc.x=path[i].x; loc.y=path[i].y;
		  res.path.points.push_back(loc);
	  }
      return true;
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){
    	//ros::this_node::getName()
    	
    	// TASK INPUT CHANNELS
    	ros::ServiceServer c31_PlanPath =
    			_node.advertiseService<C31_PathPlanner::C31_PlanPathRequest, C31_PathPlanner::C31_PlanPathResponse>(
    					STR(ros::this_node::getName()<<"/planPath"),boost::bind(&PathPlanningServer::srv_PlanPath,this,_1,_2)
    			);
    	ros::ServiceServer c31_GetPath =
    			_node.advertiseService<C31_PathPlanner::C31_GetPathRequest, C31_PathPlanner::C31_GetPathResponse>(
    					STR(ros::this_node::getName()<<"/getPath"),boost::bind(&PathPlanningServer::srv_GetPath,this,_1,_2)
    			);

		//TASK OUTPUT CHANNELS
    	ros::ServiceClient c22Client = _node.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");
		ros::Subscriber c23Client = _node.subscribe("C23/object_deminsions", 1000, &PathPlanningServer::callbackNewTargetLocation, this );
		

        while(true){
            if (isPreempt()){

                /* HERE PROCESS PREEMPTION OR INTERAPT */

            	return TaskResult::Preempted();
            }

            /* HERE PROCESS TASK */

            LOCK( locker_nds )
            	bool needNewMap = requestNewMapNeeded();
            	bool needNewLoc = requestNewLocationNeeded();
            	bool needNewTargetLoc = requestNewTargetLocationNeeded();
            UNLOCK( locker_nds )

				if(needNewMap) requestNewMap(c22Client);
				if(needNewLoc) requestNewLocation();
				if(needNewTargetLoc) requestNewTargetLocation();

            LOCK( locker )
            if(exists_new_map_or_location()){

					map_and_location_gotten();

			UNLOCK( locker )

					SET_CURRENT_TIME(statistic.time_plan_startPlanning);
					ROS_INFO("%s: plan path", _name.c_str());
					_planner.plan();
					SET_CURRENT_TIME(statistic.time_plan_stopPlanning);

            }else{

            UNLOCK( locker )

            		ROS_INFO("%s: wait for new data (map, location, target, constraints, etc.)", _name.c_str());

            }

            /* SLEEP BETWEEN LOOP ITERATIONS */
            sleep(1000); //millisec
        }

        return TaskResult::FAULT();
    }


    //==================== NEW DATA CONVERTING FROM GRID TO GPS WORLD =====================
    GPSPath get_calculated_path(){
    	PathPlanning::ReadSession session = _planner.startReading();
    	GPSPath gpspath;
    	for(size_t i=0;i<session.results.path.size();i++){
    		const Vec2d& wp = session.results.path[i];
			GPSPoint gpsp = _planner.cast(wp);
    		gpspath.push_back(gpsp);
			//ROS_INFO("PATH: cast %i,%i  ->  %f,%f", wp.x, wp.y,  gpsp.x, gpsp.y);
    	}
    	return gpspath;
    }

    //=================== NEW DATA REQUESTS ===============================================
    bool requestNewMapNeeded(){ //REQ. for external synchronization on _mtx
    	TIME_T now = NOW;
    	double duration = DURATION(statistic.time_map_lastReceive, now);
    	//ROS_INFO("requestNewMapNeeded : %s",STR("Now="<<TIME_STR(now)<<", lastReceive="<<TIME_STR(statistic.time_map_lastReceive)<<", duration="<<duration<<"s"));
    	return duration > 1;
    }
    void requestNewMap(ros::ServiceClient & c22Client){ //REQ. HAS BE NOT external synchronizationed on _mtx

		C22_GroundRecognitionAndMapping::C22 c22;
		/*
			C22_GroundRecognitionAndMapping/C22C0_PATH drivingPath
			C22_GroundRecognitionAndMapping/C22_ROW_TYPE[] row
				C22_GroundRecognitionAndMapping/C22_MAP_SQUARE[] column
				int32 AVAILABLE=0
				int32 BLOCKED=1
				int32 UNCHARTED=2
				int32 status
				C22_GroundRecognitionAndMapping/C22_PLANE_TYPE[] planes
					float32 x
					float32 y
					float32 z
					float32 d
			int32 xOffset
			int32 yOffset
			geometry_msgs/Point robotPos
				float64 x
				float64 y
				float64 z
		*/
		SYNCH(SET_CURRENT_TIME(statistic.time_map_lastRequest));
		if (c22Client.call(c22)){
			ROS_INFO("map message is gotten");
			SYNCH(SET_CURRENT_TIME(statistic.time_map_lastReceive));
			
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
    bool requestNewLocationNeeded(){ //REQ. HAS BE NOT external synchronizationed on _mtx
    	return false;
    }
    void requestNewLocation(){ //REQ. HAS BE NOT external synchronizationed on _mtx
		//TODO: [CURRENTLLY NOT ACTUAL] write real algorithm for requestNewLocation
		//......we get location from map message
    	SYNCH(SET_CURRENT_TIME(statistic.time_location_lastRequest));
		//onNewLocation(NEW_ROBOT_LOCATION_GPS)
    }
    bool requestNewTargetLocationNeeded(){ //REQ. HAS BE NOT external synchronizationed on _mtx
		double duration = DURATION(statistic.time_map_lastReceive, NOW);
		PathPlanning::ReadSession session = _planner.startReading();
		if(session.arguments.targetGoal.size()>0 && duration>1) return true;
    	return false;
    }
    void requestNewTargetLocation(){ //REQ. HAS BE NOT external synchronizationed on _mtx
		//TODO: [CURRENTLLY NOT ACTUAL] write real algorithm for requestNewTargetLocation
		//......we get location by callback from Topic listener
    	SYNCH(SET_CURRENT_TIME(statistic.time_targetLocation_lastRequest));
    	//onNewTargetLocation(NEW_TARGET_LOCATION_GPS)
    }
    void callbackNewTargetLocation(const C23_ObjectRecognition::C23C0_ODIM::ConstPtr & msg){
		onNewTargetLocation( extractObjectLocation( *msg ) );
	}

    //=================== NEW DATA INPUT ==================================================
    void dataChanged(){
    	ROS_INFO("%s: data changed", _name.c_str());
    	boost::mutex::scoped_lock locker(_mtx);
    	new_map_or_location = true;
    }

    bool exists_new_map_or_location(){ //REQ. for external synchronization on _mtx
    	return new_map_or_location;
    }
    void map_and_location_gotten(){ //REQ. for external synchronization on _mtx
    	new_map_or_location = false;
    }

    void onNewMap(const Map map, const MapProperties& prop){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.map = map;
    	session.arguments.mapProperties = prop;
    	session.arguments.start = _planner.cast(session.arguments.selfLocation);
    	session.arguments.finish = _planner.cast(session.arguments.targetPosition);
    	session.constraints.dimentions.radius = _planner.castLength(session.constraints.dimentions.gps_radius);
		ROS_INFO("GPS_GRID_CASTING: start=*(%f,%f)->(%i,%i), finis=*(%f,%f)->(%i,%i), robot.rad=*%f->%i",
			session.arguments.selfLocation.x, session.arguments.selfLocation.y, session.arguments.start.x, session.arguments.start.y,
			session.arguments.targetPosition.x, session.arguments.targetPosition.y, session.arguments.finish.x, session.arguments.finish.y,
			session.constraints.dimentions.gps_radius, session.constraints.dimentions.radius
		);
    }
    void onNewLocation(const GPSPoint& pos, const Waypoint& wp){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = wp;
    	session.arguments.selfLocation = pos;
		ROS_INFO("GPS_GRID_CASTING: start=*(%f,%f)->*(%i,%i), finis=(%f,%f)->(%i,%i), robot.rad=%f->%i",
			session.arguments.selfLocation.x, session.arguments.selfLocation.y, session.arguments.start.x, session.arguments.start.y,
			session.arguments.targetPosition.x, session.arguments.targetPosition.y, session.arguments.finish.x, session.arguments.finish.y,
			session.constraints.dimentions.gps_radius, session.constraints.dimentions.radius
		);
    }
    void onNewTargetLocation(const GPSPoint& pos){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.targetPosition = pos;
    	session.arguments.finish = _planner.cast(session.arguments.targetPosition);
		ROS_INFO("GPS_GRID_CASTING: start=(%f,%f)->(%i,%i), finis=*(%f,%f)->(%i,%i), robot.rad=%f->%i",
			session.arguments.selfLocation.x, session.arguments.selfLocation.y, session.arguments.start.x, session.arguments.start.y,
			session.arguments.targetPosition.x, session.arguments.targetPosition.y, session.arguments.finish.x, session.arguments.finish.y,
			session.constraints.dimentions.gps_radius, session.constraints.dimentions.radius
		);
    }
    void onNewLocation(const GPSPoint& pos){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = _planner.cast(pos);
    	session.arguments.selfLocation = pos;
		ROS_INFO("GPS_GRID_CASTING: start=*(%f,%f)->(%i,%i), finis=(%f,%f)->(%i,%i), robot.rad=%f->%i",
			session.arguments.selfLocation.x, session.arguments.selfLocation.y, session.arguments.start.x, session.arguments.start.y,
			session.arguments.targetPosition.x, session.arguments.targetPosition.y, session.arguments.finish.x, session.arguments.finish.y,
			session.constraints.dimentions.gps_radius, session.constraints.dimentions.radius
		);
    }
    void onNewConstraints(const Constraints& constr){
    	PathPlanning::EditSession session = _planner.startEdit();
    }

	#undef LOCK
	#undef UNLOCK
};

class PathPlanningFocusServer:public RobilTask{
	PathPlanning& _planner;
public:
    PathPlanningFocusServer(PathPlanning& planner, string name = "/PathPlanningFocus"):
    	RobilTask(name), _planner(planner)
    {  }

    TaskResult task(const string& name, const string& uid, Arguments& args){

		if( args.find("x")!=args.end() && args.find("y")!=args.end() ){
			std::stringstream numbers; numbers<<(args["x"])<<','<<(args["y"]);
			char c; double x, y;
			numbers>>x>>c>>y;
			ROS_INFO("%s: set planning goal to [x,y] = %f, %f", _name.c_str(), x, y);

			PathPlanning::EditSession session = _planner.startEdit();

			if(_planner.isMapReady()==false){ session.aborted(); return TaskResult(FAULT, "Map is not ready"); }

			session.arguments.targetGoal="";
			session.arguments.targetPosition.x=x;
			session.arguments.targetPosition.y=y;
			session.arguments.finish = _planner.cast(session.arguments.targetPosition);
			stringstream info; info<<"CONVERT "
			<<session.arguments.targetPosition.x<<","<<session.arguments.targetPosition.y
			<<" -> "
			<<session.arguments.finish.x<<","<<session.arguments.finish.y<<endl;
			ROS_INFO(info.str().c_str());

			return TaskResult(SUCCESS, "OK");
		}else{
			string desc = "I don't know to set path planner goal from current parameters ";
			ROS_INFO("%s: ERROR: %s", _name.c_str(), desc.c_str());

			return TaskResult(FAULT, desc);
		}

        return TaskResult::FAULT();
    }

};

#endif //_PATH_PLANNING_SERVER_H_
