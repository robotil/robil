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
#include <C31_PathPlanner/C31_Exception.h>

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

	ros::Publisher* c31_Exception;
public:
    PathPlanningServer(PathPlanning& planner, string name = "/PathPlanning"):
    	RobilTask(name), _planner(planner), new_map_or_location(false), c31_Exception(NULL)
    {
    	_planner.setChangeNotifier(boost::bind(&PathPlanningServer::dataChanged, this));

    }

    bool srv_PlanPath( C31_PathPlanner::C31_PlanPathRequest& req, C31_PathPlanner::C31_PlanPathResponse& res){
    	/*[CURRENTLLY NOT ACTUAL]
		ROS_INFO("START CALCULATION OF GLOBAL PATH PLANNER");

		PathPlanning planner;

		{ PathPlanning::EditSession session = planner.startEdit();
		  //TODO:  [CURRENTLLY NOT ACTUAL] fill data of planner from message
		}

		planner.plan();

		{ PathPlanning::ReadSession session = planner.startReading();
			GPSPath gpspath;
			for(size_t i=0;i<session.results.path.size();i++){
				const Vec2d& wp = session.results.path[i];
				gpspath.push_back(planner.cast(wp));
			}
			//TODO:  [CURRENTLLY NOT ACTUAL] fill message by planner results
		}*/
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
    void publish_new_plan(ros::Publisher& c31_PathPublisher){
    	//if(c31_PathPublisher.getNumSubscribers()>0){
			ROS_INFO("PUBLISH CALCULATED GLOBAL PATH");
			GPSPath path = get_calculated_path();
			C31_PathPlanner::C31_Waypoints res_path;
			for( size_t i=0;i<path.size();i++ ){
			  C31_PathPlanner::C31_Location loc; loc.x=path[i].x; loc.y=path[i].y;
			  res_path.points.push_back(loc);
			}
		if(c31_PathPublisher.getNumSubscribers()>0){
			c31_PathPublisher.publish(res_path);
		}
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){
    	//ros::this_node::getName()
    	
    	// TASK OUTPUT CHANNELS
    	/* [CURRENTLLY NOT ACTUAL]
    	ROS_INFO("advertise service /planPath <C31_PathPlanner::C31_PlanPath> ");
    	ros::ServiceServer c31_PlanPath =
    			_node.advertiseService<C31_PathPlanner::C31_PlanPathRequest, C31_PathPlanner::C31_PlanPathResponse>(
    					STR(ros::this_node::getName()<<"/planPath"),boost::bind(&PathPlanningServer::srv_PlanPath,this,_1,_2)
    			);
    	*/
    	ROS_INFO("advertise service /getPath <C31_PathPlanner::C31_GetPath>");
    	ros::ServiceServer c31_GetPath =
    			_node.advertiseService<C31_PathPlanner::C31_GetPathRequest, C31_PathPlanner::C31_GetPathResponse>(
    					STR(ros::this_node::getName()<<"/getPath"),boost::bind(&PathPlanningServer::srv_GetPath,this,_1,_2)
    			);
    	ROS_INFO("advertise topic /path <C31_PathPlanner::C31_Waypoints>");
    	ros::Publisher c31_PathPublisher =
    			_node.advertise<C31_PathPlanner::C31_Waypoints>("/path", 10);

    	ROS_INFO("advertise topic /path/exceptions <C31_PathPlanner::C31_Exception>");
    	ros::Publisher c31_Exception =
    			_node.advertise<C31_PathPlanner::C31_Exception>("/path/exceptions", 10);
    	this->c31_Exception= &c31_Exception;

		//TASK INPUT CHANNELS
    	ROS_INFO("subscribe to service /C22 <C22_GroundRecognitionAndMapping::C22>");
    	ros::ServiceClient c22Client = _node.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");
    	
		ROS_INFO("subscribe to topic /C22_pub <C22_GroundRecognitionAndMapping::C22C0_PATH>");
    	ros::Subscriber c22c0Client = _node.subscribe("C22_pub", 1000, &PathPlanningServer::callbackNewMap, this );
    	
		ROS_INFO("subscribe to topic /C23/object_deminsions <C23_ObjectRecognition::C23C0_ODIM>");
		ros::Subscriber c23Client = _node.subscribe("C23/object_deminsions", 1000, &PathPlanningServer::callbackNewTargetLocation, this );

		ROS_INFO("subscribe to topic /c11_path_update <C31_PathPlanner/C31_Waypoints>");
		ros::Subscriber c11Client = _node.subscribe("/c11_path_update", 1000, &PathPlanningServer::callbackTransitPoints, this );

		#define TURNON_REQUEST_MAP
		//#define TURNON_REQUEST_TARGET_LOCATION
		
		#ifdef TURNON_REQUEST_MAP
			ROS_INFO("TURNON_REQUEST_MAP is defined");
		#else
			ROS_INFO("TURNON_REQUEST_MAP is undefined");
		#endif
		#ifdef TURNON_REQUEST_TARGET_LOCATION
			ROS_INFO("TURNON_REQUEST_TARGET_LOCATION is defined");
		#else
			ROS_INFO("TURNON_REQUEST_TARGET_LOCATION is undefined");
		#endif
		
        while(true){
            if (isPreempt()){

                /* HERE PROCESS PREEMPTION OR INTERAPT */

            	this->c31_Exception=0;
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

				bool targetDefined = false;
				{PathPlanning::ReadSession sess = _planner.startReading(); targetDefined = sess.arguments.targetDefined;}

            LOCK( locker )
            if(exists_new_map_or_location() && targetDefined){

					map_and_location_gotten();

			UNLOCK( locker )

					SET_CURRENT_TIME(statistic.time_plan_startPlanning);
					ROS_INFO("%s: plan path", _name.c_str());
					if( _planner.plan() ){
						publish_new_plan(c31_PathPublisher);
					}else{
						throw_exception(C31_PathPlanner::C31_Exception::TYPE_NOSOLUTIONFORPLAN, "No solution for plan found.");
					}
					SET_CURRENT_TIME(statistic.time_plan_stopPlanning);

            }else{

            UNLOCK( locker )

            		ROS_INFO("%s: wait for new data (map, location, target, constraints, etc.)%s", _name.c_str(), (targetDefined?"":", target still not defined"));

            }

            /* SLEEP BETWEEN LOOP ITERATIONS */
            sleep(1000); //millisec
        }

        this->c31_Exception=0;
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
			ROS_INFO("PATH: cell:%i,%i  ->  gps:%f,%f", (int) wp.x, (int) wp.y, (float) gpsp.x, (float) gpsp.y);
    	}
    	return gpspath;
    }

    void throw_exception(int type, std::string desc){
    	C31_PathPlanner::C31_Exception exc;
		exc.type = type;
		exc.description = desc;
		if(c31_Exception) c31_Exception->publish(exc);

		this->_planner.events().push(cast(exc));
    }

    //=================== NEW DATA REQUESTS ===============================================
    //================= MAP
    bool requestNewMapNeeded(){ //REQ. for external synchronization on _mtx
#ifdef TURNON_REQUEST_MAP
    	TIME_T now = NOW;
    	double duration = DURATION(statistic.time_map_lastReceive, now);
    	//ROS_INFO("requestNewMapNeeded : %s",STR("Now="<<TIME_STR(now)<<", lastReceive="<<TIME_STR(statistic.time_map_lastReceive)<<", duration="<<duration<<"s"));
    	return duration > 5;
#else
    	return false;
#endif
    }
    void requestNewMap(ros::ServiceClient & c22Client){ //REQ. HAS BE NOT external synchronized on _mtx

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
				float64 x // <- x pos
				float64 y // <- y pos
				float64 z
			geometry_msgs/Point robotOri
				float64 x
				float64 y
				float64 z // <- heading
		*/
		SYNCH(SET_CURRENT_TIME(statistic.time_map_lastRequest));
		if (c22Client.call(c22)){
			ROS_INFO("map message is gotten");
			SYNCH(SET_CURRENT_TIME(statistic.time_map_lastReceive));
			
			MapProperties mprop = extractMapProperties(c22.response);
			Map map = extractMap(c22.response, mprop);
			Gps2Grid gps_grid = extractLocation(c22.response, mprop);
			
			bool map_size_ok = map.w()>0 && map.h()>0;
			bool robot_location_ok = map_size_ok &&
					(mprop.gps.x<=gps_grid.gps.x)&&(mprop.gps.y<=gps_grid.gps.y)
					&&
					(gps_grid.gps.x<=mprop.gps.x+map.w()*mprop.resolution)&&(gps_grid.gps.y<=mprop.gps.y+map.h()*mprop.resolution);

			if(map_size_ok && robot_location_ok){
			
				onNewMap( map , mprop );
				onNewLocation( gps_grid.gps, gps_grid.cell );
				
			}else{
				if(!map_size_ok){
					ROS_ERROR("Map gotten from C22_GroundRecognitionAndMapping::C22 is EMPTY (size=0x0)");
					throw_exception(C31_PathPlanner::C31_Exception::TYPE_MAPISEMPTY, "Map gotten from C22_GroundRecognitionAndMapping::C22 is EMPTY (size=0x0)");
				}
				if(!robot_location_ok){
					ROS_ERROR("Map gotten from C22_GroundRecognitionAndMapping::C22 is out of map: map.offset=(%f,%f), map.size=(%f,%f), robot=(%f,%f)",
							(float) mprop.gps.x, (float) mprop.gps.y,
							(float) map.w()*mprop.resolution, (float) map.h()*mprop.resolution,
							(float) gps_grid.gps.x , (float) gps_grid.gps.y
					);
					throw_exception(C31_PathPlanner::C31_Exception::TYPE_ROBOTOUTOFMAP, "Map gotten from C22_GroundRecognitionAndMapping::C22 is out of map");
				}
			}
		}else{
			ROS_ERROR("Failed to call service C22_GroundRecognitionAndMapping::C22");
			throw_exception(C31_PathPlanner::C31_Exception::TYPE_FAILTEDTOCALLSERVICEC22, "Failed to call service C22_GroundRecognitionAndMapping::C22");
		}
    }

    void callbackNewMap(const C22_GroundRecognitionAndMapping::C22C0_PATH::ConstPtr & msg){
		SYNCH(SET_CURRENT_TIME(statistic.time_map_lastRequest));

		ROS_INFO("map message is gotten");
		SYNCH(SET_CURRENT_TIME(statistic.time_map_lastReceive));

		MapProperties mprop = extractMapProperties(*msg);
		Map map = extractMap(*msg, mprop);
		Gps2Grid gps_grid = extractLocation(*msg, mprop);

		bool map_size_ok = map.w()>0 && map.h()>0;
		bool robot_location_ok = map_size_ok &&
				(mprop.gps.x<=gps_grid.gps.x)&&(mprop.gps.y<=gps_grid.gps.y)
				&&
				(gps_grid.gps.x<=mprop.gps.x+map.w()*mprop.resolution)&&(gps_grid.gps.y<=mprop.gps.y+map.h()*mprop.resolution);

		if(map_size_ok && robot_location_ok){

			onNewMap( map , mprop );
			onNewLocation( gps_grid.gps, gps_grid.cell );

		}else{
			if(!map_size_ok){
				ROS_ERROR("Map gotten from C22_pub<C22_GroundRecognitionAndMapping::C22C0_PATH> is EMPTY (size=0x0)");
				throw_exception(C31_PathPlanner::C31_Exception::TYPE_MAPISEMPTY, "Map gotten from C22_GroundRecognitionAndMapping::C22 is EMPTY (size=0x0)");
			}
			if(!robot_location_ok){
				ROS_ERROR("Robot location gotten from C22_pub<C22_GroundRecognitionAndMapping::C22C0_PATH> is out of map: map.offset=(%f,%f), map.size=(%f,%f), robot=(%f,%f)",
						(float) mprop.gps.x, (float) mprop.gps.y,
						(float) map.w()*mprop.resolution, (float) map.h()*mprop.resolution,
						(float) gps_grid.gps.x , (float) gps_grid.gps.y
				);
				throw_exception(C31_PathPlanner::C31_Exception::TYPE_ROBOTOUTOFMAP, "Map gotten from C22_GroundRecognitionAndMapping::C22 is out of map");
			}
		}

	}

    //================ ROBOT POSITION

    bool requestNewLocationNeeded(){ //REQ. HAS BE NOT external synchronized on _mtx
    	return false;
    }
    void requestNewLocation(){ //REQ. HAS BE NOT external synchronized on _mtx
		//TODO: [CURRENTLLY NOT ACTUAL] write real algorithm for requestNewLocation
		//......we get location from map message
    	SYNCH(SET_CURRENT_TIME(statistic.time_location_lastRequest));
		//onNewLocation(NEW_ROBOT_LOCATION_GPS)
    }

    //================= TARGET LOCATION

    bool requestNewTargetLocationNeeded(){ //REQ. HAS BE NOT external synchronized on _mtx
#ifdef TURNON_REQUEST_TARGET_LOCATION
		double duration = DURATION(statistic.time_map_lastReceive, NOW);
		PathPlanning::ReadSession session = _planner.startReading();
		//if defined targetGoal (name of goal object as string) and last target location update is older then 1 sec => request update;
		if(session.arguments.targetGoal.size()>0 && duration>1) return true;
    	return false;
#else
    	return false;
#endif
    }
    void requestNewTargetLocation(){ //REQ. HAS BE NOT external synchronized on _mtx
		//TODO: [CURRENTLLY NOT ACTUAL] write real algorithm for requestNewTargetLocation
		//......we get location by callback from topic listener see: callbackNewTargetLocation
    	SYNCH(SET_CURRENT_TIME(statistic.time_targetLocation_lastRequest));
    	//onNewTargetLocation(NEW_TARGET_LOCATION_GPS)
    }
    void callbackNewTargetLocation(const C23_ObjectRecognition::C23C0_ODIM::ConstPtr & msg){
    	std::string objectName = "";
    	{
    		PathPlanning::ReadSession session = _planner.startReading();
    		objectName = session.arguments.targetGoal;
    	}
    	if( objectName == msg->Object ){
    		onNewTargetLocation( extractObjectLocation( *msg ) );
    	}
	}
    void callbackTransitPoints(const C31_PathPlanner::C31_Waypoints::ConstPtr & msg){
		onNewTransitPoints( extractPoints( *msg ) );
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
    	if(session.arguments.targetDefined)
    		session.arguments.finish = _planner.cast(session.arguments.targetPosition);
    	session.constraints.dimentions.radius = _planner.castLength(session.constraints.dimentions.gps_radius);
    	session.constraints.transits = _planner.castToTransits(session.constraints.gps_transits);
		ROS_INFO("GPS_GRID_CASTING: start=*(%f,%f)->(%i,%i), finish=*(%f,%f)->(%i,%i), robot.R=*%f->%i (from onNewMap)",
			(float) session.arguments.selfLocation.x, (float) session.arguments.selfLocation.y, (int) session.arguments.start.x, (int) session.arguments.start.y,
			(float) session.arguments.targetPosition.x,(float)  session.arguments.targetPosition.y, (int) session.arguments.finish.x,(int) session.arguments.finish.y,
			(float) session.constraints.dimentions.gps_radius, (int) session.constraints.dimentions.radius
		);
		ROS_INFO("GPS_GRID_CASTING: Transits: (from onNewMap)");
		for(size_t i=0;i<session.constraints.transits.size();i++){
			ROS_INFO("... gps(%f,%f) -> wp(%i,%i)",
				(float) session.constraints.gps_transits[i].x, (float)session.constraints.gps_transits[i].y,
				(int) session.constraints.transits[i].x, (int) session.constraints.transits[i].y
			);
		}
    }
    void onNewLocation(const GPSPoint& pos, const Waypoint& wp){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = wp;
    	session.arguments.selfLocation = pos;
		ROS_INFO("GPS_GRID_CASTING: start=#(%f,%f)->#(%i,%i), finish=(%f,%f)->(%i,%i), robot.R=%f->%i (from onNewLocation(pos=wp))",
			(float) session.arguments.selfLocation.x, (float) session.arguments.selfLocation.y, (int) session.arguments.start.x, (int) session.arguments.start.y,
			(float) session.arguments.targetPosition.x,(float)  session.arguments.targetPosition.y, (int) session.arguments.finish.x,(int) session.arguments.finish.y,
			(float) session.constraints.dimentions.gps_radius, (int) session.constraints.dimentions.radius
		);
    }
    void onNewTargetLocation(const GPSPoint& pos){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.defineTarget( pos );
    	if(session.arguments.targetDefined)
    		session.arguments.finish = _planner.cast(session.arguments.targetPosition);
		ROS_INFO("GPS_GRID_CASTING: start=(%f,%f)->(%i,%i), finish=#(%f,%f)->(%i,%i), robot.R=%f->%i (from onNewTargetLocation)",
			(float) session.arguments.selfLocation.x, (float) session.arguments.selfLocation.y, (int) session.arguments.start.x, (int) session.arguments.start.y,
			(float) session.arguments.targetPosition.x,(float)  session.arguments.targetPosition.y, (int) session.arguments.finish.x,(int) session.arguments.finish.y,
			(float) session.constraints.dimentions.gps_radius, (int) session.constraints.dimentions.radius
		);
    }
    void onNewLocation(const GPSPoint& pos){
    	PathPlanning::EditSession session = _planner.startEdit();
    	session.arguments.start = _planner.cast(pos);
    	session.arguments.selfLocation = pos;
		ROS_INFO("GPS_GRID_CASTING: start=#(%f,%f)->(%i,%i), finish=(%f,%f)->(%i,%i), robot.R=%f->%i (from onNewLocation(pos=?))",
			(float) session.arguments.selfLocation.x, (float) session.arguments.selfLocation.y, (int) session.arguments.start.x, (int) session.arguments.start.y,
			(float) session.arguments.targetPosition.x,(float)  session.arguments.targetPosition.y, (int) session.arguments.finish.x,(int) session.arguments.finish.y,
			(float) session.constraints.dimentions.gps_radius, (int) session.constraints.dimentions.radius
		);
    }
    void onNewTransitPoints(const std::vector<GPSPoint> points){
    	PathPlanning::EditSession session = _planner.startEdit();
   		session.constraints.gps_transits = points;
    	session.constraints.transits = _planner.castToTransits(session.constraints.gps_transits);
		ROS_INFO("GPS_GRID_CASTING: Transits: (from NewTransitPoints)");
		for(size_t i=0;i<session.constraints.transits.size();i++){
			ROS_INFO("... gps(%f,%f) -> wp(%i,%i)",
				(float) session.constraints.gps_transits[i].x, (float)session.constraints.gps_transits[i].y,
				(int) session.constraints.transits[i].x, (int) session.constraints.transits[i].y
			);
		}
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

			//TODO: this check may be removed, but need testing after removing.
			if(_planner.isMapReady()==false){
				session.aborted();
				return TaskResult(FAULT, "Map is not ready");
			}

			session.arguments.defineTarget(TargetPosition(x,y));
			if(session.arguments.targetDefined)
				session.arguments.finish = _planner.cast(session.arguments.targetPosition);

			stringstream info;
			info<<"CONVERT "
					<<session.arguments.targetPosition.x<<","<<session.arguments.targetPosition.y
					<<" -> "
					<<session.arguments.finish.x<<","<<session.arguments.finish.y<<endl;
			ROS_INFO(info.str().c_str());

			return TaskResult(SUCCESS, "OK");
		}else
		if( args.find("target")!=args.end() ){
			std::string objectName = args["target"];

			ROS_INFO("%s: set planning goal to target = %s", _name.c_str(), objectName.c_str());

			PathPlanning::EditSession session = _planner.startEdit();

//			if(_planner.isMapReady()==false){
//				session.aborted();
//				return TaskResult(FAULT, "Map is not ready");
//			}

			session.arguments.defineTarget( objectName );
			if(session.arguments.targetDefined)
				session.arguments.finish = _planner.cast(session.arguments.targetPosition);

			return TaskResult(SUCCESS, "OK");
		}else{
			string desc = "I don't know to set path planner goal from current parameters ";
			ROS_INFO("%s: ERROR: %s", _name.c_str(), desc.c_str());

			return TaskResult(FAULT, desc);
		}

        return TaskResult::FAULT();
    }

};

class Task_whileSolution:public RobilTask{
	PathPlanning& _planner;
public:
	Task_whileSolution(PathPlanning& planner, string name = "/whileSolution"):
    	RobilTask(name), _planner(planner)
    {  }

    TaskResult task(const string& name, const string& uid, Arguments& args){
    	Events::Queue events(_planner.events());
    	while(true){
    		if(isPreempt()){

    			return TaskResult::Preempted();
    		}
    		if(events.empty() == false){
    			Event e = events.pop();
    			if(e.type == Event::TYPE_NOSOLUTIONFORPLAN){
    				return TaskResult(RobilTask::FAULT, e.description);
    			}
    		}
    		sleep(100); //millisec
    	}
        return TaskResult::FAULT();
    }
};

#endif //_PATH_PLANNING_SERVER_H_
