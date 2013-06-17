#include "PathPlanner.h"
#include "PField.h"

#include <C22_transformations/MapTransformations.h>

#include "cogniteam_pathplanner_parameters.h"

typedef World Map;

// copy from HEADER
#define SYNCHRONIZED boost::mutex::scoped_lock l(_mtx);
#define LOCK( X ) boost::shared_ptr<boost::mutex::scoped_lock> X(new boost::mutex::scoped_lock(_mtx));
#define UNLOCK( X ) X = boost::shared_ptr<boost::mutex::scoped_lock>();

	bool PathPlanning::plan(){
		bool plan_created = false;
		LOCK( locker_bfr )
			PlanningInputData _data(data);
		UNLOCK( locker_bfr )

		if(isMapReady()){
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
			
			ObsMap printed_map = _data.map.walls;;
			SmoothedPath _path;

#ifdef USE_TRANSITS_AS_PLAN
			if(_data.transits.size()>0){
#else
			if(false){
#endif
				ROS_INFO("WARNING: PLANNING RESULT IS TRANSIT POINTS AS IS.");
				printed_map = _data.map.walls;
				for(size_t i=0; i<_data.transits.size(); i++){
					_path.push_back(Vec2d(_data.transits[i].x,_data.transits[i].y));
				}
			}else{
#if MAP_MODE == MM_ALTS
				_data.map.walls(_data.start.x, _data.start.y) = ObsMap::ST_AVAILABLE;
				printed_map = _data.map.walls;
				_path = searchPath_transitAccurate(
						_data.map.altitudes, _data.map.slops, _data.map.costs, _data.map.walls, _data.map.grid, _data.map.terrain,
						_data.start, _data.finish, constraints, printed_map
				);
#else
				_data.map.grid(_data.start.x, _data.start.y) = ObsMap::ST_AVAILABLE;
				printed_map = _data.map.grid;
				_path = searchPath_transitAccurate(_data.map.grid, _data.start, _data.finish, constraints);
#endif
			}
			ROS_INFO("Calculated path (%i): %s",(int)(_path.size()),STR(_path));
			
			PField pf(printed_map, Path());
			ObsMap& map = printed_map;
			Path spath = pf.castPath(_path);
			for( size_t wp=0;wp<spath.size();wp++){
				map(spath[wp].x,spath[wp].y)='o';
			}
			cout<<map<<endl;

			if(_data.transits.size()>0){
				cout<<"transits: ";
				for(size_t t=0;t<_data.transits.size();t++){
					cout<<"("<<_data.transits[t].x<<","<<_data.transits[t].y<<") ";
				}
				cout<<endl;
			}

#if MAP_MODE == MM_ALTS
//			ROS_INFO("Altitudes:");
//			cout<<_data.map.altitudes<<endl;
//			ROS_INFO("Slops:");
//			cout<<_data.map.slops<<endl;
//			ROS_INFO("Costs:");
//			cout<<_data.map.costs<<endl;
#endif

			LOCK( locker_aft )
				path = _path;
				plan_created = path.size()>0;
				if(plan_created && targetDirPosition.defined){
					ROS_INFO("PathPlanning::plan : target dir point appended to path");
					path.push_back(finishDir);
				}
			UNLOCK( locker_aft )
		}else{
			ROS_INFO("PathPlanning::plan : %s","map is not ready => can not calculate path");
		}
		if(plan_created) ROS_INFO("PathPlanning::plan : %s","path successfully calculated");
		else ROS_INFO("PathPlanning::plan : %s","no path calculated");
		return plan_created;
	}

	#define TRANSLATE_GPS_TO_GRID(x, v) ( mapProperties.anchor.x + round( ( v - mapProperties.gps.x) / mapProperties.resolution ) )
	#define TRANSLATE_GRID_TO_GPS(x, v) ( mapProperties.gps.x +  (long)((long)v - (long)mapProperties.anchor.x) * mapProperties.resolution  )
	#define TRANSLATE_F_GRID_TO_GPS(x, v) ( mapProperties.gps.x +  (double)((double)v - (double)mapProperties.anchor.x) * mapProperties.resolution  )
	#define TRANSLATE_POINT_GPS_TO_GRID(x) TRANSLATE_GPS_TO_GRID(x, gps.x)
	#define TRANSLATE_POINT_GRID_TO_GPS(x) TRANSLATE_GRID_TO_GPS(x,  wp.x)
	#define TRANSLATE_POINT_F_GRID_TO_GPS(x) TRANSLATE_F_GRID_TO_GPS(x,  wp.x)
	#define CREATE_TRANSFORMATION(trans) C22_transform::MapProperties trans_##_prop(Vec2d(mapProperties.gps.x, mapProperties.gps.y)); C22_transform trans(trans_##_prop);

	long PathPlanning::cast(double gps)const{
		if(isMapReady()==false){
			return 0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d v(gps,0);
		Vec2d res;
		trans.GlobalToMap(v, res);
		return round(res.x);
	}
	long PathPlanning::castLength(double gps)const{
		if(isMapReady()==false){
			return 0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d z, v(gps,0);
		Vec2d tz, tv;
		trans.GlobalToMap(v, tv);
		trans.GlobalToMap(z, tz);
		return round((tv-tz).len());
	}

	double PathPlanning::cast(long cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d o(cell,0);
		Vec2d t;
		trans.MapToGlobal(o,t);
		return t.x;
	}
	double PathPlanning::castLength(long cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d z, v(cell,0);
		Vec2d tz, tv;
		trans.MapToGlobal(v, tv);
		trans.MapToGlobal(z, tz);
		return (tv-tz).len();
	}
	Waypoint PathPlanning::cast(const GPSPoint& gps)const{

		if(isMapReady()==false){
			return Waypoint(0,0);
		}

		CREATE_TRANSFORMATION(trans)
		Vec2d t, v(gps.x, gps.y);
		trans.GlobalToMap(v, t);
		long x ( round(t.x) );
		long y ( round(t.y) );

		if(data.map.inRange(x, y)==false){
			ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
			data.map.approximate(arguments.start.x, arguments.start.y, x, y);
		}

		Waypoint wp((size_t)x, (size_t)y);
		if(gps.defined==false) wp.undef();
		return wp;
	}
	Vec2d PathPlanning::castWP(const GPSPoint& gps)const{

		if(isMapReady()==false){
			return Vec2d(0,0);
		}

		CREATE_TRANSFORMATION(trans)
		Vec2d t, v(gps.x, gps.y);
		trans.GlobalToMap(v, t);
		long x ( round(t.x) );
		long y ( round(t.y) );

		if(data.map.inRange(x, y)==false){
			ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
			data.map.approximate(arguments.start.x, arguments.start.y, x, y);
			return Vec2d(x,y);
		}

		return Vec2d(t.x, t.y);
	}
	GPSPoint PathPlanning::cast(const Waypoint& wp)const{

		if(isMapReady()==false){
			return GPSPoint(0,0);
		}

		CREATE_TRANSFORMATION(trans)
		Vec2d t, v(wp.x, wp.y);
		trans.MapToGlobal(v, t);
		double x ( t.x );
		double y ( t.y );

		GPSPoint gps(x, y);
		if(wp.defined==false) gps.undef();
		return gps;
	}

	double PathPlanning::castWP(double cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d o(cell,0);
		Vec2d t;
		trans.MapToGlobal(o,t);
		return t.x;
	}
	double PathPlanning::castWPLength(double cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		CREATE_TRANSFORMATION(trans)
		Vec2d z, v(cell,0);
		Vec2d tz, tv;
		trans.MapToGlobal(v, tv);
		trans.MapToGlobal(z, tz);
		return (tv-tz).len();
	}
	GPSPoint PathPlanning::cast(const Vec2d& wp)const{

		if(isMapReady()==false){
			return GPSPoint(0,0);
		}

		CREATE_TRANSFORMATION(trans)
		Vec2d t, v(wp.x, wp.y);
		trans.MapToGlobal(v, t);
		double x ( t.x );
		double y ( t.y );


		return GPSPoint(x, y);

	}

	std::vector<Waypoint> PathPlanning::cast(const std::vector<GPSPoint>& gps_vector)const{

		if(isMapReady()==false){
			return std::vector<Waypoint>();
		}

		std::vector<Waypoint> wp_vector;
		for(size_t i=0; i<gps_vector.size(); i++){
			//GPSPoint gps( gps_vector[i].x, gps_vector[i].y);
			CREATE_TRANSFORMATION(trans)
			Vec2d t, v(gps_vector[i].x, gps_vector[i].y);
			trans.GlobalToMap(v, t);
			long x ( round(t.x) );
			long y ( round(t.y) );


			if(data.map.inRange(x, y)==false){
				ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
				data.map.approximate(arguments.start.x, arguments.start.y, x, y);
			}
			Waypoint wp((size_t)x, (size_t)y);
			if(gps_vector[i].defined==false) wp.undef();
			wp_vector.push_back( Waypoint((size_t)x, (size_t)y) );

		}

		return wp_vector;

	}
	std::vector<TransitWaypoint> PathPlanning::castToTransits(const std::vector<GPSPoint>& gps_vector)const{
		std::vector<Waypoint> wp_vector = cast(gps_vector);
		Transits transits;
		for(size_t i=0;i<wp_vector.size();i++){
			TransitWaypoint twp;
			twp.x=wp_vector[i].x;
			twp.y=wp_vector[i].y;
			transits.push_back(twp);
		}
		return transits;
	}


//	long PathPlanning::cast(double gps)const{
//		if(isMapReady()==false){
//			return 0;
//		}
//		long cell = TRANSLATE_GPS_TO_GRID(x, gps);
//		return cell;
//	}
//	long PathPlanning::castLength(double gps)const{
//		if(isMapReady()==false){
//			return 0;
//		}
//		long cell = TRANSLATE_GPS_TO_GRID(x, gps+mapProperties.gps.x) - mapProperties.anchor.x;
//		return cell;
//	}
//	double PathPlanning::cast(long cell)const{
//		if(isMapReady()==false){
//			return 0.0;
//		}
//		double gps = TRANSLATE_GRID_TO_GPS(x, cell);
//		return gps;
//	}
//	double PathPlanning::castLength(long cell)const{
//		if(isMapReady()==false){
//			return 0.0;
//		}
//		double gps = TRANSLATE_GRID_TO_GPS(x+mapProperties.anchor.x, cell) - mapProperties.gps.x;
//		return gps;
//	}
//	Waypoint PathPlanning::cast(const GPSPoint& gps)const{
//		#define TRANSLATE(x) TRANSLATE_POINT_GPS_TO_GRID(x)
//
//		if(isMapReady()==false){
//			return Waypoint(0,0);
//		}
//
//		long x ( TRANSLATE(x) );
//		long y ( TRANSLATE(y) );
//
//		if(data.map.inRange(x, y)==false){
//			ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
//			data.map.approximate(arguments.start.x, arguments.start.y, x, y);
//		}
//
//		return Waypoint((size_t)x, (size_t)y);
//
//		#undef TRANSLATE
//	}
//	GPSPoint PathPlanning::cast(const Waypoint& wp)const{
//		#define TRANSLATE(x) TRANSLATE_POINT_GRID_TO_GPS(x)
//
//		if(isMapReady()==false){
//			return GPSPoint(0,0);
//		}
//
//		double x ( TRANSLATE(x) );
//		double y ( TRANSLATE(y) );
//
//		return GPSPoint(x, y);
//
//		#undef TRANSLATE
//	}
//
//	double PathPlanning::castWP(double cell)const{
//		if(isMapReady()==false){
//			return 0.0;
//		}
//		double gps = TRANSLATE_F_GRID_TO_GPS(x, cell);
//		return gps;
//	}
//	double PathPlanning::castWPLength(double cell)const{
//		if(isMapReady()==false){
//			return 0.0;
//		}
//		double gps = TRANSLATE_F_GRID_TO_GPS(x+mapProperties.anchor.x, cell) - mapProperties.gps.x;
//		return gps;
//	}
//	GPSPoint PathPlanning::cast(const Vec2d& wp)const{
//		#define TRANSLATE(x) TRANSLATE_POINT_F_GRID_TO_GPS(x)
//
//		if(isMapReady()==false){
//			return GPSPoint(0,0);
//		}
//
//		double x ( TRANSLATE(x) );
//		double y ( TRANSLATE(y) );
//
//		return GPSPoint(x, y);
//
//		#undef TRANSLATE
//	}
//
//	std::vector<Waypoint> PathPlanning::cast(const std::vector<GPSPoint>& gps_vector)const{
//		#define TRANSLATE(x) TRANSLATE_POINT_GPS_TO_GRID(x)
//
//		if(isMapReady()==false){
//			return std::vector<Waypoint>();
//		}
//
//		std::vector<Waypoint> wp_vector;
//		for(size_t i=0; i<gps_vector.size(); i++){
//			GPSPoint gps( gps_vector[i].x, gps_vector[i].y);
//			long x ( TRANSLATE(x) );
//			long y ( TRANSLATE(y) );
//
//			if(data.map.inRange(x, y)==false){
//				ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
//				data.map.approximate(arguments.start.x, arguments.start.y, x, y);
//			}
//
//			wp_vector.push_back( Waypoint((size_t)x, (size_t)y) );
//
//		}
//
//		return wp_vector;
//
//		#undef TRANSLATE
//	}
//	std::vector<TransitWaypoint> PathPlanning::castToTransits(const std::vector<GPSPoint>& gps_vector)const{
//		std::vector<Waypoint> wp_vector = cast(gps_vector);
//		Transits transits;
//		for(size_t i=0;i<wp_vector.size();i++){ TransitWaypoint twp; twp.x=wp_vector[i].x; twp.y=wp_vector[i].y; transits.push_back(twp); }
//		return transits;
//	}


	
	
	#undef TRANSLATE_POINT_GPS_TO_GRID
	#undef TRANSLATE_POINT_GRID_TO_GPS
	#undef TRANSLATE_GPS_TO_GRID
	#undef TRANSLATE_GRID_TO_GPS	
	#undef CREATE_TRANSFORMATION
	
