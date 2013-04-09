#include "PathPlanner.h"
#include "PField.h"

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
			
			_data.map(_data.start.x, _data.start.y) = Map::ST_AVAILABLE;
			SmoothedPath _path = searchPath(_data.map, _data.start, _data.finish, constraints);
			ROS_INFO("Calculated path: %s",STR(_path));
			
			PField pf(_data.map, Path());
			Map map = _data.map;
			Path spath = pf.castPath(_path);
			for( size_t wp=0;wp<spath.size();wp++){
				map(spath[wp].x,spath[wp].y)='o';
			}
			cout<<map<<endl;

			LOCK( locker_aft )
				path = _path;
				plan_created = path.size()>0;
			UNLOCK( locker_aft )
		}else{
			ROS_INFO("PathPlanning::plan : %s","map is not ready => can not calculate path");
		}
		ROS_INFO("PathPlanning::plan : %s","no path calculated");
		return plan_created;
	}

	#define TRANSLATE_GPS_TO_GRID(x, v) ( mapProperties.anchor.x + round( ( v - mapProperties.gps.x) / mapProperties.resolution ) )
	#define TRANSLATE_GRID_TO_GPS(x, v) ( mapProperties.gps.x +  (long)((long)v - (long)mapProperties.anchor.x) * mapProperties.resolution  )
	#define TRANSLATE_F_GRID_TO_GPS(x, v) ( mapProperties.gps.x +  (double)((double)v - (double)mapProperties.anchor.x) * mapProperties.resolution  )
	#define TRANSLATE_POINT_GPS_TO_GRID(x) TRANSLATE_GPS_TO_GRID(x, gps.x)
	#define TRANSLATE_POINT_GRID_TO_GPS(x) TRANSLATE_GRID_TO_GPS(x,  wp.x)
	#define TRANSLATE_POINT_F_GRID_TO_GPS(x) TRANSLATE_F_GRID_TO_GPS(x,  wp.x)

	long PathPlanning::cast(double gps)const{
		if(isMapReady()==false){
			return 0;
		}
		long cell = TRANSLATE_GPS_TO_GRID(x, gps);
		return cell;
	}
	long PathPlanning::castLength(double gps)const{
		if(isMapReady()==false){
			return 0;
		}
		long cell = TRANSLATE_GPS_TO_GRID(x, gps+mapProperties.gps.x) - mapProperties.anchor.x;
		return cell;
	}
	double PathPlanning::cast(long cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		double gps = TRANSLATE_GRID_TO_GPS(x, cell);
		return gps;
	}
	double PathPlanning::castLength(long cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		double gps = TRANSLATE_GRID_TO_GPS(x+mapProperties.anchor.x, cell) - mapProperties.gps.x;
		return gps;
	}
	Waypoint PathPlanning::cast(const GPSPoint& gps)const{
		#define TRANSLATE(x) TRANSLATE_POINT_GPS_TO_GRID(x)

		if(isMapReady()==false){
			return Waypoint(0,0);
		}

		long x ( TRANSLATE(x) );
		long y ( TRANSLATE(y) );

		if(data.map.inRange(x, y)==false){
			ROS_INFO(STR("x or y not in map range: "<<x<<","<<y));
			data.map.approximate(arguments.start.x, arguments.start.y, x, y);
		}

		return Waypoint((size_t)x, (size_t)y);

		#undef TRANSLATE
	}
	GPSPoint PathPlanning::cast(const Waypoint& wp)const{
		#define TRANSLATE(x) TRANSLATE_POINT_GRID_TO_GPS(x)

		if(isMapReady()==false){
			return GPSPoint(0,0);
		}

		double x ( TRANSLATE(x) );
		double y ( TRANSLATE(y) );

		return GPSPoint(x, y);

		#undef TRANSLATE
	}

	double PathPlanning::castWP(double cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		double gps = TRANSLATE_F_GRID_TO_GPS(x, cell);
		return gps;
	}
	double PathPlanning::castWPLength(double cell)const{
		if(isMapReady()==false){
			return 0.0;
		}
		double gps = TRANSLATE_F_GRID_TO_GPS(x+mapProperties.anchor.x, cell) - mapProperties.gps.x;
		return gps;
	}
	GPSPoint PathPlanning::cast(const Vec2d& wp)const{
		#define TRANSLATE(x) TRANSLATE_POINT_F_GRID_TO_GPS(x)

		if(isMapReady()==false){
			return GPSPoint(0,0);
		}

		double x ( TRANSLATE(x) );
		double y ( TRANSLATE(y) );

		return GPSPoint(x, y);

		#undef TRANSLATE
	}


	
	
	#undef TRANSLATE_POINT_GPS_TO_GRID
	#undef TRANSLATE_POINT_GRID_TO_GPS
	#undef TRANSLATE_GPS_TO_GRID
	#undef TRANSLATE_GRID_TO_GPS	
	
	
