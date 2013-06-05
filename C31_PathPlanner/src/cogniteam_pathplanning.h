#ifndef _COGNITEAM_PATHPLANNING_H_
#define _COGNITEAM_PATHPLANNING_H_

#include <vector>
#include "Map.h"
#include "Vec2d.hpp"

using namespace std;

int cogniteam_pathplanning_test(int, char**);
int cogniteam_pathplanning_test_map_inflation(int argc, char** argv) ;
int cogniteam_pathplanning_test_transits(int argc, char** argv) ;
int cogniteam_pathplanning_test_alts(int argc, char** argv, int planning_code=3) ;

#define Map ObsMap

struct TransitWaypoint{
	size_t x; size_t y;
};
typedef vector<TransitWaypoint> Transits;

struct Attractor{
	size_t x; size_t y;
	int type;
};
typedef vector<Attractor> Attractors;

struct Waypoint{
	size_t x; size_t y;
	Waypoint(size_t x=0, size_t y=0):x(x),y(y){}
};
typedef vector<Waypoint> Waypoints;
typedef Waypoints Path;
typedef vector<Vec2d> SmoothedPath;

template <class A>
void append(std::vector<A>& target, const std::vector<A>& source){
	for(size_t i=0;i<source.size();i++){
		if(target.size()>0){
			const A* t = &(target.back());
			const A* s = &(source[i]);
			bool same_points = memcmp((const char*)t, (const char*)s, sizeof(A))==0;
			if(same_points){
				//std::cout<<"SAME POINTS. REMOVE ONE : "<<target.back().x<<","<<target.back().y<<" --- "<<source[i].x<<","<<source[i].y<<std::endl;
				continue;
			}
		}
		target.push_back(source[i]);
	}
}

inline std::ostream& operator<<(std::ostream& o, const Waypoint& w){
	return o<<"("<<w.x<<","<<w.y<<")";
}
inline std::ostream& operator<<(std::ostream& o, const Path& w){
	o<<"Path#"<<w.size()<<"{"; for(size_t i=0;i<w.size();i++)o<<" "<<w[i]; o<<" }";
	return o;
}
inline std::ostream& operator<<(std::ostream& o, const SmoothedPath& w){
	o<<"Path#"<<w.size()<<"{"; for(size_t i=0;i<w.size();i++)o<<" "<<w[i]; o<<" }";
	return o;
}

struct RobotDimentions{
	size_t radius;
	double gps_radius;
	RobotDimentions(double gps_rad=0.40):radius(1),gps_radius(gps_rad){}
};

struct Constraints{
	const RobotDimentions& dimentions;
	const Transits& transits;
	const Attractors& attractors;
	Constraints(const RobotDimentions& dimentions, const Transits& transits, const Attractors& attractors)
	:dimentions(dimentions),transits(transits),attractors(attractors){}
};

SmoothedPath searchPath(const Map& map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints);
SmoothedPath searchPath_transitAccurate(const Map& map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints);
SmoothedPath searchPath(
		const AltMap& alts, const AltMap& slops, const AltMap& costs, const Map& s_walls,
		const Waypoint& start, const Waypoint& finish, const Constraints& constraints
);
SmoothedPath searchPath_transitAccurate(
		const AltMap& alts, const AltMap& slops, const AltMap& costs, const Map& s_walls, const Map& s_obstacles, const Map& s_terrain,
		const Waypoint& start, const Waypoint& finish, const Constraints& constraints
);

#undef Map


#define PRINT_VERSION std::cout<<"VERSION: "<<PLANNER_VERSION<<". ["<<__FUNCTION__<<"]"<<std::endl;
#endif
