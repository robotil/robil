#ifndef _COGNITEAM_PATHPLANNING_H_
#define _COGNITEAM_PATHPLANNING_H_

#include <vector>
#include "Map.h"
#include "Vec2d.hpp"

using namespace std;

int cogniteam_pathplanning_test(int, char**);
int cogniteam_pathplanning_test_map_inflation(int argc, char** argv) ;
int cogniteam_pathplanning_test_transits(int argc, char** argv) ;
int cogniteam_pathplanning_test_alts(int argc, char** argv) ;

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


#endif
