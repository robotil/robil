#ifndef _COGNITEAM_PATHPLANNING_H_
#define _COGNITEAM_PATHPLANNING_H_

#include <vector>
#include "Map.h"

using namespace std;

int cogniteam_pathplanning_test(int, char**);
int cogniteam_pathplanning_test_map_inflation(int argc, char** argv) ;


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
	Waypoint(size_t x, size_t y):x(x),y(y){}
};
typedef vector<Waypoint> Waypoints;
typedef Waypoints Path;

struct RobotDimentions{
	size_t radius;
};

struct Constraints{
	const RobotDimentions& dimentions;
	const Transits& transits;
	const Attractors& attractors;
	Constraints(const RobotDimentions& dimentions, const Transits& transits, const Attractors& attractors)
	:dimentions(dimentions),transits(transits),attractors(attractors){}
};

Path searchPath(const Map& map, const Waypoint& start, const Waypoint& finish, const Constraints& constraints);



#endif
