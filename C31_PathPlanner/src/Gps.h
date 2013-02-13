#ifndef ___GPS_POINTS_H_
#define ___GPS_POINTS_H_

#include "cogniteam_pathplanning.h"

class GPSPoint{
public:
	double x, y, heaidng;
	GPSPoint(double x, double y):x(x),y(y),heaidng(0){

	}
	GPSPoint(double x, double y, double h):x(x),y(y),heaidng(h){

	}
};

typedef std::vector<GPSPoint> GPSPath;

struct Gps2Grid{
	GPSPoint gps;
	Waypoint cell;
	Gps2Grid(GPSPoint gps, Waypoint cell):gps(gps),cell(cell){}
};

struct MapProperties{
	double resolution;
	GPSPoint gps;
	Waypoint anchor;
	MapProperties(double res, GPSPoint gps, Waypoint anchor):resolution(res),gps(gps),anchor(anchor){}
};

#endif
