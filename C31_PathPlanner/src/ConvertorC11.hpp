#ifndef _CONVERTER_C11_H_
#define _CONVERTER_C11_H_


#include <C31_PathPlanner/C31_Waypoints.h>

#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


static std::vector<GPSPoint> extractPoints(const C31_PathPlanner::C31_Waypoints & msg ){
	std::vector<GPSPoint> points;
	for(size_t i=0; i<msg.points.size(); i++){
		GPSPoint gps(msg.points[i].x,msg.points[i].y);
		points.push_back(gps);
	}
	return points;
};



#endif
