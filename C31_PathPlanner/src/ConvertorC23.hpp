#ifndef _CONVERTER_C23_H_
#define _CONVERTER_C23_H_

#include "C23_ObjectRecognition/C23.h"
#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


static Gps2Grid extractObjectLocation(C23_ObjectRecognition::C23::Response &res){
	GPSPoint gps(0,0);
	Waypoint wp(0,0);

	//TODO : TEMPORAL CODE!!! WRITE REAL CODE
	
	return Gps2Grid(gps,wp);
};


#endif
