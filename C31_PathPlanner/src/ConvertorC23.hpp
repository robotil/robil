#ifndef _CONVERTER_C23_H_
#define _CONVERTER_C23_H_

//#include "C23_ObjectRecognition/C23.h"
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>

#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


static GPSPoint extractObjectLocation(const C23_ObjectRecognition::C23C0_ODIM & msg ){
// 	GPSPoint gps(msg.x,msg.y);
	//TODO: write real code for extractObjectLocation(C23_ObjectRecognition::C23C0_ODIM)
	GPSPoint gps(0,0);
	return gps;
};
static bool isObjectDetected(const C23_ObjectRecognition::C23C0_OD & msg ){
	return msg.ObjectDetected == 1;
};

struct C23Data{
	C23_ObjectRecognition::C23C0_ODIM odim;
	C23_ObjectRecognition::C23C0_OD od;
	
	C23Data(const C23_ObjectRecognition::C23C0_ODIM & msg1, const C23_ObjectRecognition::C23C0_OD & msg2)
	:odim(msg1),od(msg2){}
	void set(const C23_ObjectRecognition::C23C0_ODIM & msg1){odim=msg1;}
	void set(const C23_ObjectRecognition::C23C0_OD & msg2){od=msg2;}
	GPSPoint location()const{
		return extractObjectLocation(odim);
	}
	bool isDetected()const{
		return isObjectDetected(od);
	}
};


#endif
