#ifndef _CONVERTER_C23_H_
#define _CONVERTER_C23_H_

//#include "C23_ObjectRecognition/C23.h"
//#include <C23_ObjectRecognition/C23C0_OD.h>
//#include <C23_ObjectRecognition/C23C0_ODIM.h>
//#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include <C23_ObjectRecognition/C23C0_GP.h>

#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


//static GPSPoint extractObjectLocation(const C23_ObjectRecognition::C23C0_ODIM & msg ){
// 	GPSPoint gps(msg.x,msg.y);
//	//ODO: write real code for extractObjectLocation(C23_ObjectRecognition::C23C0_ODIM)
//	//GPSPoint gps(0,0);
//	return gps;
//};
static std::vector<GPSPoint> extractObjectLocation(const C23_ObjectRecognition::C23C0_GP & msg ){
	std::vector<GPSPoint> pp;
 	GPSPoint gps(msg.x,msg.y);
 	GPSPoint gps2(msg.x2,msg.y2);
	//ODO: write real code for extractObjectLocation(C23_ObjectRecognition::C23C0_ODIM)
	//GPSPoint gps(0,0);
 	if( fabs(msg.x-msg.x2) < 0.01 && fabs(msg.y-msg.y2) < 0.01 ) gps2.undef();
 	pp.push_back(gps);
 	pp.push_back(gps2);
	return pp;
};
//static bool isObjectDetected(const C23_ObjectRecognition::C23C0_OD & msg ){
//	return msg.ObjectDetected == 1;
//};

//struct C23Data{
//	int type;
////	C23_ObjectRecognition::C23C0_ODIM odim;
////	C23_ObjectRecognition::C23C0_OD od;
//	C23_ObjectRecognition::C23C0_GP gp;
//
////	C23Data(const C23_ObjectRecognition::C23C0_ODIM & msg1, const C23_ObjectRecognition::C23C0_OD & msg2)
////	:type(0),odim(msg1),od(msg2){}
//	C23Data(const C23_ObjectRecognition::C23C0_GP & mgp)
//	:type(1),gp(mgp){}
////	void set(const C23_ObjectRecognition::ccIM & msg1){odim=msg1;}
////	void set(const C23_ObjectRecognition::C23C0_OD & msg2){od=msg2;}
//	void set(const C23_ObjectRecognition::C23C0_GP & msg){gp=msg;}
//	GPSPoint location()const{
////		if(type==0)
////			return extractObjectLocation(odim);
//		return extractObjectLocation(gp)[0];
//	}
//	GPSPoint direction()const{
//		if(type==0)
//			return GPSPoint(0,0);
//		return extractObjectLocation(gp)[1];
//	}
////	bool isDetected()const{
////		return isObjectDetected(od);
////	}
//};


#endif
