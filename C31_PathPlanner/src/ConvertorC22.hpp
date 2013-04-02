#ifndef _CONVERTER_C22_H_
#define _CONVERTER_C22_H_

#include "C22_GroundRecognitionAndMapping/C22.h"
#include "C22_GroundRecognitionAndMapping/C22C0_PATH.h"
#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


static Map extractMap(const C22_GroundRecognitionAndMapping::C22C0_PATH &res, const MapProperties& prop){
	Map::MapCreator m;
	size_t h = res.row.size();
	bool size_ok = false;
	if(h){
		size_t w = res.row.at(0).column.size();
		if(w){

			ROS_INFO(STR("start converting message to map ("<<w<<"x"<<h<<")"));

			size_ok = true;
			m.w=w; m.h=h;
			for(size_t y=0;y<h;y++)
				for(size_t x=0;x<w;x++)
					m.data.push_back(res.row.at(y).column.at(x).status);

			//std::cout<<"finish converting message to map ("<<w<<"x"<<h<<")\n"<<m.map()<<std::endl;
			ROS_INFO("... new data converted");
		}
	}
	if(size_ok==false){
		std::cout<<"size of map is wrong"<<std::endl;
		return Map(0,0);
	}
	return m.map();
};


static Gps2Grid extractLocation(const C22_GroundRecognitionAndMapping::C22C0_PATH &res, const MapProperties& prop){
	GPSPoint gps(0,0);
	Waypoint wp(0,0);

	size_t h = res.row.size();
	if(h){

		ROS_INFO("start converting message to location : pos=%f,%f   map offset=%f,%f",
				 (float) res.robotPos.x, (float) res.robotPos.y, (float) res.xOffset, (float) res.yOffset);
		//size_t w = res.drivingPath.row.at(0).column.size();

		wp.x=( res.robotPos.x - res.xOffset )/ prop.resolution;
		wp.y=( res.robotPos.y - res.yOffset )/ prop.resolution;

		gps.x = res.robotPos.x;
		gps.y = res.robotPos.y;
		gps.heaidng = res.robotOri.z;

		ROS_INFO("... new data converted : gps point = %f,%f  related to  grid cell = %i,%i", (float) gps.x, (float) gps.y, (int) wp.x, (int) wp.y);
	}
	return Gps2Grid(gps,wp);
};

static MapProperties extractMapProperties(const C22_GroundRecognitionAndMapping::C22C0_PATH &res){
	GPSPoint gps(0,0);
	Waypoint wp(0,0);
	double cell_resolution_in_meters = 0;

	size_t h = res.row.size();
	if(h){
		ROS_INFO("start converting message to MapProperties : map offset=%f,%f", res.xOffset, res.yOffset);
		//size_t w = res.row.at(0).column.size();
		//wp.x = w/2;

		wp.y = 0;
		wp.x = 0;

		gps.x = res.xOffset;
		gps.y = res.yOffset;
		gps.heaidng = 0; // gotten map is a local window of global map with constant orientation to 0 (north)

		cell_resolution_in_meters = 0.25;

		ROS_INFO("... new data converted : new map anchor is ( cell = 0,0 gps = %f,%f )",  gps.x, gps.y);
	}
	return MapProperties(cell_resolution_in_meters, gps, wp);
};

//=================== from service ===============================

static Map extractMap(C22_GroundRecognitionAndMapping::C22::Response &res, const MapProperties& prop){
//	Map::MapCreator m;
//	size_t h = res.drivingPath.row.size();
//	bool size_ok = false;
//	if(h){
//		size_t w = res.drivingPath.row.at(0).column.size();
//		if(w){
//
//			ROS_INFO(STR("start converting message to map ("<<w<<"x"<<h<<")"));
//
//			size_ok = true;
//			m.w=w; m.h=h;
//			for(size_t y=0;y<h;y++)
//				for(size_t x=0;x<w;x++)
//					m.data.push_back(res.drivingPath.row.at(y).column.at(x).status);
//
//			//std::cout<<"finish converting message to map ("<<w<<"x"<<h<<")\n"<<m.map()<<std::endl;
//			ROS_INFO("... new data converted");
//		}
//	}
//	if(size_ok==false){
//		std::cout<<"size of map is wrong"<<std::endl;
//		return Map(0,0);
//	}
//	return m.map();
	return extractMap(res.drivingPath, prop);
};


static Gps2Grid extractLocation(C22_GroundRecognitionAndMapping::C22::Response &res, const MapProperties& prop){
//	GPSPoint gps(0,0);
//	Waypoint wp(0,0);
//
//	size_t h = res.drivingPath.row.size();
//	if(h){
//
//		ROS_INFO("start converting message to location : pos=%f,%f   map offset=%f,%f",
//				 res.drivingPath.robotPos.x, res.drivingPath.robotPos.y, res.drivingPath.xOffset, res.drivingPath.yOffset);
//		//size_t w = res.drivingPath.row.at(0).column.size();
//
//		wp.x=( res.drivingPath.robotPos.x - res.drivingPath.xOffset )/ prop.resolution;
//		wp.y=( res.drivingPath.robotPos.y - res.drivingPath.yOffset )/ prop.resolution;
//
//		gps.x = res.drivingPath.robotPos.x;
//		gps.y = res.drivingPath.robotPos.y;
//
//		ROS_INFO("... new data converted : gps point = %f,%f  related to  grid cell = %i,%i",  gps.x, gps.y,  wp.x, wp.y);
//	}
//	return Gps2Grid(gps,wp);
	extractLocation(res.drivingPath, prop);
};

static MapProperties extractMapProperties(C22_GroundRecognitionAndMapping::C22::Response &res){
//	GPSPoint gps(0,0);
//	Waypoint wp(0,0);
//	double cell_resolution_in_meters = 0;
//
//	size_t h = res.drivingPath.row.size();
//	if(h){
//		ROS_INFO("start converting message to MapProperties : map offset=%f,%f", res.drivingPath.xOffset, res.drivingPath.yOffset);
//		//size_t w = res.drivingPath.row.at(0).column.size();
//		//wp.x = w/2;
//
//		wp.y = 0;
//		wp.x = 0;
//
//		gps.x = res.drivingPath.xOffset;
//		gps.y = res.drivingPath.yOffset;
//
//		cell_resolution_in_meters = 0.25;
//
//		ROS_INFO("... new data converted : new map anchor is ( cell = 0,0 gps = %f,%f )",  gps.x, gps.y);
//	}
//	return MapProperties(cell_resolution_in_meters, gps, wp);
	extractMapProperties(res.drivingPath);
};


#endif
