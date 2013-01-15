#ifndef _CONVERTER_C22_H_
#define _CONVERTER_C22_H_

#include "C22_GroundRecognitionAndMapping/C22.h"
#include "C0_RobilTask/RobilTask.h"
#include "Map.h"
#include "Gps.h"


static Map extractMap(C22_GroundRecognitionAndMapping::C22::Response &res){
	Map::MapCreator m;
	size_t h = res.drivingPath.row.size();
	bool size_ok = false;
	if(h){
		size_t w = res.drivingPath.row.at(0).column.size();
		if(w){
			
			ROS_INFO(STR("start converting message to map ("<<w<<"x"<<h<<")"));
			
			size_ok = true;
			m.w=w; m.h=h;
			for(size_t y=0;y<h;y++)
				for(size_t x=0;x<w;x++)
					m.data.push_back(res.drivingPath.row.at(y).column.at(x).status);
				
			//std::cout<<"finish converting message to map ("<<w<<"x"<<h<<")\n"<<m.map()<<std::endl;
		}
	}
	if(size_ok==false){
		std::cout<<"size of map is wrong"<<std::endl;
		return Map(0,0);
	}
	return m.map();
};


static Gps2Grid extractLocation(C22_GroundRecognitionAndMapping::C22::Response &res){
	GPSPoint gps(0,0);
	Waypoint wp(0,0);

	//TODO: TEMPORAL CODE

	size_t h = res.drivingPath.row.size();
	if(h){
		
		ROS_INFO("start converting message to location");
		size_t w = res.drivingPath.row.at(0).column.size();
		wp.x = w/2;
	}
	return Gps2Grid(gps,wp);
};

static MapProperties extractMapProperties(C22_GroundRecognitionAndMapping::C22::Response &res){
	GPSPoint gps(0,0);
	Waypoint wp(0,0);

	//TODO: TEMPORAL CODE

	size_t h = res.drivingPath.row.size();
	if(h){
		ROS_INFO("start converting message to MapProperties");
		size_t w = res.drivingPath.row.at(0).column.size();
		wp.x = w/2;
	}
	return MapProperties(0.12, gps, wp);
};


#endif
