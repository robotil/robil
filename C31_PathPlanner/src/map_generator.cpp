#include <ros/ros.h>
#include <sstream>
#include <iostream>

#include "C22_GroundRecognitionAndMapping/C22.h"
#include "Map.h"
#include "MapFileReader.hpp"

typedef ObsMap Map;

struct robotPos_t{
	double x, y, z;
	robotPos_t(double x, double y, double z):x(x),y(y),z(z){}
} robotPos(0,0,0);
struct robotOri_t{
	double x, y, z;
	robotOri_t(double x, double y, double z):x(x),y(y),z(z){}
} robotOri(0,0,0);
struct Offset_t{
	double xOffset, yOffset;
	Offset_t(double x, double y):xOffset(x),yOffset(y){}
} offset(0,0);



Map *lmap = NULL;

bool proccess(C22_GroundRecognitionAndMapping::C22::Request  &req, C22_GroundRecognitionAndMapping::C22::Response &res ){
	ROS_INFO("Map request recived.");
	
	if(!lmap){
		ROS_INFO("WARNING: Map is not loaded from file.");
		return true;
	}
	
	
	res.drivingPath.robotPos.x=robotPos.x;
	res.drivingPath.robotPos.y=robotPos.y;
	res.drivingPath.robotPos.z=robotPos.z;
// 	res.drivingPath.robotOri.x=robotOri.x;
// 	res.drivingPath.robotOri.y=robotOri.y;
// 	res.drivingPath.robotOri.z=robotOri.z;
	res.drivingPath.xOffset=offset.xOffset;
	res.drivingPath.yOffset=offset.yOffset;
	
	int h = lmap->h();
	int w = lmap->w();
	
	res.drivingPath.row.resize(h);
	for(int i=0;i<h;i++){
		res.drivingPath.row.at(i).column.resize(w);
		for(int j=0;j<w;j++){
			res.drivingPath.row.at(i).column.at(j).status=(*lmap)(j,i);
			/*
			res.drivingPath.row.at(i).column.at(j).planes.resize(_myMatrix->data->at(i)->at(j)->square_Planes->size());
			for(int k=0;k<_myMatrix->data->at(i)->at(j)->square_Planes->size();k++){
				res.drivingPath.row.at(i).column.at(j).planes.at(k).x=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_x;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).y=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_y;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).z=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_z;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).d=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_d;
			}
			*/
		}
	}
	
	ROS_INFO("Map message is generated.");
	return true;
}

int main(int argc, char** argv){
	ROS_INFO("Start node map_generator");
	
	ros::init(argc, argv, "map_generator");
	ros::NodeHandle node;
	
	vector<char> map_from_file;
	char* cmap = 0;
	size_t w=0,h=0;
	if(argc>1){
		ROS_INFO("map from file: %s", argv[1]);
		map_from_file = map_file_reader::readMap(argv[1], w, h);
		cmap = map_from_file.data();
	}
	
	Map map(w, h, cmap);
	if(cmap) lmap = &map;
	

	ros::ServiceServer C22 =
	node.advertiseService("C22", &proccess);

	ros::spin();
	
	ROS_INFO("Stop node map_generator");
	return 0;
}
