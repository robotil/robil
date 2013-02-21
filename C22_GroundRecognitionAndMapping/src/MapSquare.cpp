/*
 * MapSquare.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */
#include <vector>
#include <string.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include "MPlane.h"
#include "MapSquare.h"
MapSquare::MapSquare(){
	  square_status=UNCHARTED;
	  square_Planes=new std::vector<MPlane*>();
}

void MapSquare::clearSq(){
	while(square_Planes->size()!=0){
		MPlane* temp=square_Planes->back();
		square_Planes->pop_back();
		delete temp;
	}
	delete square_Planes;
	square_Planes=new std::vector<MPlane*>();
	square_status=UNCHARTED;
}

MapSquare::~MapSquare(){
	clearSq();
	delete square_Planes;
	//delete square_Planes;
}

bool MapSquare::hasPlane(MPlane* other){
	  for(unsigned int i=0; i<square_Planes->size();i++){
		  if(square_Planes->at(i)->isEqualTo(other))
			  return true;
	  }
	  return false;
}

std::string MapSquare::toString(){
	 std::string stats[]={"FLOOR","AVAILABLE","BLOCKED","UNCHARTED"};
	 std::stringstream ss;
	 ss<<"no. of planes: "<<square_Planes->size()
			 <<") , status: "<<stats[square_status];
	 return ss.str();
}
