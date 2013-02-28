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
	  rating=0;
	  ratable=true;
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

MPlane* MapSquare::getPlane(MPlane* other){
	  for(unsigned int i=0; i<square_Planes->size();i++){
		  if(square_Planes->at(i)->isEqualTo(other))
			  return square_Planes->at(i);
	  }
	  return 0;
}

void MapSquare::addRating(){
	if(ratable){
		rating++;
		ratable=false;
	}
}

void MapSquare::setRatable(){
	if(!ratable){
		std::vector<int> *toremove=new std::vector<int>;
		for(int i=0;i<square_Planes->size();i++){
			square_Planes->at(i)->
					setRatable();
			if(square_Planes->at(i)->rating/rating<=0.2){
				toremove->push_back(i);
			}
		}
		for(int i=0;i<toremove->size();i++){
			MPlane* temp=square_Planes->at(toremove->at(i)-i);
			square_Planes->erase(square_Planes->begin() + toremove->at(i)-i);
			delete temp;
		}

		toremove->clear();
		delete toremove;
	}
	ratable=true;
}

std::string MapSquare::toString(){
	 std::string stats[]={"FLOOR","AVAILABLE","BLOCKED","UNCHARTED"};
	 std::stringstream ss;
	 ss<<"no. of planes: "<<square_Planes->size()
			 <<") , status: "<<stats[square_status];
	 return ss.str();
}
