/*
 * MapMatrix.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */
#include "pclPlane.h"
#include "MPlane.h"
#include "MapSquare.h"
#include "MapMatrix.h"
#include "math.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <boost/thread/mutex.hpp>
using namespace std;

//c24 added includes
MapMatrix::MapMatrix() {
	data=new std::deque<std::deque<MapSquare*>*>();
	for (int i=0;i<NUMOFSQUARES;i++){
		data->push_back(new std::deque<MapSquare*>());
		for (int j=0;j<NUMOFSQUARES;j++){
			data->at(i)->push_back(new MapSquare());
		}
	}
	xOffset=0;
	yOffset=-SIZEOFMAP/2;
}

MapMatrix::~MapMatrix() {
	while(data->size()!=0){
		std::deque<MapSquare*> *temp=data->back();
		while(temp->size()!=0){
			MapSquare* temp1=temp->back();
			temp->pop_back();
			//delete temp1;
		}
		data->pop_back();
		//delete temp;
	}
	//delete data;
}

void MapMatrix::setAtlasPos(geometry_msgs::Point pose){
	int ax=(pose.x-xOffset)*SIZEOFSQUARE;
	int ay=(pose.y-yOffset)*SIZEOFSQUARE;
	//cout<<"Atlas is in "<<"("<<ax<<","<<ay<<")"<<endl;

	data->at(ax)->at(ay)->square_status=ATLAS;
}

void MapMatrix::printMatrix(){
	cout<<"About to print matrix"<<endl;
	for (int i=0;i<NUMOFSQUARES;i++){
		if (i==0){
			std::cout<<"   ";
			for (int j=0;j<NUMOFSQUARES;j++){
				std::cout<<std::setw(2)<<std::setfill('0')<<j<<" ";
			}
			std::cout<<endl;
		}
	}
	for (int i=NUMOFSQUARES;i>0;i--){
		std::cout<<std::setw(2)<<std::setfill('0')<<i<<" ";
		for (int j=0;j<NUMOFSQUARES;j++){
			if (data->at(i-1)->at(j)->square_status==AVAILABLE)
				std::cout << "A  ";
				else if (data->at(i-1)->at(j)->square_status==BLOCKED)
					std::cout << "B  ";
				else   if (data->at(i-1)->at(j)->square_status==ATLAS)
					std::cout << "X  ";
				else
					std::cout << "-  ";
		}
		std::cout <<endl;
	}
}

double MapMatrix::calcSlopeZ(float a,float b,float c){
	double ang = std::acos((c)/std::sqrt(a*a+b*b+c*c))*180/M_PI;
	return ang;
}

void MapMatrix::clearMatrix(){
	for(unsigned int i=0;i<data->size();i++){
		for(unsigned int j=0;j<data->at(i)->size();j++){
			data->at(i)->at(j)->ratable=true;
		}
	}
}

bool MapMatrix::inMatrixRange(pcl::PointXYZ p){
	if (p.x<0+xOffset || (p.x >= NUMOFSQUARES*SIZEOFSQUARE+xOffset)
			||(p.y<0+yOffset) || (p.y>=NUMOFSQUARES*SIZEOFSQUARE+yOffset))
		return false;
	return true;
}


void MapMatrix::updateMapRelationToRobot(float movmentX,float movmentY,float yaw){
/*	boost::mutex::scoped_lock l(mutex);
	//calculate the map offsets according to the robot's global position and update bound
	int newOffsetX=xOffset;
	int newOffsetY=yOffset;
	if(xOffset> x-BOUNDOFUPDATE)
		newOffsetX= x-BOUNDOFUPDATE;
	if(xOffset+NUMOFSQUARES*SIZEOFSQUARE< x+BOUNDOFUPDATE)
		newOffsetX=x+BOUNDOFUPDATE;
	if(yOffset> y-BOUNDOFUPDATE)
		newOffsetY= y-BOUNDOFUPDATE;
	if(yOffset+NUMOFSQUARES*SIZEOFSQUARE< y+BOUNDOFUPDATE)
		newOffsetY=y+BOUNDOFUPDATE;
	//move the map to the right directions
	//std::cout<<"x:"<<x<<" y:"<<y<<std::endl;
	//std::cout<<"x offset"<<newOffsetX<<" y offset"<<newOffsetY<<std::endl;
	if(std::abs(newOffsetY-yOffset)>=NUMOFSQUARES*SIZEOFSQUARE || std::abs(newOffsetX-xOffset)>=NUMOFSQUARES*SIZEOFSQUARE){
		yOffset=newOffsetY;
		xOffset=newOffsetX;
		clearMatrix();
		return;
	}
	moveMapHarisontaly(((int)newOffsetY-yOffset)*(1/SIZEOFSQUARE));
	moveMapVerticaly(((int)newOffsetX-xOffset)*(1/SIZEOFSQUARE));
	yOffset=newOffsetY;
	xOffset=newOffsetX;
	*/
}

void MapMatrix::moveMapHarisontaly(int times){
	if(times<0){
		for (int i=0;i<std::abs(times);i++){
			data->pop_back();
			std::deque<MapSquare*>* temp=new std::deque<MapSquare*>;
			for (int j=0;j<NUMOFSQUARES;j++){
				temp->push_back(new MapSquare);
			}
			data->push_front(temp);
		}
	}else{
		for (int i=0;i<std::abs(times);i++){
			data->pop_front();
			std::deque<MapSquare*>* temp=new std::deque<MapSquare*>;
			for (int j=0;j<NUMOFSQUARES;j++){
				temp->push_back(new MapSquare);
			}
			data->push_back(temp);
		}
	}
}

void MapMatrix::moveMapVerticaly(int times){
	if(times<0){
		for (int i=0;i<std::abs(times);i++){
			for (int j=0;j<NUMOFSQUARES;j++){
				data->at(j)->pop_back();
				data->at(j)->push_front(new MapSquare);
			}
		}
	}else{
		for (int i=0;i<std::abs(times);i++){
			for (int j=0;j<NUMOFSQUARES;j++){
				data->at(j)->pop_front();
				data->at(j)->push_back(new MapSquare);
			}
		}
	}
}


//c24 changes + need to use type of the imu message which is const OdometryConstPtr& pos_msg
/*
 * the main idea here will be, having all planes, and the imu normal (coordinates and angle)
 * going on each plane and compare its normal with the one from the imu, then we need to decide
 * how similar they should be, and if they are similar (meaning also part of the ground) then "erase"
 * this plane and going to the next plane etc.
 */
void MapMatrix::computeMMatrix(std::vector<pclPlane*>* mapPlanes,pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud){ //update way of calculating x and y indices of mapMatrix
	for (unsigned int i=0; i< mapPlanes->size();i++){ //goes through all planes
			pcl::PointIndices::Ptr inliers = mapPlanes->at(i)->inliers;
			pcl::ModelCoefficients::Ptr coff= mapPlanes->at(i)->coefficients;
			MPlane* tempPlane=new MPlane(pcl::PointXYZ(0,0,0),coff);
			double angle=calcSlopeZ(tempPlane->coefficient_x,tempPlane->coefficient_y,tempPlane->coefficient_z);
			for (unsigned int j=0; j< inliers->indices.size();j++){ //goes through all indices in the plane i
				pcl::PointXYZ p = map_cloud->points[inliers->indices[j]];

				if (inMatrixRange(p)){
					int xIndex,yIndex;
					xIndex = (p.x -xOffset)*(1/SIZEOFSQUARE);	//added for now instead of previous three lines
					yIndex = (p.y-yOffset) *(1/SIZEOFSQUARE);	//same as above
					MapSquare* ms=data->at(xIndex)->at(yIndex);
					if(ms->ratable){
						ms->clearSq();
						ms->ratable=false;
					}
					//ms->addRating();
					if(!ms->hasPlane(tempPlane)){
						MPlane* newPlane=new MPlane(pcl::PointXYZ(p.x,p.y,p.z),coff);
						newPlane->addRating();
						newPlane->rating=20;
						ms->square_Planes->push_back(newPlane);
						if (p.z>(0.5+pelvisHeight) || p.z<-0.4+pelvisHeight){
							ms->square_status = BLOCKED;
						}
						else{
							if(ms->square_status!=BLOCKED){
								if(p.z>(0.15+pelvisHeight) || p.z<(-0.15+pelvisHeight)){
									ms->square_status = DEBREE;
									cout<<p.z<<"\n";
								}
							}
							if(ms->square_status!=BLOCKED && ms->square_status!=DEBREE){
								ms->square_status = AVAILABLE;
							}
						}
					}else{
						MPlane* temp=ms->getPlane(tempPlane);
						 if(temp->representing_point.z<p.z){
							 temp->representing_point.z=p.z;
							 temp->representing_point.y=p.y;
							 temp->representing_point.x=p.x;

						 }
					}
				}
			}
		}
		/*for(unsigned int i=0; i< data->size();i++){
			for(unsigned int j=0; j< data->at(i)->size();j++){
				 data->at(i)->at(j)->setRatable();
			}
		}*/

	//pcl::ModelCoefficients c;
	/*for (unsigned int i=0;i<NUMOFSQUARES;i++){
		for (unsigned int j=0;j<NUMOFSQUARES;j++){
			data->at(i)->at(j)->square_status=UNCHARTED;
			/*if (data->at(i)->at(j)->scansLeft-- ==0 ){
				data->at(i)->at(j)->square_status=UNCHARTED;
				data->at(i)->at(j)->scansLeft=MAXSCANS;
			}*/
/*		}
	}
	

	for (unsigned int i=0; i< map_cloud->points.size();i++){
		pcl::PointXYZ p = map_cloud->points.at(i);
				if (inMatrixRange(p)){
					int xIndex,yIndex;
					xIndex = (p.x -xOffset)*(1/SIZEOFSQUARE);	//added for now instead of previous three lines
					yIndex = (p.y-yOffset) *(1/SIZEOFSQUARE);	//same as above
					//std::cout<<"xOffset:"<<xOffset<<" yOffset:"<<yOffset<<"\n";
					//std::cout<<"indexX:"<<xIndex<<" indexY"<<yIndex<<"\n";
					boost::mutex::scoped_lock l(mutex);
					MapSquare* ms=data->at(xIndex)->at(yIndex);
					ms->addRating();
					
					if(!ms->hasTop(p.z)){
						MPlane* newPlane=new MPlane(p);
						newPlane->addRating();
						newPlane->rating=20;
						ms->square_Planes->push_back(newPlane);
						if (((p.z > -PELVIS_HEIGHT-0.5) && (p.z < -PELVIS_HEIGHT+0.5))||(p.z > -PELVIS_HEIGHT+2.0)){
							ms->square_status = AVAILABLE;
							ms->scansLeft=MAXSCANS;
						}
						else{
							ms->square_status = BLOCKED;
							ms->scansLeft=MAXSCANS;
						}
						
					}else{
						MPlane* temp=ms->getTop(p.z);
						if(temp)
							temp->addRating();
					}
						/*MPlane* newPlane=new MPlane(p);
						newPlane->addRating();
						newPlane->rating=20;
						ms->square_Planes->push_back(newPlane);
						if (((p.z > -PELVIS_HEIGHT-0.5) && (p.z < -PELVIS_HEIGHT+0.5))||(p.z > -PELVIS_HEIGHT+2.0))
						//if (((p.z > -1.40845) && (p.z < -0.40845))||(p.z > 1.40845))
							ms->square_status = AVAILABLE;
						else{
							ms->square_status = BLOCKED;
						}*/
/*				}
	}

	for(unsigned int i=0; i< data->size();i++){
		for(unsigned int j=0; j< data->at(i)->size();j++){
			 data->at(i)->at(j)->setRatable();
		}
	}*/
}
