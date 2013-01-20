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
using namespace std;

//c24 added includes
MapMatrix::MapMatrix() {
	data=new std::vector<std::vector<MapSquare*>*>();
	for (int i=0;i<NUMOFSQUARES;i++){
		data->push_back(new std::vector<MapSquare*>());
		for (int j=0;j<NUMOFSQUARES;j++){
			data->at(i)->push_back(new MapSquare());
		}
	}
}

MapMatrix::~MapMatrix() {
	while(data->size()!=0){
		std::vector<MapSquare*> *temp=data->back();
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
				else std::cout << "-  ";
		}
		std::cout <<endl;
	}
}

double MapMatrix::calcSlopeZ(float a,float b,float c){
	double ang = 180-std::acos((c)/std::sqrt(a*a+b*b+c*c))*180/M_PI;
	return ang;
}

void MapMatrix::clearMatrix(){
	for(unsigned int i=0;i<data->size();i++){
		for(unsigned int j=0;j<data->at(i)->size();j++){
			data->at(i)->at(j)->clearSq();
		}
	}
}

bool MapMatrix::inMatrixRange(pcl::PointXYZ p){
	if (p.x<0 || (p.x >= SIZEOFMAP)
			||(p.z<0) || (p.z>=SIZEOFMAP))
		return false;
	return true;
}

//c24 changes + need to use type of the imu message which is const OdometryConstPtr& pos_msg
/*
 * the main idea here will be, having all planes, and the imu normal (coordinates and angle)
 * going on each plane and compare its normal with the one from the imu, then we need to decide
 * how similar they should be, and if they are similar (meaning also part of the ground) then "erase"
 * this plane and going to the next plane etc.
 */
void MapMatrix::computeMMatrix(std::vector<pclPlane*>* mapPlanes,pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud){ //update way of calculating x and y indices of mapMatrix
	int xIndex,yIndex;
	data->at(0)->at(SIZEOFMAP*2)->square_status=BLOCKED;

	for (unsigned int i=0; i< mapPlanes->size();i++){ //goes through all planes
		pcl::PointIndices::Ptr inliers = mapPlanes->at(i)->inliers;
		pcl::ModelCoefficients::Ptr coff= mapPlanes->at(i)->coefficients;
		MPlane* tempPlane=new MPlane(pcl::PointXYZ(0,0,0),coff);
		double angle=calcSlopeZ(tempPlane->coefficient_x,tempPlane->coefficient_z,tempPlane->coefficient_y);
		for (unsigned int j=0; j< inliers->indices.size();j++){ //goes through all indices in the plane i
			pcl::PointXYZ p = map_cloud->points[inliers->indices[j]];
			p.x+= (SIZEOFMAP/2);	//adapt x axis to matrix
			//p.z+= BEHIND;	//adapt y axis to matrix
			if (inMatrixRange(p)){
				xIndex = p.x *4;	//added for now instead of previous three lines
				yIndex = p.z *4;	//same as above
				MapSquare* ms=data->at(yIndex)->at(xIndex);
				if(!ms->hasPlane(tempPlane)){
					MPlane* newPlane=new MPlane(pcl::PointXYZ(p.x,p.y,p.z),coff);

					ms->square_Planes->push_back(newPlane);
					if (angle>30){
						ms->square_status = BLOCKED;
					}
					else{
						if(ms->square_status!=BLOCKED){
							ms->square_status = AVAILABLE;
						}
					}
				}
			}
		}
	}
}
