/*
 * MatrixPlane.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#include "MPlane.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
double EQUALITY_THRESH=0.01;

MPlane::MPlane():representing_point(0,0,0){
	coefficient_x=0;
	coefficient_y=0;
	coefficient_z=0;
	coefficient_d=0;
	rating=0;
	  ratable=true;
}

MPlane::MPlane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr c_):representing_point(point){
	coefficient_x=c_->values.at(0);
	coefficient_y=c_->values.at(1);
	coefficient_z=c_->values.at(2);
	coefficient_d=c_->values.at(3);
}

MPlane::~MPlane(){

}

void MPlane::addRating(){
	if(ratable){
		rating++;
		if(rating==100)
			rating=100;
		ratable=false;
	}
}
void MPlane::setRatable(){
	if(ratable){
		rating--;
	}
	ratable=true;
}

bool MPlane::isEqualTo(MPlane * other){
  if (std::max(other->coefficient_x,coefficient_x)-std::min(other->coefficient_x,coefficient_x)<=EQUALITY_THRESH)
	  if (std::max(other->coefficient_y,coefficient_y)-std::min(other->coefficient_y,coefficient_y)<=EQUALITY_THRESH)
		  if (std::max(other->coefficient_z,coefficient_z)-std::min(other->coefficient_z,coefficient_z)<=EQUALITY_THRESH)
			  if (std::max(other->coefficient_d,coefficient_d)-std::min(other->coefficient_d,coefficient_d)<=EQUALITY_THRESH)
				  return true;
  return false;
}

std::string MPlane::toString(){
  std::stringstream ss;
  //ss<<coefficient_x<<"x + "<<coefficient_y<<"y + "<<coefficient_z<<"z + "<<coefficient_d<<" = 0";
  return ss.str();
}

