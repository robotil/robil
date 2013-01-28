/*
 * MatrixPlane.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#include "MPlane.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

MPlane::MPlane():representing_point(0,0,0){
	coefficient_x=0;
	coefficient_y=0;
	coefficient_z=0;
	coefficient_d=0;
}

MPlane::MPlane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr c_):representing_point(point){
	coefficient_x=c_->values.at(0);
	coefficient_y=c_->values.at(1);
	coefficient_z=c_->values.at(2);
	coefficient_d=c_->values.at(3);
}

MPlane::~MPlane(){

}

bool MPlane::isEqualTo(MPlane * other){
  if (other->coefficient_x==coefficient_x)
	  if (other->coefficient_y==coefficient_y)
		  if (other->coefficient_z==coefficient_z)
			  if (other->coefficient_d==coefficient_d)
				  return true;
  return false;
}

std::string MPlane::toString(){
  std::stringstream ss;
  ss<<coefficient_x<<"x + "<<coefficient_y<<"y + "<<coefficient_z<<"z + "<<coefficient_d<<" = 0";
  return ss.str();
}

