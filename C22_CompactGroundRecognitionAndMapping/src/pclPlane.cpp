/*
 * pclPlan.cpp
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#include "pclPlane.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

pclPlane::pclPlane(){}

pclPlane::pclPlane(pcl::PointIndices::Ptr i_, pcl::ModelCoefficients::Ptr c_):inliers(i_),coefficients(c_){}


pclPlane::~pclPlane(){
	inliers.reset();
	coefficients.reset();
}


std::string pclPlane::toString(){
	std::stringstream ss;
	ss<<coefficients->values[0]<<"x + "<<coefficients->values[1]<<"y + "<<coefficients->values[2]<<"z + "<<coefficients->values[3]<<" = 0";
    return ss.str();
}
