/*
 * pclPlan.h
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#ifndef PCLPLANE_H_
#define PCLPLANE_H_

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
/**
 * this class is used to hold the plane we find in the SEGMENTATION part of the algorithm
 */
class pclPlane {
public:
	pcl::PointIndices::Ptr inliers;
	pcl::ModelCoefficients::Ptr coefficients;
	pclPlane();
	virtual ~pclPlane();
	pclPlane(pcl::PointIndices::Ptr i_, pcl::ModelCoefficients::Ptr c_);


	std::string toString();
};

#endif /* PCLPLAN_H_ */
