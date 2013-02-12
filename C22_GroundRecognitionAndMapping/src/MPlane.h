/*
 * MatrixPlane.h
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#ifndef MATRIXPLANE_H_
#define MATRIXPLANE_H_
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
/**
 * this class is used to hold the plane we find in the MAPPING part of the algorithm
 */
class MPlane {
public:
	  double coefficient_x,coefficient_y,coefficient_z,coefficient_d;
	  pcl::PointXYZ representing_point;
	  MPlane();
	  int rating;
	  MPlane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr c_);
	  virtual ~MPlane();
	  bool isEqualTo(MPlane * other);
	  std::string toString();
};



#endif /* MATRIXPLANE_H_ */
