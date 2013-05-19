/*
 * MapMatrix.h
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#ifndef MAPMATRIX_H_
#define MAPMATRIX_H_
#include "pclPlane.h"
#include "MPlane.h"
#include "MapSquare.h"
#include "math.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <deque>
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/Point.h"

#define PELVIS_HEIGHT 0.90845
const int SIZEOFMAP = 10;	//The map matrix is defined as a square of SIZEOFMAP*SIZEOFMAP meters
const double SIZEOFSQUARE = 0.25;	//Size of each square is defined by SIZEOFSQUARE*SIZEOFSQUARE in meters
const int BEHIND = 2;		//Number of meters behind robot in map matrix
const int BOUNDOFUPDATE = 0.05;
const int NUMOFSQUARES = SIZEOFMAP/SIZEOFSQUARE;	//in one row or column
class MapMatrix {
public:
	boost::mutex mutex;
	std::deque<std::deque<MapSquare*>*>* data;
	MapMatrix();
	virtual ~MapMatrix();
	void clearMatrix();
	void setAtlasPos(geometry_msgs::Point pose);
	void printMatrix();
	void updateMapRelationToRobot(float movmentX,float movmentY,float yaw);
	void moveMapHarisontaly(int times);
	void moveMapVerticaly(int times);
	int xOffset,yOffset;
	void computeMMatrix(std::vector<pclPlane*>* mapPlanes,pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud);
private:
	double calcSlopeZ(float a,float b,float c);
	bool inMatrixRange(pcl::PointXYZ p);
};

#endif /* MAPMATRIX_H_ */
