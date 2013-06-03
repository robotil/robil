/*
 * MapSquare.h
 *
 *  Created on: Dec 26, 2012
 *      Author: root
 */

#ifndef MAPSQUARE_H_
#define MAPSQUARE_H_

#include <vector>
#include <string.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include "MPlane.h"

#define MAXSCANS 360
enum status{AVAILABLE,BLOCKED,UNCHARTED,ATLAS};		//AVAILABLE = 0,BLOCKED = 1,UNCHARTED = 2

class MapSquare {
public:
	int rating;
	bool ratable;
	status square_status;
	std::vector<MPlane*> *square_Planes;
	int scansLeft;
	MapSquare();
	virtual ~MapSquare();
	void addRating();
	void setRatable();
	void clearSq();
	bool hasPlane(MPlane* other);
	bool hasTop(double top);
	MPlane *getTop(double top);
	MPlane* getPlane(MPlane* other);
	std::string toString();
};


#endif /* MAPSQUARE_H_ */
