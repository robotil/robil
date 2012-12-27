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
enum status{AVAILABLE,BLOCKED,UNCHARTED};		//AVAILABLE = 0,BLOCKED = 1,UNCHARTED = 2

class MapSquare {
public:
	status square_status;
	std::vector<MPlane*> *square_Planes;
	MapSquare();
	virtual ~MapSquare();
	void clearSq();
	bool hasPlane(MPlane* other);
	std::string toString();
};


#endif /* MAPSQUARE_H_ */
