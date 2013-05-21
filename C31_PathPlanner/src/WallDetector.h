/*
 * WallDetector.h
 *
 *  Created on: May 19, 2013
 *      Author: dan
 */

#ifndef WALLDETECTOR_H_
#define WALLDETECTOR_H_

#include "Map.h"

class WallDetector {
public:
	const AltMap& alts;
	const double max_alt;
	WallDetector(const AltMap& m, double maxalt):alts(m), max_alt(maxalt){}
	Map simplify()const;
private:
	char filter(size_t x, size_t y)const;
	char filterBorder(size_t x, size_t y)const;
};

#endif /* WALLDETECTOR_H_ */
