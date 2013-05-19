/*
 * WallDetector.cpp
 *
 *  Created on: May 19, 2013
 *      Author: dan
 */

#include "WallDetector.h"

Map WallDetector::simplify()const{
	Map grid(alts.w(), alts.h());
	std::cout<<"Grid created: "<< grid.w() <<'x'<< grid.h()<< std::endl;
	size_t f_x=0, l_x=alts.w()-1;
	size_t f_y=0, l_y=alts.h()-1;
	for(size_t y=f_y+1; y<l_y; y++)	for(size_t x=f_x+1; x<l_x; x++) grid(x,y)=filter(x,y);
	for(size_t y=f_y+1; y<l_y; y++) { grid(f_x,y)=filterBorder(f_x,y); grid(l_x,y)=filterBorder(l_x,y); }
	for(size_t x=f_x+1; x<l_x; x++) { grid(x,f_y)=filterBorder(x,f_y); grid(x,l_y)=filterBorder(x,l_y); }
	return grid;
}

char WallDetector::filter(size_t x, size_t y)const{
	double alt = alts(x,y)-max_alt;
	if(
		alt > alts(x-1,y) || alt > alts(x+1,y) || alt > alts(x,y-1) || alt > alts(x,y+1)
	) return Map::ST_BLOCKED;
	return Map::ST_AVAILABLE;
}
char WallDetector::filterBorder(size_t x, size_t y)const{
	double alt = alts(x,y)-max_alt;
	if( x>0 			&& alt > alts(x-1,y)) return Map::ST_BLOCKED;
	if( x<alts.w()-1 	&& alt > alts(x+1,y)) return Map::ST_BLOCKED;
	if( y>0 			&& alt > alts(x,y-1)) return Map::ST_BLOCKED;
	if( y<alts.h()-1 	&& alt > alts(x,y+1)) return Map::ST_BLOCKED;
	return Map::ST_AVAILABLE;
}
