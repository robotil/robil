/*
 * Trace.h
 *
 *  Created on: May 8, 2013
 *      Author: michaeldavid
 */

#ifndef TRACE_H_
#define TRACE_H_
//#include <iostream>

class Trace{
public:
	//int size;
	RPY *pArray;
	Trace(RPY r1, RPY r2, int pointsNum);
	Trace(RPY center, double radius, double angle_i, double angle_f, int pointsNum);
	~Trace();

};
RPY TraceAngle(RPY center, RPY position, double angle);


#endif /* TRACE_H_ */
