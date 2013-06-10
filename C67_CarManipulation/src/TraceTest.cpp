/*
 * TraceTest.cpp
 *
 *  Created on: May 9, 2013
 *      Author: michaeldavid
 */
#include <math.h>
#include <iostream>
#include <string>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Path.h>
#include <C67_CarManipulation/Trace.h>

int main(int argc, char** argv)
{
	std::cout.precision(6);
	std::cout.setf (std::ios::fixed , std::ios::floatfield );
	std::cout<< "Test Program End\n";
	//pelvis in world coordinates
	RPY A = RPY(-1.928,-5.0712,0.916,0,-.016,2.357);
	//hose in world coordinates
	RPY C = RPY(-2.19,-4.698,1.052,0,0.0329,1.672);
	//hose in pelvis coordinates
	RPY B = TraceAB(TraceInverse(A),C);
	B.Print();

}



