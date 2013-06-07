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
//	Trace wheelTrace  = Trace(RPY(0.3,0,0.4,M_PI,M_PI,0), 0.2, 0, M_PI/2, 10);
//	IkSolution solution = rScanRPY(0, 0, 0, RPY(.42 ,-.34 ,0.0,M_PI,0,0), 0.01);
//	if (solution.valid)
//	{
//		solution.Print();
//		RPY r = rPose(0,0,0, solution);
//		r.Print();
//		std::cout << "error1: " << solution.error << std::endl;
//	}
//	solution = rScanRPY(0, 0, 0, RPY(.50 ,-.34 ,0.02,M_PI,0,0), 0.01);
//	if (solution.valid)
//	{
//		solution.Print();
//		RPY r = rPose(0,0,0, solution);
//		r.Print();
//		std::cout << "error2: " << solution.error << std::endl;
//	}
//	solution = rScanRPY(0, 0, 0, RPY(.50 ,-.34 ,-0.02,M_PI,0,0), 0.01);
//	if (solution.valid)
//	{
//		solution.Print();
//		RPY r = rPose(0,0,0, solution);
//		r.Print();
//		std::cout << "error3: " << solution.error << std::endl;
//	}
	IkSolution T0 = IkSolution(0,0,0,0,0,0);
	IkSolution T1 = IkSolution(1,0,0,0,0,0);
	IkSolution T2 = IkSolution(2,0,0,0,0,0);
	IkSolution T3 = IkSolution(3,0,0,0,0,0);
	pPathPoints points = pPathPoints(T0, T1, 10, NoEnd);
	int i;
	for (i = 0 ; i< 10; i++)
	{
		std::cout<<"x:"<<points.pArray[i]._q4<<"\n";
	}
	points.Update(T1, T2, NoStartEnd);
	for (i = 0 ; i< 10; i++)
	{
		std::cout<<"x:"<<points.pArray[i]._q4<<"\n";
	}
	points.Update(T2, T3, NoStart);
	for (i = 0 ; i< 10; i++)
	{
		std::cout<<"x:"<<points.pArray[i]._q4<<"\n";
	}


//	Trace Trc2 = Trace(RPY(0.3,0,0.4,1.5,0.4,0), 0.2, 0, 1.57, 5);
//	RPY r1,r2;
//	double A[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
//	r1.ToRPY(Matrix(A));
//	r1.Print();
//	Matrix T1 = r1.FromRPY();
//	T1.Print();
//	double B[4][4] = {{-1,0,0,0},{0,-1,0,0},{0,0,-1,0},{0,0,0,1}};
//	r2.ToRPY(Matrix(B));
//	r2.Print();
//	Matrix T2 = r2.FromRPY();
//	T2.Print();
//	Quarternion Q1 = Quarternion(0,0,0,1,1,1,1);
//	Q1.Print();
//	Quarternion Q2 = QuarAngle(0,0,0,0.1,1,1,1);
//	Q2.Print();
//	Matrix T3 = Q2.ToMatrix();
//	T3.Print();
//	RPY r3;
//	r3.ToRPY(T3);
//	r3.Print();
//	Quarternion Q3 = QuarAngle(0,0,0,-0.1,-1,-1,-1);
//	Q3.Print();
//	Matrix T4 = Q3.ToMatrix();
//	T4.Print();
//	RPY r4;
//	r4.ToRPY(T4);
//	r4.Print();
//	std::cout<<"R P Y:"<<r3.R-r4.R<<" "<<r3.P-r4.P<<" "<<r3.Y-r4.Y<<"\n";

}



