/*
 * Trace.cpp
 *
 *  Created on: May 8, 2013
 *      Author: michaeldavid
 */
#include <math.h>
#include <iostream>
#include <string>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Trace.h>


Trace::Trace(RPY r1, RPY r2, int pointsNum)
{
	pArray = new RPY[pointsNum];
	for (int i = 0; i < pointsNum; i++)
	{
		pArray[i].x = (r2.x - r1.x)*(i+1)/pointsNum + r1.x;
		pArray[i].y = (r2.y - r1.y)*(i+1)/pointsNum + r1.y;
		pArray[i].z = (r2.z - r1.z)*(i+1)/pointsNum + r1.z;
		pArray[i].R = (r2.R - r1.R)*(i+1)/pointsNum + r1.R;
		pArray[i].P = (r2.P - r1.P)*(i+1)/pointsNum + r1.P;
		pArray[i].Y = (r2.Y - r1.Y)*(i+1)/pointsNum + r1.Y;
		std::cout<<"index "<<i<<":";
		pArray[i].Print();
	}
}

Trace::Trace(RPY center, double radius, double angle_i, double angle_f, int pointsNum)
{
	pArray = new RPY[pointsNum];
	double angle;
	double R0 = center.R;
	RPY local_r;

	for (int i = 0; i < pointsNum; i++)
	{
		angle = (angle_f - angle_i)*(i)/(pointsNum-1) + angle_i;
		center.R = R0 + angle;
		Matrix A = center.FromRPY();
		local_r = RPY(0, radius, 0, M_PI/2, M_PI + M_PI/3, +M_PI/6);
		A.Multiply(local_r.FromRPY());
		pArray[i].ToRPY(A);
		std::cout<<"index "<<i<<" angle "<< angle<< ":";
		pArray[i].Print();
	}
}

Trace::~Trace()
{
	//std::cout << "before delete\n";
	delete [] pArray;
	//std::cout << "after delete\n";
}

RPY TraceAngle(RPY center, RPY position, double angle)
{
	center.R += angle;
	Matrix A = center.FromRPY();
	A.Multiply(position.FromRPY());
	RPY ans;
	ans.ToRPY(A);
	return ans;
}

RPY TraceInverse(RPY A)
{
	Matrix MA = A.FromRPY();
	// make -Rt*P
	double Rx = -MA.T[0][0]*MA.T[0][3] + -MA.T[1][0]*MA.T[1][3] + -MA.T[2][0]*MA.T[2][3];
	double Ry = -MA.T[0][1]*MA.T[0][3] + -MA.T[1][1]*MA.T[1][3] + -MA.T[2][1]*MA.T[2][3];
	double Rz = -MA.T[0][2]*MA.T[0][3] + -MA.T[1][2]*MA.T[1][3] + -MA.T[2][2]*MA.T[2][3];

	double R[4][4] = 	{{MA.T[0][0],MA.T[1][0],MA.T[2][0],Rx},
						{MA.T[0][1],MA.T[1][1],MA.T[2][1],Ry},
						{MA.T[0][2],MA.T[1][2],MA.T[2][2],Rz},
						{0,0,0,1}};

	Matrix MB = Matrix(R);
	RPY B;
	B.ToRPY(MB);
	return B;
}

RPY TraceAB(RPY A, RPY B)
{
	Matrix MA = A.FromRPY();
	Matrix MB = B.FromRPY();
	MA.Multiply(MB);
	RPY C;
	C.ToRPY(MA);
	return C;
}
