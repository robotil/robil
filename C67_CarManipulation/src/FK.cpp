/*
FK.cpp Forward Kinematics file
*/
#include <math.h>
#include <iostream>
#include <string>
#include "FK.h"

void Matrix::RotateX(double A[4][4], double val)
{
	double B[3][3] = {{1,0,0},{0,cos(val),-sin(val)},{0,sin(val),cos(val)}};
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			A[i][j] = B[i][j];
}
void Matrix::RotateY(double A[4][4], double val)
{
	double B[3][3] = {{cos(val),0,sin(val)},{0,1,0},{-sin(val),0,cos(val)}};
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			A[i][j] = B[i][j];
}
void Matrix::RotateZ(double A[4][4], double val)
{
	double B[3][3] = {{cos(val),-sin(val),0},{sin(val),cos(val),0},{0,0,1}};
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			A[i][j] = B[i][j];
}
void Matrix::rRotateYZ(double A[4][4], double val)
{
	double B[3][3] = {{cos(val), -rzr*sin(val), ryr*sin(val)}, {rzr*sin(val), ryr*ryr*(1-cos(val))+cos(val), ryr*rzr*(1-cos(val))},
		{-ryr*sin(val), ryr*rzr*(1-cos(val)), rzr*rzr*(1-cos(val))+cos(val)}};
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			A[i][j] = B[i][j];
}

void Matrix::lRotateYZ(double A[4][4], double val)
{
	double B[3][3] = {{cos(val), -rzl*sin(val), ryl*sin(val)}, {rzl*sin(val), ryl*ryl*(1-cos(val))+cos(val), ryl*rzl*(1-cos(val))},
		{-ryl*sin(val), ryl*rzl*(1-cos(val)), rzl*rzl*(1-cos(val))+cos(val)}};
	int i,j;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			A[i][j] = B[i][j];
}
void Matrix::rUpdate(double A[4][4], int Type, double val)
{
	A[3][0] = A[3][1] = A[3][2] = 0;
	A[dx] = A[dy] = A[dz] = 0;
	A[3][3] = 1;
	switch(Type)
	{
		case q1:
		RotateZ(A,val);
		A[dx] = x1;
		break;
		
		case q2:
		RotateY(A,val);
		A[dz] = z2;
		break;
		
		case q3:
		RotateX(A,val);
		A[dz] = z3;
		break;
		
		case q4r:
		rRotateYZ(A,val);
		A[dx] = x4r;
		A[dy] = y4r;
		A[dz] = z4r;
		break;
		
		case q5r:
		RotateX(A,val);
		A[dy] = y5r;
		A[dz] = z5r;
		break;
		
		case q6r:
		RotateY(A,val);
		A[dy] = y6r;
		break;
		
		case q7r:
		RotateX(A,val);
		A[dy] = y7r;
		A[dz] = z7r;
		break;
		
		case q8r:
		RotateY(A,val);
		A[dy] = y8r;
		A[dz] = z8r;
		break;
		
		case q9r:
		RotateX(A,val);
		A[dy] = y9r;
		break;
		
		default:
		break;
	}
}

void Matrix::lUpdate(double A[4][4], int Type, double val)
{
	A[3][0] = A[3][1] = A[3][2] = 0;
	A[dx] = A[dy] = A[dz] = 0;
	A[3][3] = 1;
	switch(Type)
	{
		case q1:
		RotateZ(A,val);
		A[dx] = x1;
		break;
		
		case q2:
		RotateY(A,val);
		A[dz] = z2;
		break;
		
		case q3:
		RotateX(A,val);
		A[dz] = z3;
		break;
		
		case q4l:
		lRotateYZ(A,val);
		A[dx] = x4l;
		A[dy] = y4l;
		A[dz] = z4l;
		break;
		
		case q5l:
		RotateX(A,val);
		A[dy] = y5l;
		A[dz] = z5l;
		break;
		
		case q6l:
		RotateY(A,val);
		A[dy] = y6l;
		break;
		
		case q7l:
		RotateX(A,val);
		A[dy] = y7l;
		A[dz] = z7l;
		break;
		
		case q8l:
		RotateY(A,val);
		A[dy] = y8l;
		A[dz] = z8l;
		break;
		
		case q9l:
		RotateX(A,val);
		A[dy] = y9l;
		break;
		
		default:
		break;
	}
}


void Matrix::rGet(int Type, double val)
{
	rUpdate(T, Type, val); 
}

void Matrix::lGet(int Type, double val)
{
	lUpdate(T, Type, val); 
}

void Matrix::rMultiply(int Type, double val)
{
	double B[4][4];
	double C[4][4];
	int i,j,k;
	rUpdate(B, Type, val);
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
		{
	 		C[i][j] = 0;
	 		for (k=0; k<4; k++)
	 			C[i][j] += T[i][k]*B[k][j];
	 	}
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
			T[i][j] = C[i][j];		
	
}
void Matrix::lMultiply(int Type, double val)
{
	double B[4][4];
	double C[4][4];
	int i,j,k;
	lUpdate(B, Type, val);
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
		{
	 		C[i][j] = 0;
	 		for (k=0; k<4; k++)
	 			C[i][j] += T[i][k]*B[k][j];
	 	}
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
			T[i][j] = C[i][j];		
	
}
void Matrix::Multiply(Matrix A)
{
	double B[4][4];
	int i,j,k;
	
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
		{
	 		B[i][j] = 0;
	 		for (k=0; k<4; k++)
	 			B[i][j] += T[i][k]*A.T[k][j];
	 	}
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
			T[i][j] = B[i][j];		
	
}
void Matrix::Inverse()
{
	double B[4][4];
	int i,j;
	//rotary matrix transpose
	for (i=0; i<3; i++)
		for (j=0; j<3 ; j++)
			B[i][j] = T[j][i];
	//position
	B[0][3] = B[1][3] = B[2][3] = 0;
	for(i=0;i<3;i++)
	{
		B[0][3] += -T[i][0]*T[i][3]; 
		B[1][3] += -T[i][1]*T[i][3];
		B[2][3] += -T[i][2]*T[i][3]; 
	}
	// result to T (we only pass 3,4 matrix)
	for (i=0; i<3; i++)
		for (j=0; j<4 ; j++)
			T[i][j] = B[i][j];	
	
}
void Matrix::Print()
{
	std::cout.precision(6);
	std::cout.setf (std::ios::fixed , std::ios::floatfield );    
	for(int i=0;i<4;i++)
	{
		for (int j = 0;j<4;j++)
			std::cout << T[i][j] << " ,";
		std::cout << std::endl;
	}
}

void RPY::ToRPY(Matrix T)
{
	x = T.T[0][3];	
	y = T.T[1][3];
	z = T.T[2][3];
	R = atan2(T.T[2][1],T.T[2][2]);
	Y = atan2(T.T[1][0],T.T[0][0]);
	P = atan2(-T.T[2][0],cos(Y)*T.T[0][0] + sin(Y)*T.T[1][0]);
	
	if (cos(P) < 0)
	{
		R = atan2(-T.T[2][1],-T.T[2][2]);
		Y = atan2(-T.T[1][0],-T.T[0][0]);
		P = atan2(-T.T[2][0],cos(Y)*T.T[0][0] + sin(Y)*T.T[1][0]);
	}
	
}

Matrix RPY::FromRPY()
{
	int i,j;
	double B[4][4] = {{cos(Y)*cos(P),cos(Y)*sin(P)*sin(R)-sin(Y)*cos(R),cos(Y)*sin(P)*cos(R)+sin(Y)*sin(R),x},
	{sin(Y)*cos(P), sin(Y)*sin(P)*sin(R)+cos(Y)*cos(R), sin(Y)*sin(P)*cos(R)-cos(Y)*sin(R), y},
	{-sin(P), cos(P)*sin(R), cos(P)*cos(R),z},{0,0,0,1}};
	
	Matrix T = Matrix();
	for (i=0; i<4; i++)
		for (j=0; j<4 ; j++)
			T.T[i][j] = B[i][j];	
	
	return T;
}

void RPY::Print()
{
	std::cout << "RPY(" << x << ", "<< y << ", " << z << ", " << R << ", " << P << ", " << Y << ")\n";
}

Matrix rDest(double mq1,double mq2,double mq3,double mq4, RPY Target)
{
	Matrix T = Matrix();
	T.rGet(q1,mq1);
	T.rMultiply(q2,mq2);
	T.rMultiply(q3,mq3);
	T.rMultiply(q4r,mq4);
	T.Inverse();
	
	Matrix A = Matrix();
	A = Target.FromRPY();
	T.Multiply(A);

	return T;
}

Matrix lDest(double mq1,double mq2,double mq3,double mq4, RPY Target)
{
	Matrix T = Matrix();
	T.lGet(q1,mq1);
	T.lMultiply(q2,mq2);
	T.lMultiply(q3,mq3);
	T.lMultiply(q4l,mq4);
	T.Inverse();
	
	Matrix A = Matrix();
	A = Target.FromRPY();
	T.Multiply(A);

	return T;
}


