/*
IK.cpp Inverse Kinematics file
*/
#include <math.h>
#include <iostream>
#include <string>
#include "FK.h"
#include "IK.h"

// local functions
bool IkSqrSolve(double cc, double cs, double c1, double angles[2]); 


RPY IkSolution::Fk5Dof()
{
	RPY R;
	Matrix T = Matrix();
	T.Get(q5,_q5);
	T.Multiply(q6,_q6);
	T.Multiply(q7,_q7);
	T.Multiply(q8,_q8);
	T.Multiply(q9,_q9);
	R.ToRPY(T);
	return R;
}
bool IkSqrSolve(double cc, double cs, double c1, double angle[2])
{
	double m = - c1*c1 + cc*cc + cs*cs;
	if (m < 0)
		return false;
	if (c1 == cc)
	{
		angle[0] = M_PI;
		angle[1] = -M_PI;
		return true;
	}
	
	double x;
	x = -(cs + sqrt(m))/(c1 - cc);
	angle[0] = atan2(2*x,1-x*x);
	x = -(cs - sqrt(m))/(c1 - cc);
	angle[1] = atan2(2*x,1-x*x);
	return true;
}
void IkSolution::Print()
{
	std::cout.precision(6);
	std::cout.setf (std::ios::fixed , std::ios::floatfield ); 
	std::cout << "q5-q9:"<< _q5<< ", "<< _q6<< ", "<< _q7<< ", "<< _q8<< ", "<< _q9<< std::endl;
}
IkSolution Ik5Dof(RPY target)
{
	Matrix A = target.FromRPY();
	double cc,cs,c1, solution[2];
	bool solutionExist;
	
	
	IkSolution Ik[8];
	
	//solve q7
	cc = 2*(y67*y89 + z7*z8);
	cs = 2*(y89*z7 - y67*z8);
	c1 = y89*y89 + z8*z8 + y67*y67 + z7*z7 - py5*py5 - pz5*pz5 - px*px;
	
	solutionExist = IkSqrSolve(cc,cs,c1,solution);
	
	for(int k=0; k<2; k++) // k=0,1
		if (solutionExist && (solution[k] >= -2.35619) && (solution[k] <= 0))
		{
			Ik[s111+k*4]._q7 = Ik[s112+k*4]._q7 = Ik[s121+k*4]._q7 = Ik[s122+k*4]._q7 = solution[k];
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = true;
		}
		else
		{
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = false;
		}
	
	//solve q5
	cc = py5;
	cs = pz5;
	for(int i = 0; i<5; i+=4) //i = 0,4
		if (Ik[s111+i].valid)
		{
			c1 = - y67 - y89*cos(Ik[s111+i]._q7) + z8*sin(Ik[s111+i]._q7);
			solutionExist = IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
				if (solutionExist && (solution[k] >= -1.74533) && (solution[k] <= 1.39626 ))
				{
					Ik[s111+k*2+i]._q5 = Ik[s112+k*2+i]._q5 =  solution[k];
					Ik[s111+k*2+i].valid = Ik[s112+k*2+i].valid = true;
				}
				else
				{
					Ik[s111+k*2+i].valid = Ik[s112+k*2+i].valid = false;
				}
		
		}
	
	// solve q6
	int sgn;
	for(int j = 0; j<5; j+=4) //j = 0,4
	{
		//choose sign
		if ((Ik[s111+j].valid)||(Ik[s121+j].valid))
			sgn = ((z7 + z8*cos(Ik[s111+j]._q7) + y89*sin(Ik[s111+j]._q7) >= 0)? 1 : -1 );
	
		for(int i = 0; i<3; i+=2) //i = 0,2
			if (Ik[s111+i+j].valid)
			{
				solution[0] = atan2(sgn*px,sgn*(pz5*cos(Ik[s111+i+j]._q5) - py5*sin(Ik[s111+i+j]._q5)));
				if ( (solution[0] >= 0) && (solution[0] <= 3.14159 ))
				{
					Ik[s111+i+j]._q6 = Ik[s112+i+j]._q6 =  solution[0];
					Ik[s111+i+j].valid = Ik[s112+i+j].valid = true;
				}
				else
				{
					Ik[s111+i+j].valid = Ik[s112+i+j].valid = false;
				}
			}
	}
	
	
	// solve q9
	cc = sx;
	cs = -ax;
	for(int i=0; i<7; i+=2) //i = 0,2,4,6
		if (Ik[s111+i].valid)
		{
			c1 = - sin(Ik[s111+i]._q6)*sin(Ik[s111+i]._q7);
			solutionExist = IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
				if (solutionExist && (solution[k] >= -1.571) && (solution[k] <= 0.436 ))
				{
					Ik[s111+k+i]._q9 =  solution[k];
					Ik[s111+k+i].valid = true;
				}
				else
				{
					Ik[s111+k+i].valid = false;
				}
		}
	
	
	// solve q8
	for(int j = 0; j<5; j+=4) //j = 0,4
	{
		// choose sign
		if ((Ik[s111+j].valid)||(Ik[s112+j].valid)||(Ik[s121+j].valid)||(Ik[s122+j].valid))
			sgn = ((sin(Ik[s111+j]._q7) >= 0)? 1: -1);
	
		for(int i=0; i<4; i++)// i=0..3
			if(Ik[s111+i+j].valid)
			{
				solution[0] = atan2(sgn*(ny*cos(Ik[s111+i+j]._q5) + nz*sin(Ik[s111+i+j]._q5)),
					sgn*(-sin(Ik[s111+i+j]._q9)*(sy*cos(Ik[s111	+i+j]._q5) + sz*sin(Ik[s111+i+j]._q5)) - 
					cos(Ik[s111+i+j]._q9)*(ay*cos(Ik[s111+i+j]._q5) + az*sin(Ik[s111+i+j]._q5))));
				if ( (solution[0] >= -1.571) && (solution[0] <= 1.571 ))
				{
					Ik[s111+i+j]._q8 = solution[0];
					Ik[s111+i+j].valid = true;
				}
				else
				{
					Ik[s111+i+j].valid = false;
				}
			}
	}
	
	
	RPY R;
	double err = 1000; 	// big value
	int index = 10;		// big value	
	
	for(int i=s111; i<s222+1; i++)
	{
		if(Ik[i].valid)
		{
			R = Ik[i].Fk5Dof();
			Ik[i].error = RpyError(target,R);
			if (Ik[i].error <= err)
			{
				err = Ik[i].error;
				index  = i;
			}
		}
	}
	if (index < 10)
	{
		//std::cout << "error: " << Ik[index].error << std::endl;
		return Ik[index];
	}
	else
	{
		//std::cout << "No solution.\n";
		Ik[0].valid = false;
		return Ik[0];
	} 
	 
}

double RpyError(RPY R1, RPY R2)
{
	double err = 0;
	
	err += fabs(R1.x - R2.x);
	err += fabs(R1.y - R2.y);
	err += fabs(R1.z - R2.z);
	err += fabs(R1.R - R2.R);
	err += fabs(R1.P - R2.P);
	err += fabs(R1.Y - R2.Y);
	
	
	return err;
}


IkSolution SearchSolution(double mq1, double mq2, double mq3, RPY target)
{
	RPY Offset;
	IkSolution Ik, Ik_Best;
	Matrix T_Offset;
	double err = 1000;
	double mq4; 
	
	for(mq4 = -1.9; mq4<2.0; mq4+=0.01)
	{
		T_Offset = DestinationRhand(mq1, mq2, mq3, mq4, target);
		Offset.ToRPY(T_Offset);
		Ik = Ik5Dof(Offset);
		if ((Ik.valid)&&(Ik.error < err))
		{
			err = Ik.error;
			Ik_Best = Ik;
			Ik_Best._q4 = mq4;
		} 
	}
	
	if (err == 1000) 
		Ik_Best.valid = false;
	else
		std::cout << "error: " << Ik_Best.error << std::endl;
	
	return Ik_Best;
}












