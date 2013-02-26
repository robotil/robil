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


RPY IkSolution::rFk5Dof()
{
	RPY R;
	Matrix T = Matrix();
	T.rGet(q5r,_q5);
	T.rMultiply(q6r,_q6);
	T.rMultiply(q7r,_q7);
	T.rMultiply(q8r,_q8);
	T.rMultiply(q9r,_q9);
	R.ToRPY(T);
	return R;
}

RPY IkSolution::lFk5Dof()
{
	RPY R;
	Matrix T = Matrix();
	T.lGet(q5l,_q5);
	T.lMultiply(q6l,_q6);
	T.lMultiply(q7l,_q7);
	T.lMultiply(q8l,_q8);
	T.lMultiply(q9l,_q9);
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
IkSolution rIk5Dof(RPY target)
{
	Matrix A = target.FromRPY();
	double cc,cs,c1, solution[2];
	bool solutionExist;
	
	
	IkSolution Ik[8];
	
	//solve q7
	cc = 2*(y67r*y89r + z7r*z8r);
	cs = 2*(y89r*z7r - y67r*z8r);
	c1 = y89r*y89r + z8r*z8r + y67r*y67r + z7r*z7r - py5r*py5r - pz5r*pz5r - px*px;
	
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
	cc = py5r;
	cs = pz5r;
	for(int i = 0; i<5; i+=4) //i = 0,4
		if (Ik[s111+i].valid)
		{
			c1 = - y67r - y89r*cos(Ik[s111+i]._q7) + z8r*sin(Ik[s111+i]._q7);
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
			sgn = ((z7r + z8r*cos(Ik[s111+j]._q7) + y89r*sin(Ik[s111+j]._q7) >= 0)? 1 : -1 );
	
		for(int i = 0; i<3; i+=2) //i = 0,2
			if (Ik[s111+i+j].valid)
			{
				solution[0] = atan2(sgn*px,sgn*(pz5r*cos(Ik[s111+i+j]._q5) - py5r*sin(Ik[s111+i+j]._q5)));
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
			R = Ik[i].rFk5Dof();
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

IkSolution lIk5Dof(RPY target)
{
	Matrix A = target.FromRPY();
	double cc,cs,c1, solution[2];
	bool solutionExist;
	
	
	IkSolution Ik[8];
	
	//solve q7
	cc = 2*(y67l*y89l + z7l*z8l);
	cs = 2*(y89l*z7l - y67l*z8l);
	c1 = y89l*y89l + z8l*z8l + y67l*y67l + z7l*z7l - py5l*py5l - pz5l*pz5l - px*px;
	
	solutionExist = IkSqrSolve(cc,cs,c1,solution);
	
	for(int k=0; k<2; k++) // k=0,1
		if (solutionExist && (solution[k] >= 0) && (solution[k] <= 2.35619))
		{
			Ik[s111+k*4]._q7 = Ik[s112+k*4]._q7 = Ik[s121+k*4]._q7 = Ik[s122+k*4]._q7 = solution[k];
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = true;
		}
		else
		{
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = false;
		}
	
	//solve q5
	cc = py5l;
	cs = pz5l;
	for(int i = 0; i<5; i+=4) //i = 0,4
		if (Ik[s111+i].valid)
		{
			c1 = - y67l - y89l*cos(Ik[s111+i]._q7) + z8l*sin(Ik[s111+i]._q7);
			solutionExist = IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
				if (solutionExist && (solution[k] >= -1.39626) && (solution[k] <= 1.74533 ))
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
			sgn = ((z7l + z8l*cos(Ik[s111+j]._q7) + y89l*sin(Ik[s111+j]._q7) >= 0)? 1 : -1 );
	
		for(int i = 0; i<3; i+=2) //i = 0,2
			if (Ik[s111+i+j].valid)
			{
				solution[0] = atan2(sgn*px,sgn*(pz5l*cos(Ik[s111+i+j]._q5) - py5l*sin(Ik[s111+i+j]._q5)));
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
				if (solutionExist && (solution[k] >= -0.436) && (solution[k] <= 1.571 ))
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
			R = Ik[i].lFk5Dof();
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


IkSolution rSearchSolution(double mq1, double mq2, double mq3, RPY target)
{
	RPY Offset;
	IkSolution Ik, Ik_Best;
	Matrix T_Offset;
	double err = 1000;
	double mq4; 
	
	for(mq4 = -1.9; mq4<2.0; mq4+=0.01)
	{
		T_Offset = rDest(mq1, mq2, mq3, mq4, target);
		Offset.ToRPY(T_Offset);
		Ik = rIk5Dof(Offset);
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

IkSolution lSearchSolution(double mq1, double mq2, double mq3, RPY target)
{
	RPY Offset;
	IkSolution Ik, Ik_Best;
	Matrix T_Offset;
	double err = 1000;
	double mq4; 
	
	for(mq4 = -1.9; mq4<2.0; mq4+=0.01)
	{
		T_Offset = lDest(mq1, mq2, mq3, mq4, target);
		Offset.ToRPY(T_Offset);
		Ik = lIk5Dof(Offset);
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













