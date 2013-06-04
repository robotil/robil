/*
IK.cpp Inverse Kinematics file
*/
#include <math.h>
#include <iostream>
#include <string>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>

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
	double x;	
	double m = - c1*c1 + cc*cc + cs*cs;

	if (c1 == cc)
	{
		angle[0] = M_PI;
		angle[1] = -M_PI;
		if (m < 0) return false;
		return true;
	}

	if (m < 0)
	{
		x = -(cs)/(c1 - cc);
		angle[0] = angle[1] = atan2(2*x,1-x*x);		
		return false;
	}
	
	
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
	std::cout << "q4-q9:"<< _q4<< ", "<< _q5<< ", "<< _q6<< ", "<< _q7<< ", "<< _q8<< ", "<< _q9<< std::endl;
}
IkSolution rIk5Dof(RPY target)
{
	Matrix A = target.FromRPY();
	double cc, cs, c1, solution[2];
	bool solutionExist;
	
	
	IkSolution Ik[8];
	
	//solve q7
	cc = cc_q7r;
	cs = cs_q7r;
	c1 = c1_q7r - py5r*py5r - pz5r*pz5r - px*px;
	
	solutionExist = IkSqrSolve(cc,cs,c1,solution);
	if (!solutionExist)
	{
		rFindAltSolution(px,py,pz);
		c1 = c1_q7r - py5r*py5r - pz5r*pz5r - px*px;
		solutionExist = IkSqrSolve(cc,cs,c1,solution);
	}
	
	for(int k=0; k<2; k++) // k=0,1
		//if (solutionExist && (solution[k] >= -2.35619) && (solution[k] <= 0))
		if (solutionExist)
		{
			solution[k] = (solution[k] < q7rMin)? q7rMin: solution[k];	
			solution[k] = (solution[k] > q7rMax)? q7rMax: solution[k];			
			Ik[s111+k*4]._q7 = Ik[s112+k*4]._q7 = Ik[s121+k*4]._q7 = Ik[s122+k*4]._q7 = solution[k];
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = true;
		}
		else
		{
			Ik[s111+k*4].valid = Ik[s112+k*4].valid = Ik[s121+k*4].valid = Ik[s122+k*4].valid = false;
		}
	
	//if (!solutionExist) std::cout << "No solution q7\n";
	//solve q5
	cc = py5r;
	cs = pz5r;
	for(int i = 0; i<5; i+=4) //i = 0,4
		if (Ik[s111+i].valid)
		{
			c1 = - y67r - y89r*cos(Ik[s111+i]._q7) + z8r*sin(Ik[s111+i]._q7);
			solutionExist = IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
				//if (solutionExist && (solution[k] >= -1.74533) && (solution[k] <= 1.39626 ))
				if (solutionExist)				
				{
					solution[k] = (solution[k] < q5rMin)? q5rMin: solution[k];	
					solution[k] = (solution[k] > q5rMax)? q5rMax: solution[k];					
					Ik[s111+k*2+i]._q5 = Ik[s112+k*2+i]._q5 =  solution[k];
					Ik[s111+k*2+i].valid = Ik[s112+k*2+i].valid = true;
				}
				else
				{
					Ik[s111+k*2+i].valid = Ik[s112+k*2+i].valid = false;
				}
		
		}
		//if ((!Ik[s111].valid)&&(!Ik[s121].valid)&&(!Ik[s211].valid)&&(!Ik[s221].valid))
			//std::cout << "No solution q5\n";
	// solve q6
	int sgn = 1;
	for(int j = 0; j<5; j+=4) //j = 0,4
	{
		//choose sign
		if ((Ik[s111+j].valid)||(Ik[s121+j].valid))
			sgn = ((z7r + z8r*cos(Ik[s111+j]._q7) + y89r*sin(Ik[s111+j]._q7) >= 0)? 1 : -1 );
	
		for(int i = 0; i<3; i+=2) //i = 0,2
			if (Ik[s111+i+j].valid)
			{
				solution[0] = atan2(sgn*px,sgn*(pz5r*cos(Ik[s111+i+j]._q5) - py5r*sin(Ik[s111+i+j]._q5)));
				solution[0] = (solution[0] < q6rMin)? q6rMin: solution[0];	
				solution[0] = (solution[0] > q6rMax)? q6rMax: solution[0];					
				Ik[s111+i+j]._q6 = Ik[s112+i+j]._q6 =  solution[0];
				Ik[s111+i+j].valid = Ik[s112+i+j].valid = true;
				
			}
	}
	
	
	// solve q9
	cc = sx;
	cs = -ax;
	for(int i=0; i<7; i+=2) //i = 0,2,4,6
		if (Ik[s111+i].valid)
		{
			c1 = - sin(Ik[s111+i]._q6)*sin(Ik[s111+i]._q7);
			IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
			{
				solution[k] = (solution[k] < q9rMin)? q9rMin: solution[k];	
				solution[k] = (solution[k] > q9rMax)? q9rMax: solution[k];					
				Ik[s111+k+i]._q9 =  solution[k];
				Ik[s111+k+i].valid = true;
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
				
				solution[0] = (solution[0] < q8rMin)? q8rMin: solution[0];	
				solution[0] = (solution[0] > q8rMax)? q8rMax: solution[0];					
				Ik[s111+i+j]._q8 = solution[0];
				Ik[s111+i+j].valid = true;
				
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
	cc = cc_q7l;
	cs = cs_q7l;
	c1 = c1_q7l - py5l*py5l - pz5l*pz5l - px*px;
	
	solutionExist = IkSqrSolve(cc,cs,c1,solution);
	if (!solutionExist)
	{
		lFindAltSolution(px,py,pz);
		c1 = c1_q7l - py5l*py5l - pz5l*pz5l - px*px;
		solutionExist = IkSqrSolve(cc,cs,c1,solution);
	}
	for(int k=0; k<2; k++) // k=0,1
		//if (solutionExist && (solution[k] >= 0) && (solution[k] <= 2.35619))
		if (solutionExist)
		{
			solution[k] = (solution[k] < q7lMin)? q7lMin: solution[k];	
			solution[k] = (solution[k] > q7lMax)? q7lMax: solution[k];			
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
				//if (solutionExist && (solution[k] >= -1.39626) && (solution[k] <= 1.74533 ))
				if (solutionExist)				
				{
					solution[k] = (solution[k] < q5lMin)? q5lMin: solution[k];	
					solution[k] = (solution[k] > q5lMax)? q5lMax: solution[k];					
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
				
				solution[0] = (solution[0] < q6lMin)? q6lMin: solution[0];	
				solution[0] = (solution[0] > q6lMax)? q6lMax: solution[0];					
				Ik[s111+i+j]._q6 = Ik[s112+i+j]._q6 =  solution[0];
				Ik[s111+i+j].valid = Ik[s112+i+j].valid = true;
				
			}
	}
	
	
	// solve q9
	cc = sx;
	cs = -ax;
	for(int i=0; i<7; i+=2) //i = 0,2,4,6
		if (Ik[s111+i].valid)
		{
			c1 = - sin(Ik[s111+i]._q6)*sin(Ik[s111+i]._q7);
			IkSqrSolve(cc,cs,c1,solution);
			for(int k=0; k<2; k++) // k=0,1
				//if (solutionExist && (solution[k] >= -0.436) && (solution[k] <= 1.571 ))
			{	
				solution[k] = (solution[k] < q9lMin)? q9lMin: solution[k];	
				solution[k] = (solution[k] > q9lMax)? q9lMax: solution[k];					
				Ik[s111+k+i]._q9 =  solution[k];
				Ik[s111+k+i].valid = true;
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
				//if ( (solution[0] >= -1.571) && (solution[0] <= 1.571 ))
				solution[0] = (solution[0] < q8lMin)? q8lMin: solution[0];	
				solution[0] = (solution[0] > q8lMax)? q8lMax: solution[0];
				Ik[s111+i+j]._q8 = solution[0];
				Ik[s111+i+j].valid = true;
				
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
	err += 0.2*fabs(R1.R - R2.R);
	err += 0.2*fabs(R1.P - R2.P);
	err += 0.2*fabs(R1.Y - R2.Y);
	
	
	return err;
}


IkSolution rSearchSolution(double mq1, double mq2, double mq3, RPY target)
{
	RPY Offset;
	IkSolution Ik, Ik_Best;
	Matrix T_Offset;
	double err = 1000;
	double mq4; 
	
	for(mq4 = -1.9; mq4<=1.9; mq4+=0.01)
	//for(mq4 = 0; mq4<.1; mq4+=0.1)
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
	//else	
	//	std::cout << "error: " << Ik_Best.error << std::endl;
	
	return Ik_Best;
}

IkSolution lSearchSolution(double mq1, double mq2, double mq3, RPY target)
{
	RPY Offset;
	IkSolution Ik, Ik_Best;
	Matrix T_Offset;
	double err = 1000;
	double mq4; 
	
	for(mq4 = -1.9; mq4<=1.9; mq4+=0.01)
	//for(mq4 = 0; mq4<.1; mq4+=0.1)
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
	//else
	//	std::cout << "error: " << Ik_Best.error << std::endl;
	
	return Ik_Best;
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

	// Transition To Sandia Palm
	RPY SPr = RPY(rHandToPalm);
	Matrix S = SPr.FromRPY();
	// Transition to Sandia f1 base - middle finger base
	RPY f1r = RPY(rPalmToF1);
	Matrix f1m = f1r.FromRPY();
	S.Multiply(f1m);
	S.Inverse();
	T.Multiply(S);

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

	// Transition To Sandia Palm
	RPY SPr = RPY(lHandToPalm);
	Matrix S = SPr.FromRPY();
	// Transition to Sandia f1 base - middle finger base
	RPY f1r = RPY(lPalmToF1);
	Matrix f1m = f1r.FromRPY();
	S.Multiply(f1m);
	S.Inverse();
	T.Multiply(S);

	return T;
}

RPY rPose(double mq1,double mq2,double mq3, IkSolution ik)
{
	Matrix T = Matrix();
	T.rGet(q1,mq1);
	T.rMultiply(q2,mq2);
	T.rMultiply(q3,mq3);
	T.rMultiply(q4r,ik._q4);
	T.rMultiply(q5r,ik._q5);
	T.rMultiply(q6r,ik._q6);
	T.rMultiply(q7r,ik._q7);
	T.rMultiply(q8r,ik._q8);
	T.rMultiply(q9r,ik._q9);

	// Transition to Sandia Palm
	RPY SPr = RPY(rHandToPalm);
	Matrix SPm = SPr.FromRPY();
	T.Multiply(SPm);
	// Transition to Sandia f1 base - middle finger base
	RPY f1r = RPY(rPalmToF1);
	Matrix f1m = f1r.FromRPY();
	T.Multiply(f1m);

	RPY r;
	r.ToRPY(T);
	return r;
}

RPY lPose(double mq1,double mq2,double mq3, IkSolution ik)
{
	Matrix T = Matrix();
	T.lGet(q1,mq1);
	T.lMultiply(q2,mq2);
	T.lMultiply(q3,mq3);
	T.lMultiply(q4l,ik._q4);
	T.lMultiply(q5l,ik._q5);
	T.lMultiply(q6l,ik._q6);
	T.lMultiply(q7l,ik._q7);
	T.lMultiply(q8l,ik._q8);
	T.lMultiply(q9l,ik._q9);

	// Transition to Sandia Palm
	RPY SPr = RPY(lHandToPalm);
	Matrix SPm = SPr.FromRPY();
	T.Multiply(SPm);
	// Transition to Sandia f1 base - middle finger base
	RPY f1r = RPY(lPalmToF1);
	Matrix f1m = f1r.FromRPY();
	T.Multiply(f1m);

	RPY r;
	r.ToRPY(T);
	return r;
}

void rFindAltSolution(double &mpx, double &mpy, double &mpz )
{
	double cc7, cs7, c17, mpy5, mpz5, l, m, n, d, R;
	mpy5 = mpy - y5r;
	mpz5 = mpz - z5r;
	cc7 = cc_q7r;
	cs7 = cs_q7r;
	c17 = c1_q7r - mpy5*mpy5 - mpz5*mpz5 - mpx*mpx;
	d = sqrt(mpy5*mpy5 + mpz5*mpz5 + mpx*mpx);
	l = mpx/d; m = mpy5/d, n = mpz5/d;

	if (c17 >= 0)
		R = sqrt(c1_q7r - sqrt(cc7*cc7 + cs7*cs7)) + 0.0001;
	else
		R = sqrt(c1_q7r + sqrt(cc7*cc7 + cs7*cs7)) - 0.0001;

	mpx = l*R; mpy = m*R + y5r; mpz = n*R + z5r;

}
void lFindAltSolution(double &mpx, double &mpy, double &mpz )
{
	double cc7, cs7, c17, mpy5, mpz5, l, m, n, d, R;
	mpy5 = mpy - y5l;
	mpz5 = mpz - z5l;
	cc7 = cc_q7l;
	cs7 = cs_q7l;
	c17 = c1_q7l - mpy5*mpy5 - mpz5*mpz5 - mpx*mpx;
	d = sqrt(mpy5*mpy5 + mpz5*mpz5 + mpx*mpx);
	l = mpx/d; m = mpy5/d, n = mpz5/d;

	if (c17 >= 0)
		R = sqrt(c1_q7l - sqrt(cc7*cc7 + cs7*cs7)) + 0.0001;
	else
		R = sqrt(c1_q7l + sqrt(cc7*cc7 + cs7*cs7)) - 0.0001;

	mpx = l*R; mpy = m*R + y5l; mpz = n*R + z5l;

}

IkSolution ScanRPY(double mq1, double mq2, double mq3, RPY target, double error, bool rightSide)
{
	double i,j,k,R0,P0,Y0;
	IkSolution solution;
	bool valid = false;
	R0 = target.R;
	P0 = target.P;
	Y0 = target.Y;
	for (i = 0;i < M_PI/2; i+=0.1)
	{
		for (j = 0; j<M_PI/2; j+=0.1)
		{
			for (k = 0; k<M_PI/2; k+=0.1)
			{
				target.R = R0 + i;
				target.P = P0 + j;
				target.Y = Y0 + k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 + i;
				target.P = P0 + j;
				target.Y = Y0 - k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 + i;
				target.P = P0 - j;
				target.Y = Y0 + k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 + i;
				target.P = P0 - j;
				target.Y = Y0 - k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 - i;
				target.P = P0 + j;
				target.Y = Y0 + k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 - i;
				target.P = P0 + j;
				target.Y = Y0 - k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 - i;
				target.P = P0 - j;
				target.Y = Y0 + k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
				target.R = R0 - i;
				target.P = P0 - j;
				target.Y = Y0 - k;
				solution = rightSide? rSearchSolution(mq1, mq2, mq3, target): lSearchSolution(mq1, mq2, mq3, target);
				if (solution.error < error)
				{
					valid = true;
					break;
				}
			}
			if (valid) break;
		}
		if (valid) break;
	}
	solution.valid = valid;

	return solution;
}

IkSolution rScanRPY(double mq1, double mq2, double mq3, RPY target, double error)
{
	return ScanRPY(mq1, mq2, mq3, target, error, true);
}

IkSolution lScanRPY(double mq1, double mq2, double mq3, RPY target, double error)
{
	return ScanRPY(mq1, mq2, mq3, target, error, false);
}


