/*
Path.cpp Find Trajectory
*/
#include <math.h>
#include <iostream>
#include <string>
#include "FK.h"
#include "IK.h"
#include "Path.h"

PathPoints::PathPoints(IkSolution R0,IkSolution L0,IkSolution R1,IkSolution L1)
{
	IkSolution rDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
		R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);
		
	IkSolution lDelta = IkSolution(L1.m_q[0]-L0.m_q[0], L1.m_q[1]-L0.m_q[1], L1.m_q[2]-L0.m_q[2],
		L1.m_q[3]-L0.m_q[3], L1.m_q[4]-L0.m_q[4], L1.m_q[5]-L0.m_q[5]);
	
	int i;
	for (i=0; i<N/4; i++)
	{
		Array[_r][i]._q4 = 8.0/3.0*rDelta._q4/(N*N)*i*i + R0._q4;
		Array[_r][i]._q5 = 8.0/3.0*rDelta._q5/(N*N)*i*i + R0._q5;
		Array[_r][i]._q6 = 8.0/3.0*rDelta._q6/(N*N)*i*i + R0._q6;
		Array[_r][i]._q7 = 8.0/3.0*rDelta._q7/(N*N)*i*i + R0._q7;
		Array[_r][i]._q8 = 8.0/3.0*rDelta._q8/(N*N)*i*i + R0._q8;
		Array[_r][i]._q9 = 8.0/3.0*rDelta._q9/(N*N)*i*i + R0._q9;
		
		Array[_l][i]._q4 = 8.0/3.0*lDelta._q4/(N*N)*i*i + L0._q4;
		Array[_l][i]._q5 = 8.0/3.0*lDelta._q5/(N*N)*i*i + L0._q5;
		Array[_l][i]._q6 = 8.0/3.0*lDelta._q6/(N*N)*i*i + L0._q6;
		Array[_l][i]._q7 = 8.0/3.0*lDelta._q7/(N*N)*i*i + L0._q7;
		Array[_l][i]._q8 = 8.0/3.0*lDelta._q8/(N*N)*i*i + L0._q8;
		Array[_l][i]._q9 = 8.0/3.0*lDelta._q9/(N*N)*i*i + L0._q9;
	} 
	
	for (; i<N*3/4; i++)
	{
		Array[_r][i]._q4 = 4.0/3.0*rDelta._q4/N*i - rDelta._q4/3 + R0._q4;
		Array[_r][i]._q5 = 4.0/3.0*rDelta._q4/N*i - rDelta._q5/3 + R0._q5;
		Array[_r][i]._q6 = 4.0/3.0*rDelta._q4/N*i - rDelta._q6/3 + R0._q6;
		Array[_r][i]._q7 = 4.0/3.0*rDelta._q4/N*i - rDelta._q7/3 + R0._q7;
		Array[_r][i]._q8 = 4.0/3.0*rDelta._q4/N*i - rDelta._q8/3 + R0._q8;
		Array[_r][i]._q9 = 4.0/3.0*rDelta._q4/N*i - rDelta._q9/3 + R0._q9;
		
		Array[_l][i]._q4 = 4.0/3.0*lDelta._q4/N*i - lDelta._q4/3 + L0._q4;
		Array[_l][i]._q5 = 4.0/3.0*lDelta._q5/N*i - lDelta._q5/3 + L0._q5;
		Array[_l][i]._q6 = 4.0/3.0*lDelta._q6/N*i - lDelta._q6/3 + L0._q6;
		Array[_l][i]._q7 = 4.0/3.0*lDelta._q7/N*i - lDelta._q7/3 + L0._q7;
		Array[_l][i]._q8 = 4.0/3.0*lDelta._q8/N*i - lDelta._q8/3 + L0._q8;
		Array[_l][i]._q9 = 4.0/3.0*lDelta._q9/N*i - lDelta._q9/3 + L0._q9;
	}
	
	for (; i<N; i++)
	{
		Array[_r][i]._q4 = -8.0/3.0*rDelta._q4/(N*N)*i*i + 
			16.0/3.0*rDelta._q4/N*i - 2.5*rDelta._q4 + R0._q4;
		Array[_r][i]._q5 = -8.0/3.0*rDelta._q5/(N*N)*i*i + 
			16.0/3.0*rDelta._q5/N*i - 2.5*rDelta._q5 + R0._q5;
		Array[_r][i]._q6 = -8.0/3.0*rDelta._q6/(N*N)*i*i + 
			16.0/3.0*rDelta._q6/N*i - 2.5*rDelta._q6 + R0._q6;
		Array[_r][i]._q7 = -8.0/3.0*rDelta._q7/(N*N)*i*i + 
			16.0/3.0*rDelta._q7/N*i - 2.5*rDelta._q7 + R0._q7;
		Array[_r][i]._q8 = -8.0/3.0*rDelta._q8/(N*N)*i*i + 
			16.0/3.0*rDelta._q8/N*i - 2.5*rDelta._q8 + R0._q8;
		Array[_r][i]._q9 = -8.0/3.0*rDelta._q9/(N*N)*i*i + 
			16.0/3.0*rDelta._q9/N*i - 2.5*rDelta._q9 + R0._q9;
		
		Array[_l][i]._q4 = -8.0/3.0*lDelta._q4/(N*N)*i*i + 
			16.0/3.0*lDelta._q4/N*i - 2.5*lDelta._q4 + L0._q4;
		Array[_l][i]._q5 = -8.0/3.0*lDelta._q5/(N*N)*i*i + 
			16.0/3.0*lDelta._q5/N*i - 2.5*lDelta._q5 + L0._q5;
		Array[_l][i]._q6 = -8.0/3.0*lDelta._q6/(N*N)*i*i + 
			16.0/3.0*lDelta._q6/N*i - 2.5*lDelta._q6 + L0._q6;
		Array[_l][i]._q7 = -8.0/3.0*lDelta._q7/(N*N)*i*i + 
			16.0/3.0*lDelta._q7/N*i - 2.5*lDelta._q7 + L0._q7;
		Array[_l][i]._q8 = -8.0/3.0*lDelta._q8/(N*N)*i*i + 
			16.0/3.0*lDelta._q8/N*i - 2.5*lDelta._q8 + L0._q8;
		Array[_l][i]._q9 = -8.0/3.0*lDelta._q9/(N*N)*i*i + 
			16.0/3.0*lDelta._q9/N*i - 2.5*lDelta._q9 + L0._q9;
	}
}

sPathPoints::sPathPoints(IkSolution R0,IkSolution R1)
{
	IkSolution sDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
		R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);
	
	int i;
	for (i=0; i<N/4; i++)
	{
		Array[i]._q4 = 8.0/3.0*sDelta._q4/(N*N)*i*i + R0._q4;
		Array[i]._q5 = 8.0/3.0*sDelta._q5/(N*N)*i*i + R0._q5;
		Array[i]._q6 = 8.0/3.0*sDelta._q6/(N*N)*i*i + R0._q6;
		Array[i]._q7 = 8.0/3.0*sDelta._q7/(N*N)*i*i + R0._q7;
		Array[i]._q8 = 8.0/3.0*sDelta._q8/(N*N)*i*i + R0._q8;
		Array[i]._q9 = 8.0/3.0*sDelta._q9/(N*N)*i*i + R0._q9;
	} 
	
	for (; i<N*3/4; i++)
	{
		Array[i]._q4 = 4.0/3.0*sDelta._q4/N*i - sDelta._q4/6 + R0._q4;
		Array[i]._q5 = 4.0/3.0*sDelta._q5/N*i - sDelta._q5/6 + R0._q5;
		Array[i]._q6 = 4.0/3.0*sDelta._q6/N*i - sDelta._q6/6 + R0._q6;
		Array[i]._q7 = 4.0/3.0*sDelta._q7/N*i - sDelta._q7/6 + R0._q7;
		Array[i]._q8 = 4.0/3.0*sDelta._q8/N*i - sDelta._q8/6 + R0._q8;
		Array[i]._q9 = 4.0/3.0*sDelta._q9/N*i - sDelta._q9/6 + R0._q9;
	}
	
	for (; i<N; i++)
	{
		Array[i]._q4 = -8.0/3.0*sDelta._q4/(N*N)*i*i + 
			16.0/3.0*sDelta._q4/N*i - 5.0/3.0*sDelta._q4 + R0._q4;
		Array[i]._q5 = -8.0/3.0*sDelta._q5/(N*N)*i*i + 
			16.0/3.0*sDelta._q5/N*i - 5.0/3.0*sDelta._q5 + R0._q5;
		Array[i]._q6 = -8.0/3.0*sDelta._q6/(N*N)*i*i + 
			16.0/3.0*sDelta._q6/N*i - 5.0/3.0*sDelta._q6 + R0._q6;
		Array[i]._q7 = -8.0/3.0*sDelta._q7/(N*N)*i*i + 
			16.0/3.0*sDelta._q7/N*i - 5.0/3.0*sDelta._q7 + R0._q7;
		Array[i]._q8 = -8.0/3.0*sDelta._q8/(N*N)*i*i + 
			16.0/3.0*sDelta._q8/N*i - 5.0/3.0*sDelta._q8 + R0._q8;
		Array[i]._q9 = -8.0/3.0*sDelta._q9/(N*N)*i*i + 
			16.0/3.0*sDelta._q9/N*i - 5.0/3.0*sDelta._q9 + R0._q9;
		
	}

	
}

pPathPoints::pPathPoints(IkSolution R0,IkSolution R1, int m_size)
{
	size = m_size;
	pArray = new IkSolution[size];
	
	IkSolution pDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
		R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);
	
	int i;
	for (i=0; i<size/4; i++)
	{
		pArray[i]._q4 = 8.0/3.0*pDelta._q4/(size*size)*i*i + R0._q4;
		pArray[i]._q5 = 8.0/3.0*pDelta._q5/(size*size)*i*i + R0._q5;
		pArray[i]._q6 = 8.0/3.0*pDelta._q6/(size*size)*i*i + R0._q6;
		pArray[i]._q7 = 8.0/3.0*pDelta._q7/(size*size)*i*i + R0._q7;
		pArray[i]._q8 = 8.0/3.0*pDelta._q8/(size*size)*i*i + R0._q8;
		pArray[i]._q9 = 8.0/3.0*pDelta._q9/(size*size)*i*i + R0._q9;
	} 
	
	for (; i<size*3/4; i++)
	{
		pArray[i]._q4 = 4.0/3.0*pDelta._q4/size*i - pDelta._q4/6 + R0._q4;
		pArray[i]._q5 = 4.0/3.0*pDelta._q5/size*i - pDelta._q5/6 + R0._q5;
		pArray[i]._q6 = 4.0/3.0*pDelta._q6/size*i - pDelta._q6/6 + R0._q6;
		pArray[i]._q7 = 4.0/3.0*pDelta._q7/size*i - pDelta._q7/6 + R0._q7;
		pArray[i]._q8 = 4.0/3.0*pDelta._q8/size*i - pDelta._q8/6 + R0._q8;
		pArray[i]._q9 = 4.0/3.0*pDelta._q9/size*i - pDelta._q9/6 + R0._q9;
	}
	
	for (; i<size; i++)
	{
		pArray[i]._q4 = -8.0/3.0*pDelta._q4/(size*size)*i*i + 
			16.0/3.0*pDelta._q4/size*i - 5.0/3.0*pDelta._q4 + R0._q4;
		pArray[i]._q5 = -8.0/3.0*pDelta._q5/(size*size)*i*i + 
			16.0/3.0*pDelta._q5/size*i - 5.0/3.0*pDelta._q5 + R0._q5;
		pArray[i]._q6 = -8.0/3.0*pDelta._q6/(size*size)*i*i + 
			16.0/3.0*pDelta._q6/size*i - 5.0/3.0*pDelta._q6 + R0._q6;
		pArray[i]._q7 = -8.0/3.0*pDelta._q7/(size*size)*i*i + 
			16.0/3.0*pDelta._q7/size*i - 5.0/3.0*pDelta._q7 + R0._q7;
		pArray[i]._q8 = -8.0/3.0*pDelta._q8/(size*size)*i*i + 
			16.0/3.0*pDelta._q8/size*i - 5.0/3.0*pDelta._q8 + R0._q8;
		pArray[i]._q9 = -8.0/3.0*pDelta._q9/(size*size)*i*i + 
			16.0/3.0*pDelta._q9/size*i - 5.0/3.0*pDelta._q9 + R0._q9;
		
	}

	
}
