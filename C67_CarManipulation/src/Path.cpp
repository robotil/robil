/*
Path.cpp Find Trajectory
*/
#include <math.h>
#include <iostream>
#include <string>
#include <C67_CarManipulation/FK.h>
#include <C67_CarManipulation/IK.h>
#include <C67_CarManipulation/Path.h>

PathPoints::PathPoints(IkSolution R0,IkSolution L0,IkSolution R1,IkSolution L1)
{
	IkSolution rDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
		R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);
		
	IkSolution lDelta = IkSolution(L1.m_q[0]-L0.m_q[0], L1.m_q[1]-L0.m_q[1], L1.m_q[2]-L0.m_q[2],
		L1.m_q[3]-L0.m_q[3], L1.m_q[4]-L0.m_q[4], L1.m_q[5]-L0.m_q[5]);
	
	int i;
	for (i=0; i<N/4; i++)
	{
		Array[_r][i]._q4 = 8.0/3.0*rDelta._q4/(N*N)*(i+1)*(i+1) + R0._q4;
		Array[_r][i]._q5 = 8.0/3.0*rDelta._q5/(N*N)*(i+1)*(i+1) + R0._q5;
		Array[_r][i]._q6 = 8.0/3.0*rDelta._q6/(N*N)*(i+1)*(i+1) + R0._q6;
		Array[_r][i]._q7 = 8.0/3.0*rDelta._q7/(N*N)*(i+1)*(i+1) + R0._q7;
		Array[_r][i]._q8 = 8.0/3.0*rDelta._q8/(N*N)*(i+1)*(i+1) + R0._q8;
		Array[_r][i]._q9 = 8.0/3.0*rDelta._q9/(N*N)*(i+1)*(i+1) + R0._q9;
		
		Array[_l][i]._q4 = 8.0/3.0*lDelta._q4/(N*N)*(i+1)*(i+1) + L0._q4;
		Array[_l][i]._q5 = 8.0/3.0*lDelta._q5/(N*N)*(i+1)*(i+1) + L0._q5;
		Array[_l][i]._q6 = 8.0/3.0*lDelta._q6/(N*N)*(i+1)*(i+1) + L0._q6;
		Array[_l][i]._q7 = 8.0/3.0*lDelta._q7/(N*N)*(i+1)*(i+1) + L0._q7;
		Array[_l][i]._q8 = 8.0/3.0*lDelta._q8/(N*N)*(i+1)*(i+1) + L0._q8;
		Array[_l][i]._q9 = 8.0/3.0*lDelta._q9/(N*N)*(i+1)*(i+1) + L0._q9;
	} 
	
	for (; i<N*3/4; i++)
	{
		Array[_r][i]._q4 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q4/3 + R0._q4;
		Array[_r][i]._q5 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q5/3 + R0._q5;
		Array[_r][i]._q6 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q6/3 + R0._q6;
		Array[_r][i]._q7 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q7/3 + R0._q7;
		Array[_r][i]._q8 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q8/3 + R0._q8;
		Array[_r][i]._q9 = 4.0/3.0*rDelta._q4/N*(i+1) - rDelta._q9/3 + R0._q9;
		
		Array[_l][i]._q4 = 4.0/3.0*lDelta._q4/N*(i+1) - lDelta._q4/3 + L0._q4;
		Array[_l][i]._q5 = 4.0/3.0*lDelta._q5/N*(i+1) - lDelta._q5/3 + L0._q5;
		Array[_l][i]._q6 = 4.0/3.0*lDelta._q6/N*(i+1) - lDelta._q6/3 + L0._q6;
		Array[_l][i]._q7 = 4.0/3.0*lDelta._q7/N*(i+1) - lDelta._q7/3 + L0._q7;
		Array[_l][i]._q8 = 4.0/3.0*lDelta._q8/N*(i+1) - lDelta._q8/3 + L0._q8;
		Array[_l][i]._q9 = 4.0/3.0*lDelta._q9/N*(i+1) - lDelta._q9/3 + L0._q9;
	}
	
	for (; i<N; i++)
	{
		Array[_r][i]._q4 = -8.0/3.0*rDelta._q4/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q4/N*(i+1) - 2.5*rDelta._q4 + R0._q4;
		Array[_r][i]._q5 = -8.0/3.0*rDelta._q5/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q5/N*(i+1) - 2.5*rDelta._q5 + R0._q5;
		Array[_r][i]._q6 = -8.0/3.0*rDelta._q6/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q6/N*(i+1) - 2.5*rDelta._q6 + R0._q6;
		Array[_r][i]._q7 = -8.0/3.0*rDelta._q7/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q7/N*(i+1) - 2.5*rDelta._q7 + R0._q7;
		Array[_r][i]._q8 = -8.0/3.0*rDelta._q8/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q8/N*(i+1) - 2.5*rDelta._q8 + R0._q8;
		Array[_r][i]._q9 = -8.0/3.0*rDelta._q9/(N*N)*(i+1)*(i+1) +
			16.0/3.0*rDelta._q9/N*(i+1) - 2.5*rDelta._q9 + R0._q9;
		
		Array[_l][i]._q4 = -8.0/3.0*lDelta._q4/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q4/N*(i+1) - 2.5*lDelta._q4 + L0._q4;
		Array[_l][i]._q5 = -8.0/3.0*lDelta._q5/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q5/N*(i+1) - 2.5*lDelta._q5 + L0._q5;
		Array[_l][i]._q6 = -8.0/3.0*lDelta._q6/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q6/N*(i+1) - 2.5*lDelta._q6 + L0._q6;
		Array[_l][i]._q7 = -8.0/3.0*lDelta._q7/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q7/N*(i+1) - 2.5*lDelta._q7 + L0._q7;
		Array[_l][i]._q8 = -8.0/3.0*lDelta._q8/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q8/N*(i+1) - 2.5*lDelta._q8 + L0._q8;
		Array[_l][i]._q9 = -8.0/3.0*lDelta._q9/(N*N)*(i+1)*(i+1) +
			16.0/3.0*lDelta._q9/N*(i+1) - 2.5*lDelta._q9 + L0._q9;
	}
}

sPathPoints::sPathPoints(IkSolution R0,IkSolution R1)
{
	IkSolution sDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
		R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);
	
	int i;
	for (i=0; i<N/4; i++)
	{
		Array[i]._q4 = 8.0/3.0*sDelta._q4/(N*N)*(i+1)*(i+1) + R0._q4;
		Array[i]._q5 = 8.0/3.0*sDelta._q5/(N*N)*(i+1)*(i+1) + R0._q5;
		Array[i]._q6 = 8.0/3.0*sDelta._q6/(N*N)*(i+1)*(i+1) + R0._q6;
		Array[i]._q7 = 8.0/3.0*sDelta._q7/(N*N)*(i+1)*(i+1) + R0._q7;
		Array[i]._q8 = 8.0/3.0*sDelta._q8/(N*N)*(i+1)*(i+1) + R0._q8;
		Array[i]._q9 = 8.0/3.0*sDelta._q9/(N*N)*(i+1)*(i+1) + R0._q9;
	} 
	
	for (; i<N*3/4; i++)
	{
		Array[i]._q4 = 4.0/3.0*sDelta._q4/N*(i+1) - sDelta._q4/6 + R0._q4;
		Array[i]._q5 = 4.0/3.0*sDelta._q5/N*(i+1) - sDelta._q5/6 + R0._q5;
		Array[i]._q6 = 4.0/3.0*sDelta._q6/N*(i+1) - sDelta._q6/6 + R0._q6;
		Array[i]._q7 = 4.0/3.0*sDelta._q7/N*(i+1) - sDelta._q7/6 + R0._q7;
		Array[i]._q8 = 4.0/3.0*sDelta._q8/N*(i+1) - sDelta._q8/6 + R0._q8;
		Array[i]._q9 = 4.0/3.0*sDelta._q9/N*(i+1) - sDelta._q9/6 + R0._q9;
	}
	
	for (; i<N; i++)
	{
		Array[i]._q4 = -8.0/3.0*sDelta._q4/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q4/N*(i+1) - 5.0/3.0*sDelta._q4 + R0._q4;
		Array[i]._q5 = -8.0/3.0*sDelta._q5/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q5/N*(i+1) - 5.0/3.0*sDelta._q5 + R0._q5;
		Array[i]._q6 = -8.0/3.0*sDelta._q6/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q6/N*(i+1) - 5.0/3.0*sDelta._q6 + R0._q6;
		Array[i]._q7 = -8.0/3.0*sDelta._q7/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q7/N*(i+1) - 5.0/3.0*sDelta._q7 + R0._q7;
		Array[i]._q8 = -8.0/3.0*sDelta._q8/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q8/N*(i+1) - 5.0/3.0*sDelta._q8 + R0._q8;
		Array[i]._q9 = -8.0/3.0*sDelta._q9/(N*N)*(i+1)*(i+1) +
			16.0/3.0*sDelta._q9/N*(i+1) - 5.0/3.0*sDelta._q9 + R0._q9;
		
	}

	
}

pPathPoints::pPathPoints(IkSolution R0,IkSolution R1, int m_size, int option)
{
	size = m_size;
	pArray = new IkSolution[size];
	
	this->Update(R0, R1, option);

}

void pPathPoints::Update(IkSolution R0, IkSolution R1, int option)
{
	IkSolution pDelta = IkSolution(R1.m_q[0]-R0.m_q[0], R1.m_q[1]-R0.m_q[1], R1.m_q[2]-R0.m_q[2],
			R1.m_q[3]-R0.m_q[3], R1.m_q[4]-R0.m_q[4], R1.m_q[5]-R0.m_q[5]);

		int i, i0;
		if (option == StartEnd)
		{
			for (i=0; i<size/4; i++)
			{
				pArray[i]._q4 = 8.0/3.0*pDelta._q4/(size*size)*(i+1)*(i+1) + R0._q4;
				pArray[i]._q5 = 8.0/3.0*pDelta._q5/(size*size)*(i+1)*(i+1) + R0._q5;
				pArray[i]._q6 = 8.0/3.0*pDelta._q6/(size*size)*(i+1)*(i+1) + R0._q6;
				pArray[i]._q7 = 8.0/3.0*pDelta._q7/(size*size)*(i+1)*(i+1) + R0._q7;
				pArray[i]._q8 = 8.0/3.0*pDelta._q8/(size*size)*(i+1)*(i+1) + R0._q8;
				pArray[i]._q9 = 8.0/3.0*pDelta._q9/(size*size)*(i+1)*(i+1) + R0._q9;
			}

			for (; i<size*3/4; i++)
			{
				pArray[i]._q4 = 4.0/3.0*pDelta._q4/size*(i+1) - pDelta._q4/6 + R0._q4;
				pArray[i]._q5 = 4.0/3.0*pDelta._q5/size*(i+1) - pDelta._q5/6 + R0._q5;
				pArray[i]._q6 = 4.0/3.0*pDelta._q6/size*(i+1) - pDelta._q6/6 + R0._q6;
				pArray[i]._q7 = 4.0/3.0*pDelta._q7/size*(i+1) - pDelta._q7/6 + R0._q7;
				pArray[i]._q8 = 4.0/3.0*pDelta._q8/size*(i+1) - pDelta._q8/6 + R0._q8;
				pArray[i]._q9 = 4.0/3.0*pDelta._q9/size*(i+1) - pDelta._q9/6 + R0._q9;
			}

			for (; i<size; i++)
			{
				pArray[i]._q4 = -8.0/3.0*pDelta._q4/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q4/size*(i+1) - 5.0/3.0*pDelta._q4 + R0._q4;
				pArray[i]._q5 = -8.0/3.0*pDelta._q5/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q5/size*(i+1) - 5.0/3.0*pDelta._q5 + R0._q5;
				pArray[i]._q6 = -8.0/3.0*pDelta._q6/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q6/size*(i+1) - 5.0/3.0*pDelta._q6 + R0._q6;
				pArray[i]._q7 = -8.0/3.0*pDelta._q7/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q7/size*(i+1) - 5.0/3.0*pDelta._q7 + R0._q7;
				pArray[i]._q8 = -8.0/3.0*pDelta._q8/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q8/size*(i+1) - 5.0/3.0*pDelta._q8 + R0._q8;
				pArray[i]._q9 = -8.0/3.0*pDelta._q9/(size*size)*(i+1)*(i+1) +
					16.0/3.0*pDelta._q9/size*(i+1) - 5.0/3.0*pDelta._q9 + R0._q9;
			}
		}
		else if (option == NoStartEnd)
		{
			// constant velocity
			for (i=0; i<size; i++)
			{
				pArray[i]._q4 = pDelta._q4/size*(i+1) + R0._q4;
				pArray[i]._q5 = pDelta._q5/size*(i+1) + R0._q5;
				pArray[i]._q6 = pDelta._q6/size*(i+1) + R0._q6;
				pArray[i]._q7 = pDelta._q7/size*(i+1) + R0._q7;
				pArray[i]._q8 = pDelta._q8/size*(i+1) + R0._q8;
				pArray[i]._q9 = pDelta._q9/size*(i+1) + R0._q9;
			}
		}
		else if (option == NoEnd)
		{
			// constant acceleration
			for (i=0; i<size/2; i++)
			{
				pArray[i]._q4 = 4.0/6.0*pDelta._q4/(size*size)*(i+1)*(i+1) + R0._q4;
				pArray[i]._q5 = 4.0/6.0*pDelta._q5/(size*size)*(i+1)*(i+1) + R0._q5;
				pArray[i]._q6 = 4.0/6.0*pDelta._q6/(size*size)*(i+1)*(i+1) + R0._q6;
				pArray[i]._q7 = 4.0/6.0*pDelta._q7/(size*size)*(i+1)*(i+1) + R0._q7;
				pArray[i]._q8 = 4.0/6.0*pDelta._q8/(size*size)*(i+1)*(i+1) + R0._q8;
				pArray[i]._q9 = 4.0/6.0*pDelta._q9/(size*size)*(i+1)*(i+1) + R0._q9;
			}
			// constant velocity
			i0 = i;
			for (; i<size; i++)
			{
				pArray[i]._q4 = 4.0/3.0*pDelta._q4/size*(i+1-i0) + pDelta._q4/3.0 + R0._q4;
				pArray[i]._q5 = 4.0/3.0*pDelta._q5/size*(i+1-i0) + pDelta._q5/3.0 + R0._q5;
				pArray[i]._q6 = 4.0/3.0*pDelta._q6/size*(i+1-i0) + pDelta._q6/3.0 + R0._q6;
				pArray[i]._q7 = 4.0/3.0*pDelta._q7/size*(i+1-i0) + pDelta._q7/3.0 + R0._q7;
				pArray[i]._q8 = 4.0/3.0*pDelta._q8/size*(i+1-i0) + pDelta._q8/3.0 + R0._q8;
				pArray[i]._q9 = 4.0/3.0*pDelta._q9/size*(i+1-i0) + pDelta._q9/3.0 + R0._q9;
			}
		}
		else // No Start
		{
			// constant velocity
			for (i=0; i<size/2; i++)
			{
				pArray[i]._q4 = 4.0/3.0*pDelta._q4/size*(i+1) + R0._q4;
				pArray[i]._q5 = 4.0/3.0*pDelta._q5/size*(i+1) + R0._q5;
				pArray[i]._q6 = 4.0/3.0*pDelta._q6/size*(i+1) + R0._q6;
				pArray[i]._q7 = 4.0/3.0*pDelta._q7/size*(i+1) + R0._q7;
				pArray[i]._q8 = 4.0/3.0*pDelta._q8/size*(i+1) + R0._q8;
				pArray[i]._q9 = 4.0/3.0*pDelta._q9/size*(i+1) + R0._q9;
			}
			i0 = i+1;
			// constant deceleration
			for (; i<(size); i++)
			{
				pArray[i]._q4 =4.0/3.0*pDelta._q4/size*(i+1-i0) -4.0/6.0*pDelta._q4/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q4*2.0/3.0 + R0._q4;
				pArray[i]._q5 =4.0/3.0*pDelta._q5/size*(i+1-i0) -4.0/6.0*pDelta._q5/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q5*2.0/3.0 + R0._q5;
				pArray[i]._q6 =4.0/3.0*pDelta._q6/size*(i+1-i0) -4.0/6.0*pDelta._q6/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q6*2.0/3.0 + R0._q6;
				pArray[i]._q7 =4.0/3.0*pDelta._q7/size*(i+1-i0) -4.0/6.0*pDelta._q7/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q7*2.0/3.0 + R0._q7;
				pArray[i]._q8 =4.0/3.0*pDelta._q8/size*(i+1-i0) -4.0/6.0*pDelta._q8/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q8*2.0/3.0 + R0._q8;
				pArray[i]._q9 =4.0/3.0*pDelta._q9/size*(i+1-i0) -4.0/6.0*pDelta._q9/(size*size)*(i+1-i0)*(i+1-i0) +
						+ pDelta._q9*2.0/3.0 + R0._q9;

			}

		}

}
