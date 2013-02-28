//IK.h
#ifndef IK_H
#define IK_H

//#include "FK.h"
#define _USE_MATH_DEFINES
#define nx A.T[0][0]
#define	ny A.T[1][0]
#define	nz A.T[2][0]
#define	sx A.T[0][1]
#define	sy A.T[1][1]
#define	sz A.T[2][1]
#define	ax A.T[0][2]
#define	ay A.T[1][2]
#define	az A.T[2][2]
#define	px A.T[0][3]
#define	py A.T[1][3]
#define	pz A.T[2][3]
#define py5r (py - y5r)
#define pz5r (pz - z5r)
#define py5l (py - y5l)
#define pz5l (pz - z5l)
#define _q4	m_q[0]
#define _q5	m_q[1]
#define _q6	m_q[2]
#define _q7	m_q[3]
#define _q8	m_q[4]
#define _q9	m_q[5]

enum{
	s111,
	s112,
	s121,
	s122,
	s211,
	s212,
	s221,
	s222
};

class IkSolution{
public:
	double m_q[6];
	double error;
	bool valid;
	IkSolution(){}
	RPY rFk5Dof();
	RPY lFk5Dof();
	void Print();
};

IkSolution rIk5Dof(RPY target); 
IkSolution lIk5Dof(RPY target); 
double RpyError(RPY R1, RPY R2);
IkSolution rSearchSolution(double mq1, double mq2, double mq3, RPY target);
IkSolution lSearchSolution(double mq1, double mq2, double mq3, RPY target);




#endif
