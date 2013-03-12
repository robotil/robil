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
#define cc_q7r 2*(y67r*y89r + z7r*z8r)
#define cs_q7r 2*(y89r*z7r - y67r*z8r)
#define c1_q7r (y89r*y89r + z8r*z8r + y67r*y67r + z7r*z7r)
#define cc_q7l 2*(y67l*y89l + z7l*z8l)
#define cs_q7l 2*(y89l*z7l - y67l*z8l)
#define c1_q7l (y89l*y89l + z8l*z8l + y67l*y67l + z7l*z7l)


#define q4rMin -1.9
#define q4rMax 1.9
#define q5rMin -1.74533
#define q5rMax 1.39626
#define q6rMin 0
#define q6rMax 3.14159
#define q7rMin -2.35619
#define q7rMax 0	
#define q8rMin -1.571
#define q8rMax 1.571
#define q9rMin -1.571
#define q9rMax 0.436

#define q4lMin -1.9
#define q4lMax 1.9
#define q5lMin -1.39626
#define q5lMax 1.74533
#define q6lMin 0
#define q6lMax 3.14159
#define q7lMin 0
#define q7lMax 2.35619	
#define q8lMin -1.571
#define q8lMax 1.571
#define q9lMin -0.436
#define q9lMax 1.571

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
	IkSolution(double mq1,double mq2,double mq3,double mq4,double mq5,double mq6):
		m_q{mq1,mq2,mq3,mq4,mq5,mq6}{}
	RPY rFk5Dof();
	RPY lFk5Dof();
	void Print();
};

IkSolution rIk5Dof(RPY target); 
IkSolution lIk5Dof(RPY target); 
double RpyError(RPY R1, RPY R2);
IkSolution rSearchSolution(double mq1, double mq2, double mq3, RPY target);
IkSolution lSearchSolution(double mq1, double mq2, double mq3, RPY target);
Matrix rDest(double mq1,double mq2,double mq3,double mq4, RPY Target);
Matrix lDest(double mq1,double mq2,double mq3,double mq4, RPY Target);
RPY rPose(double mq1,double mq2,double mq3, IkSolution ik);
RPY lPose(double mq1,double mq2,double mq3, IkSolution ik);
void rFindAltSolution(double &mpx, double &mpy, double &mpz);
void lFindAltSolution(double &mpx, double &mpy, double &mpz);

#endif
