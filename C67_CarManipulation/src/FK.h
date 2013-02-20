//FK.h
#ifndef FK_H
#define FK_H

#define dx 0][3
#define dy 1][3
#define dz 2][3
#define x1 -0.0125
#define z2 0.09
#define z3 0.05
#define ry 0.5
#define rz -0.866
#define x4 0.024
#define y4 -0.221
#define z4 0.289
#define y5 (-0.075)
#define z5 0.036
#define y6 (-0.185)
#define y7 (-0.121)
#define z7 0.013
#define y8 (-0.188)
#define z8 (-0.013)
#define y9 (-0.058)
#define y67 (y6 + y7)
#define y89 (y8 + y9)

enum {
	back_lbz,
	back_mby,
	back_ubx,
	neck_ay,
	l_leg_uhz,
	l_leg_mhx,
	l_leg_lhy,
	l_leg_kny,
	l_leg_uay,
	l_leg_lax,
	r_leg_uhz,
	r_leg_mhx,
	r_leg_lhy,
	r_leg_kny,
	r_leg_uay,
	r_leg_lax,
	l_arm_usy,
	l_arm_shx,
	l_arm_ely,
	l_arm_elx,
	l_arm_uwy,
	l_arm_mwx,
	r_arm_usy,
	r_arm_shx,
	r_arm_ely,
	r_arm_elx,
	r_arm_uwy,
	r_arm_mwx
};
enum {
	q1 = back_lbz,
	q2 = back_mby,
	q3 = back_ubx,
	q4 = r_arm_usy,
	q5 = r_arm_shx,
	q6 = r_arm_ely,
	q7 = r_arm_elx,
	q8 = r_arm_uwy,
	q9 = r_arm_mwx
	
};


class Matrix{
	//double q[10];	
	
	void Update(double A[4][4], int Type, double val);
	void RotateX(double A[4][4], double val);
	void RotateY(double A[4][4], double val);
	void RotateZ(double A[4][4], double val);
	void RotateYZ(double A[4][4], double val);
public:
	double T[4][4];	
	Matrix(){}
	void Get(int Type, double val);
	void Multiply(int Type, double val);
	void Multiply(Matrix A);
	void Inverse();
	void Print(); 
};

class RPY{
public:
	double x,y,z,R,P,Y;
	RPY(){}
	RPY(double nx, double ny, double nz, double nR, double nP, double nY)
		{x = nx; y=ny; z=nz, R=nR; P=nP; Y=nY;}
	void ToRPY(Matrix T);
	Matrix FromRPY();
	void Print();
};

Matrix DestinationRhand(double mq1,double mq2,double mq3,double mq4, RPY Target);


#endif
