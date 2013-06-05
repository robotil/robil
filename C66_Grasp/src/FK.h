//FK.h
#ifndef FK_H
#define FK_H

#define dx 0][3
#define dy 1][3
#define dz 2][3
#define x1 (-0.0125)
#define z2 0.09
#define z3 0.05
#define ryr 0.5
#define rzr (-0.866025)
#define x4r 0.024
#define y4r (-0.221)
#define z4r 0.289
#define y5r (-0.075)
#define z5r 0.036
#define y6r (-0.185)
#define y7r (-0.121)
#define z7r 0.013
#define y8r (-0.188)
#define z8r (-0.013)
#define y9r (-0.058)
#define y67r (y6r + y7r)
#define y89r (y8r + y9r)
#define ryl 0.5
#define rzl 0.866025
#define x4l 0.024
#define y4l 0.221
#define z4l 0.289
#define y5l 0.075
#define z5l 0.036
#define y6l 0.185
#define y7l 0.121
#define z7l 0.013
#define y8l 0.188
#define z8l (-0.013)
#define y9l 0.058
#define y67l (y6l + y7l)
#define y89l (y8l + y9l)

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
	q4l = l_arm_usy,
	q5l = l_arm_shx,
	q6l = l_arm_ely,
	q7l = l_arm_elx,
	q8l = l_arm_uwy,
	q9l = l_arm_mwx,
	q4r = r_arm_usy,
	q5r = r_arm_shx,
	q6r = r_arm_ely,
	q7r = r_arm_elx,
	q8r = r_arm_uwy,
	q9r = r_arm_mwx
	
};


class Matrix{
	//double q[10];	
	
	void rUpdate(double A[4][4], int Type, double val);
	void lUpdate(double A[4][4], int Type, double val);
	void RotateX(double A[4][4], double val);
	void RotateY(double A[4][4], double val);
	void RotateZ(double A[4][4], double val);
	void rRotateYZ(double A[4][4], double val);
	void lRotateYZ(double A[4][4], double val);
public:
	double T[4][4];	
	Matrix(){}
	Matrix(double A[4][4]);
	void rGet(int Type, double val);
	void lGet(int Type, double val);
	void rMultiply(int Type, double val);
	void lMultiply(int Type, double val);
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

class Quarternion{
public:
	double x, y, z, a, b, c, d;
	Quarternion (){};
	Quarternion(double m_x, double m_y, double m_z, double m_a, double m_b, double m_c, double m_d);

	Matrix ToMatrix();
	void Print();
};
Quarternion QuarAngle(double m_x, double m_y, double m_z, double angle, double ax, double ay, double az);

#endif
