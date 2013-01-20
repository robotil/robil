#ifndef ____VEC2D____H_
#define ____VEC2D____H_

#include <math.h>
#include <iostream>

using namespace  std;

#define PI 3.14159265359
#define PI05 1.57079632679
#define PI2 6.28318530718

class Vec2d{
public:
	
	static const double eps = 0.0001;
	
	double x;
	double y;
	
	bool isNull;
	
	Vec2d(double x=0, double y=0):x(x),y(y),isNull(false){}
// 	Vec2d(size_t x, size_t y):x((double)x),y((double)y),isNull(false){}
// 	Vec2d(long x, long y):x((double)x),y((double)y),isNull(false){}
	Vec2d(const Vec2d& v):x(v.x),y(v.y),isNull(v.isNull){}
	Vec2d& operator=(const Vec2d& v){ x=v.x; y=v.y; isNull=v.isNull; return *this; }
	bool operator==(const Vec2d& v)const{ return (v-(*this)).len()<eps || (isNull && v.isNull); }
	bool operator!=(const Vec2d& v)const{ return !((*this)==v); }
	
	Vec2d clone()const{ return Vec2d(x,y); }
	static Vec2d poliar(double a, double l){ return Vec2d(l*cos(a), l*sin(a)); }
	
		
	long int_x()const{ return (long)round(x); }
	long int_y()const{ return (long)round(y); }
	
	double len()const{ return hypot(x,y); }
	double ang()const{ return atan2(y,x); }
	double angY()const{ return poliar(ang()-PI05,1).ang(); }
	
	Vec2d operator+(const Vec2d& v)const{ return Vec2d(x+v.x, y+v.y); }
	Vec2d operator-(const Vec2d& v)const{ return Vec2d(x-v.x, y-v.y); }
    Vec2d rotate(double a)const{ return poliar(ang()+a,len()); }
    Vec2d changeAng(double a)const{ return poliar(a,len()); }
    Vec2d changeLen(double a)const{ return poliar(ang(),a); }
    Vec2d changeX(double a)const{ return Vec2d(a,y); }
    Vec2d changeY(double a)const{ return Vec2d(x,a); }
    Vec2d scale(double fx, double fy)const{ return Vec2d(x*fx, y*fy); }
    Vec2d scale(double f)const{ return scale(f,f); }
    Vec2d operator*(double f)const{ return scale(f,f); }
    Vec2d norm()const{ return Vec2d(x/len(),y/len()); }
	double dot(const Vec2d &vect) const{return (x * vect.x) + (y * vect.y);}
	double cross(const Vec2d &vect2) const{return (this->x * vect2.y) - (this->y * vect2.x);}
    bool inTriangle(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3)const {
		const Vec2d& pt = *this;
		double AB = (pt.y-p1.y)*(p2.x-p1.x) - (pt.x-p1.x)*(p2.y-p1.y);
		double CA = (pt.y-p3.y)*(p1.x-p3.x) - (pt.x-p3.x)*(p1.y-p3.y);
		double BC = (pt.y-p2.y)*(p3.x-p2.x) - (pt.x-p2.x)*(p3.y-p2.y);
		return (AB*BC>0.f && BC*CA>0.f);
	}
	
	Vec2d addLen(double l)const{ return poliar(ang(),len()+l); }
	
    static Vec2d Unit(){ return Vec2d(0,1); }
    static Vec2d Ziro(){ return Vec2d(0,0); }
    static const Vec2d& Null(){ static Vec2d nullvec(0,0); nullvec.isNull=true; return nullvec; }

	
	static double distance(const Vec2d& v1, const Vec2d& v2){ return (v2-v1).len(); }
	static double heading(const Vec2d& v1, const Vec2d& v2){ return (v2-v1).angY()+v1.angY(); }
	static double d2r(double d){return d/180*PI;}
	static double r2d(double r){return r/PI*180;}
};

static ostream& operator<<(ostream& o, const Vec2d& v){
	if(v.isNull) return o<<"(NULL)";
	return o<<"("<<v.x<<","<<v.y<<")";
}

#endif