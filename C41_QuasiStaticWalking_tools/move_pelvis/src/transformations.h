/*
 * transformations.h
 *
 *  Created on: Apr 18, 2013
 *      Author: lab116-1
 */

#ifndef TRANSFORMATIONS_H_
#define TRANSFORMATIONS_H_

#include <tf/tf.h>
#include <Eigen/Dense>

double QuatToRoll(double x, double y, double z, double w){
	return atan2(2*(w*x + y*z), 1 - 2*(pow(x,2) + pow(y,2)));
}
double QuatToRoll(const tf::Quaternion &quat){
	return atan2(2*(quat.w()*quat.x() + quat.y()*quat.z()), 1 - 2*(pow(quat.x(),2) + pow(quat.y(),2)));
}
double QuatToRoll(const geometry_msgs::Quaternion &quat){
	return atan2(2*(quat.w*quat.x + quat.y*quat.z), 1 - 2*(pow(quat.x,2) + pow(quat.y,2)));
}


double QuatToPitch(double x, double y, double z, double w){
	return asin(2*(w*y - z*x));
}
double QuatToPitch(const tf::Quaternion &quat){
	return asin(2*(quat.w()*quat.y() - quat.z()*quat.x()));
}
double QuatToPitch(const geometry_msgs::Quaternion &quat){
	return asin(2*(quat.w*quat.y - quat.z*quat.x));
}


double QuatToYaw(double x, double y, double z, double w){
	return atan2(2*(w*z + x*y), 1 - 2*(pow(y,2) + pow(z,2)));
}
double QuatToYaw(const tf::Quaternion &quat){
	return atan2(2*(quat.w()*quat.z() + quat.x()*quat.y()), 1 - 2*(pow(quat.y(),2) + pow(quat.z(),2)));
}
double QuatToYaw(const geometry_msgs::Quaternion &quat){
	return atan2(2*(quat.w*quat.z + quat.x*quat.y), 1 - 2*(pow(quat.y,2) + pow(quat.z,2)));
}


struct XYZRPY{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

/**
 * If V1:Y->Z & V2:X->Y,
 * 	return X->Z
 */
XYZRPY VectorTranformation(	double x1, double y1, double z1, double roll1, double pitch1, double yaw1,
		double x2, double y2, double z2, double roll2, double pitch2, double yaw2)
{

	double cosr = cos(roll1);
	double cosp = cos(pitch1);
	double cosy = cos(yaw1);
	double sinr = sin(roll1);
	double sinp = sin(pitch1);
	double siny = sin(yaw1);
	Eigen::MatrixXf M1, M2, G;
	M1.resize(4,4);
	M2.resize(4,4);
	G.resize(4,4);
	M1 << 	cosp*cosy, 	cosy*sinp*sinr-cosr*siny, 	sinr*siny+cosr*cosy*sinp, 	x1,
			cosp*siny, 	cosr*cosy+cosy*sinp*sinr, 	cosr*sinp*siny-cosy*sinr, 	y1,
			-sinp,		cosp*sinr,					cosp*cosr,					z1,
			0,			0,							0,							1;

	cosr = cos(roll2);
	cosp = cos(pitch2);
	cosy = cos(yaw2);
	sinr = sin(roll2);
	sinp = sin(pitch2);
	siny = sin(yaw2);
	M2 << 	cosp*cosy, 	cosy*sinp*sinr-cosr*siny, 	sinr*siny+cosr*cosy*sinp, 	x2,
			cosp*siny, 	cosr*cosy+cosy*sinp*sinr, 	cosr*sinp*siny-cosy*sinr, 	y2,
			-sinp,		cosp*sinr,					cosp*cosr,					z2,
			0,			0,							0,							1;

	G = M2*M1;

	XYZRPY n;
	n.x = G(0,3);
	n.y = G(1,3);
	n.z = G(2,3);
	n.roll = atan2((double) G(2,1),(double) G(2,2));
	n.pitch = atan2((double) -G(2,0), sqrt(pow((double) G(2,1),2)+pow((double) G(2,2),2)));
	n.yaw = atan2((double) G(1,0), (double) G(0,0));
	return n;
}


XYZRPY VectorTransposeTranformation(	double x1, double y1, double z1, double roll1, double pitch1, double yaw1,
										double x2, double y2, double z2, double roll2, double pitch2, double yaw2)
{

	double cosr = cos(roll1);
	double cosp = cos(pitch1);
	double cosy = cos(yaw1);
	double sinr = sin(roll1);
	double sinp = sin(pitch1);
	double siny = sin(yaw1);
	Eigen::MatrixXf M1, M2, G;
	M1.resize(4,4);
	M2.resize(4,4);
	G.resize(4,4);
	M1 << 	cosp*cosy, 	cosy*sinp*sinr-cosr*siny, 	sinr*siny+cosr*cosy*sinp, 	x1,
			cosp*siny, 	cosr*cosy+cosy*sinp*sinr, 	cosr*sinp*siny-cosy*sinr, 	y1,
			-sinp,		cosp*sinr,					cosp*cosr,					z1,
			0,			0,							0,							1;

	cosr = cos(roll2);
	cosp = cos(pitch2);
	cosy = cos(yaw2);
	sinr = sin(roll2);
	sinp = sin(pitch2);
	siny = sin(yaw2);
	Eigen::Matrix3f Rt;
	Rt << 	-cosp*cosy, 					-cosp*siny, 			-sinp,
			-cosy*sinp*sinr+cosr*siny, 	-cosr*cosy-cosy*sinp*sinr, 	-cosp*sinr,
			-sinr*siny-cosr*cosy*sinp,	-cosr*sinp*siny+cosy*sinr,	-cosp*cosr;
	Eigen::Vector3f v, rtv;
	v << 	x2,
			y2,
			z2;

	rtv = Rt*v;

	M2 << 	cosp*cosy, 					cosp*siny, 					-sinp,					 	rtv(0),
			cosy*sinp*sinr-cosr*siny, 	cosr*cosy+cosy*sinp*sinr, 	cosp*sinr, 					rtv(1),
			sinr*siny+cosr*cosy*sinp,	cosr*sinp*siny-cosy*sinr,	cosp*cosr,					rtv(2),
			0,							0,							0,							1;

	G = M2*M1;

	XYZRPY n;
	n.x = G(0,3);
	n.y = G(1,3);
	n.z = G(2,3);
	n.roll = atan2((double) G(2,1),(double) G(2,2));
	n.pitch = atan2((double) -G(2,0), sqrt(pow((double) G(2,1),2)+pow((double) G(2,2),2)));
	n.yaw = atan2((double) G(1,0), (double) G(0,0));
	return n;
}



#endif /* TRANSFORMATIONS_H_ */
