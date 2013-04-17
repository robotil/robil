/**
 * traj_splitter_to_vector service
 * -----
 * Input: target coordinations (x, y, z, roll, pitch, yaw), number of segments, total time
 *
 * Output: a position vector (an array) consisting of velocities describing the following speed function:
 * First quarter of total time: linear acceleration
 * 2nd and 3rd quarters of total time: no acceleration
 * 4th quarter: linear deceleration
 *
 * Service name: walking_position_vector
 *
 */



#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/common.h"
#include "ros/ros.h"
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "math.h"
#include <geometry_msgs/Vector3.h>

class walking_position_vector{
private:
	ros::NodeHandle n;
	ros::ServiceServer service;
public:
	walking_position_vector(){
		service = n.advertiseService("walking_position_vector", &walking_position_vector::split, this);
		ROS_INFO("Ready to make walking position vector");
	}
	bool split(traj_splitter_to_vector::trajectory_vector::Request &req,
			traj_splitter_to_vector::trajectory_vector::Response &res )
	{
		ROS_INFO("Making walking position");
		int segments_number 	= req.segments_number ;
		double total_time 		= req.total_time  ; //seconds
		res.dt.resize(segments_number);
		res.PositionArray.resize(segments_number);
		res.AngleArray.resize(segments_number);
		double T = total_time;
		double Sn = segments_number;

		int i=0;
		//Part 1
		for ( ; i<segments_number/4 ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			//res.PositionArray[i].x = (16.0/3)*(req.Position.x/pow(T, 2))*t;
			double x = (8.0/3)*(req.Position.x/pow(T,2))*pow(t,2);
			double y = (8.0/3)*(req.Position.y/pow(T,2))*pow(t,2);
			double V = req.Position.z;
			double u = sqrt(pow(req.Position.x,2)+pow(req.Position.y,2));
			double h = sqrt(pow(x,2)+pow(y,2));
			//double z = V*pow(M_E, 1-pow(u,2)/(4*u*h-4*pow(h,2)));
			double z = (8.0/3)*(req.Position.z/pow(T,2))*pow(t,2);
			//double xdot = res.PositionArray[i].x;
			//long double root = ((pow(u,2))*pow(V,2)*x*(2*u-x));
			//long double z = -xdot*(pow(V,2)*(u-x))/(sqrt(root));
			//long double z = (pow(M_E,(1-pow(u,2)/(4*u*x-4*pow(x,2))))*pow(u,2)*V*(u-2*x))/(4*pow(u-x,2)*pow(x,2))*xdot;
			res.PositionArray[i].x = x;
			res.PositionArray[i].y = y;
			res.PositionArray[i].z = z;
			res.AngleArray[i].x = req.Angle.x*(i+1)/Sn;
			res.AngleArray[i].y = req.Angle.y*(i+1)/Sn;
			res.AngleArray[i].z = req.Angle.z*(i+1)/Sn;
			/*ROS_INFO("x of i %d: %f", i, x);
			ROS_INFO("zdot of i %d: %16.10Lf", i, z);
			ROS_INFO("xdot of i %d: %16.10f", i, xdot);*/
			ROS_INFO("X: %f" ,x);
			ROS_INFO("Y: %f" ,y);
			ROS_INFO("Z(%d): %f" ,i+1, z);
		}



		//Part 2
		for (; i<(3*segments_number)/4 ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			//res.PositionArray[i].x = (4.0/3)*(req.Position.x/T);
			double x = (4.0/3)*(req.Position.x/T)*(t-T/4.0) + req.Position.x/6.0;
			double y = (4.0/3)*(req.Position.y/T)*(t-T/4.0) + req.Position.y/6.0;
			double z = (4.0/3)*(req.Position.z/T)*(t-T/4.0) + req.Position.z/6.0;
			double V = req.Position.z;
			double u = sqrt(pow(req.Position.x,2)+pow(req.Position.y,2));
			double h = sqrt(pow(x,2)+pow(y,2));
			//double z = V*pow(M_E, 1-pow(u,2)/(4*u*h-4*pow(h,2)));
			//double xdot = res.PositionArray[i].x;
			//long double root = ((pow(u,2))*pow(V,2)*x*(2*u-x));
			//long double z = -xdot*(pow(V,2)*(u-x))/(sqrt(root));
			//long double z = (pow(M_E,(1-pow(u,2)/(4*u*x-4*pow(x,2))))*pow(u,2)*V*(u-2*x))/(4*pow(u-x,2)*pow(x,2))*xdot;
			res.PositionArray[i].x = x;
			res.PositionArray[i].y = y;
			res.PositionArray[i].z = z;
			res.AngleArray[i].x = req.Angle.x*(i+1)/Sn;
			res.AngleArray[i].y = req.Angle.y*(i+1)/Sn;
			res.AngleArray[i].z = req.Angle.z*(i+1)/Sn;
			//long double root = ((pow(u,2))*pow(V,2)*x*(2*u-x));
			/*res.PositionArray[i].y = 0;
			res.PositionArray[i].z = -z;
			res.AngleArray[i].x = 0;
			res.AngleArray[i].y = 0;
			res.AngleArray[i].z = 0;*/
			/*ROS_INFO("x of i %d: %f", i, x);
			ROS_INFO("zdot of i %d: %16.10Lf", i, z);
			ROS_INFO("xdot of i %d: %16.10f", i, xdot);*/
			ROS_INFO("X: %f" ,x);
			ROS_INFO("Y: %f" ,y);
			ROS_INFO("Z(%d): %f" ,i+1, z);
		}

		//Part 3
		for (; i<segments_number ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			//res.PositionArray[i].x = (16.0/3)*(req.Position.x/T) - ((16.0/3)*(req.Position.x/pow(T, 2))*t);
			double x = (-2.5 + (5.0/6.0))*req.Position.x + (16.0/3.0)*(req.Position.x/T)*t - (8.0/3.0)*(req.Position.x/pow(T,2))*pow(t,2);
			double y = (-2.5 + (5.0/6.0))*req.Position.y + (16.0/3.0)*(req.Position.y/T)*t - (8.0/3.0)*(req.Position.y/pow(T,2))*pow(t,2);
			double z = (-2.5 + (5.0/6.0))*req.Position.z + (16.0/3.0)*(req.Position.z/T)*t - (8.0/3.0)*(req.Position.z/pow(T,2))*pow(t,2);
			double V = req.Position.z;
			double u = sqrt(pow(req.Position.x,2)+pow(req.Position.y,2));
			double h = sqrt(pow(x,2)+pow(y,2));
			//double z = V*pow(M_E, 1-pow(u,2)/(4*u*h-4*pow(h,2)));
			//double xdot = res.PositionArray[i].x;
			//long double root = ((pow(u,2))*pow(V,2)*x*(2*u-x));
			//long double z = -xdot*(pow(V,2)*(u-x))/(sqrt(root));
			//long double z = (pow(M_E,(1-pow(u,2)/(4*u*x-4*pow(x,2))))*pow(u,2)*V*(u-2*x))/(4*pow(u-x,2)*pow(x,2))*xdot;
			res.PositionArray[i].x = x;
			res.PositionArray[i].y = y;
			res.PositionArray[i].z = /*(i+1==segments_number) ? 0 : */z;
			res.AngleArray[i].x = req.Angle.x*(i+1)/Sn;
			res.AngleArray[i].y = req.Angle.y*(i+1)/Sn;
			res.AngleArray[i].z = req.Angle.z*(i+1)/Sn;
			//double root = ((pow(u,2))*pow(V,2)*x*(2*u-x));
			//long double z = -xdot*(pow(V,2)*(u-x))/(sqrt(root));
			//long double z = (pow(M_E,(1-pow(u,2)/(4*u*x-4*pow(x,2))))*pow(u,2)*V*(u-2*x))/(4*pow(u-x,2)*pow(x,2))*xdot;
			/*ROS_INFO("x of i %d: %f", i, x);
			ROS_INFO("zdot of i %d: %16.10Lf", i, z);
			ROS_INFO("xdot of i %d: %16.10f", i, xdot);*/
			/*res.PositionArray[i].y = 0;
			res.PositionArray[i].z = (i==segments_number-1) ? 0 : -z;
			res.AngleArray[i].x = 0;
			res.AngleArray[i].y = 0;
			res.AngleArray[i].z = 0;*/
			ROS_INFO("X: %f" ,x);
			ROS_INFO("Y: %f" ,y);
			ROS_INFO("Z(%d): %f" ,i+1, res.PositionArray[i].z);
		}
		return true;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "walking_position_vector");
	walking_position_vector *s = new walking_position_vector();
	ros::spin();

	return 0;
}

