/**
 * traj_splitter_to_vector service
 * -----
 * Input: target coordinations (x, y, z, roll, pitch, yaw), number of segments, total time
 *
 * Output: a trajectory vector (an array) consisting of velocities describing the following speed function:
 * First quarter of total time: linear acceleration
 * 2nd and 3rd quarters of total time: no acceleration
 * 4th quarter: linear deceleration
 *
 * Service name: traj_vector_server
 *
 */



#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "ros/common.h"
#include "ros/ros.h"
#include "traj_splitter_to_vector/trajectory_vector.h"
#include "math.h"
#include <geometry_msgs/Vector3.h>

class traj_vector_server{
private:
	ros::NodeHandle n;
	ros::ServiceServer service;
public:
	traj_vector_server(){
		service = n.advertiseService("traj_vector_server", &traj_vector_server::split, this);
		ROS_INFO("Ready to split trajectory");
	}
	bool split(traj_splitter_to_vector::trajectory_vector::Request &req,
			traj_splitter_to_vector::trajectory_vector::Response &res )
	{
		ROS_INFO("Splitting trajectory");
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
			res.PositionArray[i].x = (16.0/3)*(req.Position.x/pow(T, 2))*t;
			res.PositionArray[i].y = (16.0/3)*(req.Position.y/pow(T, 2))*t;
			res.PositionArray[i].z = (16.0/3)*(req.Position.z/pow(T, 2))*t;
			res.AngleArray[i].x = (16.0/3)*(req.Angle.x/pow(T, 2))*t;
			res.AngleArray[i].y = (16.0/3)*(req.Angle.y/pow(T, 2))*t;
			res.AngleArray[i].z = (16.0/3)*(req.Angle.z/pow(T, 2))*t;
		}



		//Part 2
		for (; i<(3*segments_number)/4 ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = (4.0/3)*(req.Position.x/T);
			res.PositionArray[i].y = (4.0/3)*(req.Position.y/T);
			res.PositionArray[i].z = (4.0/3)*(req.Position.z/T);
			res.AngleArray[i].x = (4.0/3)*(req.Angle.x/T);
			res.AngleArray[i].y = (4.0/3)*(req.Angle.y/T);
			res.AngleArray[i].z = (4.0/3)*(req.Angle.z/T);
		}

		//Part 3
		for (; i<segments_number ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = (16.0/3)*(req.Position.x/T) - ((16.0/3)*(req.Position.x/pow(T, 2))*t);
			res.PositionArray[i].y = (16.0/3)*(req.Position.y/T) - ((16.0/3)*(req.Position.y/pow(T, 2))*t);
			res.PositionArray[i].z = (16.0/3)*(req.Position.z/T) - ((16.0/3)*(req.Position.z/pow(T, 2))*t);
			res.AngleArray[i].x = (16.0/3)*(req.Angle.x/T) - ((16.0/3)*(req.Angle.x/pow(T, 2))*t);
			res.AngleArray[i].y = (16.0/3)*(req.Angle.y/T) - ((16.0/3)*(req.Angle.y/pow(T, 2))*t);
			res.AngleArray[i].z = (16.0/3)*(req.Angle.z/T) - ((16.0/3)*(req.Angle.z/pow(T, 2))*t);

		}
		return true;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "traj_vector_server");
	traj_vector_server *s = new traj_vector_server();
	ros::spin();

	return 0;
}

