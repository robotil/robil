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
		geometry_msgs::Vector3 velocity_pos;
		velocity_pos.x = 0;
		velocity_pos.y = 0;
		velocity_pos.z = 0;
		geometry_msgs::Vector3 velocity_ang;
		velocity_ang.x = 0;
		velocity_ang.y = 0;
		velocity_ang.z = 0;
		geometry_msgs::Point Vpos_max,Vang_max;
		Vpos_max.x = req.Position.x/total_time;
		Vpos_max.y = req.Position.y/total_time;
		Vpos_max.z = req.Position.z/total_time;
		Vang_max.x = req.Angle.x/total_time;
		Vang_max.y = req.Angle.y/total_time;
		Vang_max.z = req.Angle.z/total_time;
		double T = total_time;
		double Sn = segments_number;
/*
		for(int i=0; i<segments_number; i++){

			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = ((req.Position.x/segments_number)*(i+1) - velocity_pos.x)/res.dt[i];
			velocity_pos.x = (req.Position.x/segments_number)*(i+1);

			res.PositionArray[i].y = ((req.Position.y/segments_number)*(i+1) - velocity_pos.y)/res.dt[i];
			velocity_pos.y = (req.Position.y/segments_number)*(i+1);

			res.PositionArray[i].z = ((req.Position.z/segments_number)*(i+1) - velocity_pos.z)/res.dt[i];
			velocity_pos.z = (req.Position.z/segments_number)*(i+1);

			res.AngleArray[i].x = ((req.Angle.x/segments_number)*(i+1) - velocity_ang.x)/res.dt[i];
			velocity_ang.x = (req.Angle.x/segments_number)*(i+1);

			res.AngleArray[i].y = ((req.Angle.y/segments_number)*(i+1) - velocity_ang.y)/res.dt[i];
			velocity_ang.y = (req.Angle.y/segments_number)*(i+1);

			res.AngleArray[i].z = ((req.Angle.z/segments_number)*(i+1) - velocity_ang.z)/res.dt[i];
			velocity_ang.z = (req.Angle.z/segments_number)*(i+1);

		}*/


		//ROS_INFO("Part 1");
		int i=0;
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

			/*ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );*/
		}



		//ROS_INFO("Part 2");
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

			/*ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );*/
		}

		//ROS_INFO("Part 3");
		for (; i<segments_number ; i++)
		{
			double t = (i+1)*T/Sn;
			//ROS_INFO("t: %f", t);
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = (16.0/3)*(req.Position.x/T) - ((16.0/3)*(req.Position.x/pow(T, 2))*t);
			res.PositionArray[i].y = (16.0/3)*(req.Position.y/T) - ((16.0/3)*(req.Position.y/pow(T, 2))*t);
			res.PositionArray[i].z = (16.0/3)*(req.Position.z/T) - ((16.0/3)*(req.Position.z/pow(T, 2))*t);
			res.AngleArray[i].x = (16.0/3)*(req.Angle.x/T) - ((16.0/3)*(req.Angle.x/pow(T, 2))*t);
			res.AngleArray[i].y = (16.0/3)*(req.Angle.y/T) - ((16.0/3)*(req.Angle.y/pow(T, 2))*t);
			res.AngleArray[i].z = (16.0/3)*(req.Angle.z/T) - ((16.0/3)*(req.Angle.z/pow(T, 2))*t);

			/*ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );*/
		}

/*
		ROS_INFO("Part 1");
		int i=0;
		for ( ; i<segments_number/4 ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = (8.0/3)*(req.Position.x/pow(T, 2))*(pow(t,2));
			res.PositionArray[i].y = (8.0/3)*(req.Position.y/pow(T, 2))*(pow(t,2));
			res.PositionArray[i].z = (8.0/3)*(req.Position.z/pow(T, 2))*(pow(t,2));
			res.AngleArray[i].x = (8.0/3)*(req.Angle.x/pow(T, 2))*(pow(t,2));
			res.AngleArray[i].y = (8.0/3)*(req.Angle.y/pow(T, 2))*(pow(t,2));
			res.AngleArray[i].z = (8.0/3)*(req.Angle.z/pow(T, 2))*(pow(t,2));

			ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );
		}



		ROS_INFO("Part 2");
		for (; i<(3*segments_number)/4 ; i++)
		{
			double t = (i+1)*T/Sn;
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = (4.0/3)*(req.Position.x/T)*(t) - (req.Position.x/6.0);
			res.PositionArray[i].y = (4.0/3)*(req.Position.y/T)*(t) - (req.Position.y/6.0);
			res.PositionArray[i].z = (4.0/3)*(req.Position.z/T)*(t) - (req.Position.z/6.0);
			res.AngleArray[i].x = (4.0/3)*(req.Angle.x/T)*(t) - (req.Angle.x/6.0);
			res.AngleArray[i].y = (4.0/3)*(req.Angle.y/T)*(t) - (req.Angle.y/6.0);
			res.AngleArray[i].z = (4.0/3)*(req.Angle.z/T)*(t) - (req.Angle.z/6.0);

			ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );
		}

		ROS_INFO("Part 3");
		for (; i<segments_number ; i++)
		{
			double t = (i+1)*T/Sn;
			ROS_INFO("t: %f", t);
			res.dt[i] = total_time/segments_number;
			res.PositionArray[i].x = -(5.0/3)*req.Position.x + (16.0/3)*(req.Position.x/T)*(t) - ((8.0/3)*(req.Position.x/pow(T, 2))*(pow(t,2)));
			res.PositionArray[i].y = -(5.0/3)*req.Position.y + (16.0/3)*(req.Position.y/T)*(t) - ((8.0/3)*(req.Position.y/pow(T, 2))*(pow(t,2)));
			res.PositionArray[i].z = -(5.0/3)*req.Position.z + (16.0/3)*(req.Position.z/T)*(t) - ((8.0/3)*(req.Position.z/pow(T, 2))*(pow(t,2)));
			res.AngleArray[i].x = -(5.0/3)*req.Angle.x + (16.0/3)*(req.Angle.x/T)*(t) - ((8.0/3)*(req.Angle.x/pow(T, 2))*(pow(t,2)));
			res.AngleArray[i].y = -(5.0/3)*req.Angle.y + (16.0/3)*(req.Angle.y/T)*(t) - ((8.0/3)*(req.Angle.y/pow(T, 2))*(pow(t,2)));
			res.AngleArray[i].z = -(5.0/3)*req.Angle.z + (16.0/3)*(req.Angle.z/T)*(t) - ((8.0/3)*(req.Angle.z/pow(T, 2))*(pow(t,2)));

			ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );
		}*/

		//ROS_INFO("position %f, %f, %f",res.PositionArray.poses[1].position.x , res.PositionArray.poses[1].position.y , res.PositionArray.poses[1].position.z );
		//ROS_INFO("sending back response: [%f]", res.PositionArray.poses);
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

