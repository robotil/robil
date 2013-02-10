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
		for (int i=0 ; i<segments_number ; i++)
		{
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

			/*ROS_INFO("position %f, %f, %f",res.PositionArray[i].x , res.PositionArray[i].y , res.PositionArray[i].z );
			ROS_INFO("angles %f, %f, %f",res.AngleArray[i].x , res.AngleArray[i].y , res.AngleArray[i].z );
			ROS_INFO("dt %f",res.dt[i] );*/
		}

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

