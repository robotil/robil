#include <ros/ros.h>
#include <pelvis_leg_target/pelvis_leg_target.h>
#include <pelvis_leg_target/static_leg.h>
#include <tf/transform_listener.h>
#include <move_pelvis/move_pelvis.h>
#include <hrl_kinematics/TestStability.h>
#include <sensor_msgs/Imu.h>

class move_to_leg{
private:
	ros::NodeHandle nh_, nh2_;
	tf::TransformListener listener;
	ros::ServiceClient move_pelvis_client;
	ros::ServiceServer srv_server,static_leg_server;
	ros::Subscriber pcom_sub_;
	std::string static_leg_variable;
	geometry_msgs::Point com;

	ros::Subscriber imu_sub_;
	geometry_msgs::Point imu, imu_v;

public:
	move_to_leg():static_leg_variable("pelvis"){
		move_pelvis_client = nh_.serviceClient<move_pelvis::move_pelvis>("move_pelvis");
		pcom_sub_ = nh_.subscribe("projected_com", 1, &move_to_leg::get_com_from_hrl_kinematics, this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100, &move_to_leg::imu_CB,this);
		srv_server = nh_.advertiseService("pelvis_leg_target",&move_to_leg::srv_CB,this);
		static_leg_server = nh2_.advertiseService("current_static_leg",&move_to_leg::static_leg,this);
	}


	~move_to_leg(){
	}

	void get_com_from_hrl_kinematics(const visualization_msgs::MarkerConstPtr& comMarker){
		//ROS_INFO("# of points: %d", (int) comMarker->points.size());

		com = comMarker->pose.position;
		//this->_CoM.x = com.x();
		//this->_CoM.y = com.y();
	}

	bool srv_CB(pelvis_leg_target::pelvis_leg_targetRequest &req,pelvis_leg_target::pelvis_leg_targetResponse &res){
		tf::StampedTransform transform;
		try {
			listener.waitForTransform("/pelvis",req.leg,ros::Time(0),ros::Duration(0.2));
			listener.lookupTransform("/pelvis",req.leg,ros::Time(0),transform);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
		}
		move_pelvis::move_pelvis move_pelvis_srv;

		move_pelvis_srv.request.PositionDestination.x=transform.getOrigin().x()/*-com.x*/;// + 0.04;
		move_pelvis_srv.request.PositionDestination.y=transform.getOrigin().y()/*-com.y*/;// + ((req.leg == "l_foot") ? -0.01 : 0.01);
		move_pelvis_srv.request.PositionDestination.z=0;
		move_pelvis_srv.request.LinkToMove = "pelvis";
		move_pelvis_srv.request.AngleDestination.x=-imu.x;//QuatToRoll(transform.getRotation());
		move_pelvis_srv.request.AngleDestination.y=-imu.y;//QuatToPitch(transform.getRotation());
		move_pelvis_srv.request.AngleDestination.z=QuatToYaw(transform.getRotation());
		ROS_INFO("Sending pelvis to relative x,y,z: %f,%f,%f",move_pelvis_srv.request.PositionDestination.x,move_pelvis_srv.request.PositionDestination.y,move_pelvis_srv.request.PositionDestination.z);
		move_pelvis_client.call(move_pelvis_srv);
		static_leg_variable = req.leg;
		res.SUCCESS=move_pelvis_srv.response.success;
		return true;
	}

	bool static_leg(pelvis_leg_target::static_leg::Request &req,pelvis_leg_target::static_leg::Response &res){
		res.leg = this->static_leg_variable;
		return true;
	}

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


	void imu_CB(const sensor_msgs::ImuConstPtr& imuC){

		double x = imuC->orientation.x;
		double y = imuC->orientation.y;
		double z = imuC->orientation.z;
		double w = imuC->orientation.w;
		/*Roll*/	imu.x = atan2(2*(y*z + w*x), w*w - x*x - y*y + z*z);
		/*Pitch*/	imu.y = asin(-2*(x*z - w*y));
		/*Yaw*/		imu.z = atan2(2*(x*y + w*z), w*w + x*x - y*y - z*z);
		imu_v.x = imuC->angular_velocity.x;
		imu_v.y = imuC->angular_velocity.y;
		imu_v.z = imuC->angular_velocity.z;

	}


};

int main(int argc, char **argv) {
	ros::init(argc, argv, "pelvis_leg_target");
	move_to_leg *move_to_leg_obj = new move_to_leg();
	ROS_INFO("ready");
	ros::spin();
	return 0;
}
