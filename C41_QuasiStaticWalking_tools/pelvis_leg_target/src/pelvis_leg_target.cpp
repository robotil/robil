#include <ros/ros.h>
#include <pelvis_leg_target/pelvis_leg_target.h>
#include <pelvis_leg_target/static_leg.h>
#include <tf/transform_listener.h>

#include <C43_LocalBodyCOM/Kinematics.h>
#include <C43_LocalBodyCOM/CoM_Array_msg.h>
#include <move_pelvis/move_pelvis.h>
#include <sensor_msgs/Imu.h>

class move_to_leg{
private:
	ros::NodeHandle nh_, nh2_;
	tf::TransformListener listener;
	ros::ServiceClient move_pelvis_client, make_step_cli_;
	ros::ServiceServer srv_server, full_step_server;
	ros::Subscriber pcom_sub_;
	std::string static_leg_variable;
	C43_LocalBodyCOM::CoM_Array_msg com;

	ros::Subscriber imu_sub_;
	geometry_msgs::Point imu, imu_v;

public:
	move_to_leg(){
		move_pelvis_client = nh_.serviceClient<move_pelvis::move_pelvis>("move_pelvis");
		pcom_sub_ = nh_.subscribe("/c43_local_body/com", 1, &move_to_leg::get_com_from_hrl_kinematics, this);
		imu_sub_ = nh_.subscribe("/atlas/imu",100, &move_to_leg::imu_CB,this);
		srv_server = nh_.advertiseService("pelvis_leg_target",&move_to_leg::srv_CB,this);

		make_step_cli_ = nh_.serviceClient<move_pelvis::move_pelvis>("make_step");
		full_step_server = nh_.advertiseService("/full_step",&move_to_leg::full_step_CB,this);
	}


	~move_to_leg(){
	}

	void get_com_from_hrl_kinematics(const C43_LocalBodyCOM::CoM_Array_msgConstPtr& comArray){
		//ROS_INFO("# of points: %d", (int) comMarker->points.size());

		com = *comArray;
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
		move_pelvis_srv.request.AngleDestination.z=0;//QuatToYaw(transform.getRotation());
		ROS_INFO("Sending pelvis to relative x,y,z: %f,%f,%f",move_pelvis_srv.request.PositionDestination.x,move_pelvis_srv.request.PositionDestination.y,move_pelvis_srv.request.PositionDestination.z);
		move_pelvis_client.call(move_pelvis_srv);
		static_leg_variable = req.leg;
		res.SUCCESS=move_pelvis_srv.response.success;
		return true;
	}

	bool full_step_CB(move_pelvis::move_pelvis::Request &req,move_pelvis::move_pelvis::Response &res){

		pelvis_leg_target::pelvis_leg_target leg_target;
		move_pelvis::move_pelvis move_pelvis;

		if(!req.LinkToMove.compare("r_leg")){
			//Move over to left leg
			leg_target.request.leg = "l_foot";
			ROS_INFO("Moving to left leg");
			if(!this->srv_CB(leg_target.request, leg_target.response)){
				ROS_ERROR("Could not move to left leg");
				return false;
			}

			tf::StampedTransform p2r;
			try {
				listener.waitForTransform("/pelvis","/r_foot",ros::Time(0),ros::Duration(0.2));
				listener.lookupTransform("/pelvis","/r_foot",ros::Time(0),p2r);
			} catch (tf::TransformException &ex) {
				ROS_ERROR("%s",ex.what());
			}

			move_pelvis.request.PositionDestination.x = req.PositionDestination.x;
			move_pelvis.request.PositionDestination.y = req.PositionDestination.y;
			move_pelvis.request.PositionDestination.z = req.PositionDestination.z;
			move_pelvis.request.AngleDestination.x = req.AngleDestination.x;
			move_pelvis.request.AngleDestination.y = req.AngleDestination.y;
			move_pelvis.request.AngleDestination.z = req.AngleDestination.z;
			move_pelvis.request.LinkToMove = "r_leg";
			ROS_INFO("Moving left right forward");
			if(!make_step_cli_.call(move_pelvis)){
				ROS_ERROR("Could not move right leg forward");
				return false;
			}
			//Move over to right leg
			leg_target.request.leg = "r_foot";
			ROS_INFO("Moving to right leg");
			if(!this->srv_CB(leg_target.request, leg_target.response)){
				ROS_ERROR("Could not move to right leg");
				return false;
			}
		}else{
			if(!req.LinkToMove.compare("l_leg")){

				//Move over to right leg
				leg_target.request.leg = "r_foot";
				ROS_INFO("Moving to right leg");
				if(!this->srv_CB(leg_target.request, leg_target.response)){
					ROS_ERROR("Could not move to right leg");
					return false;
				}

				tf::StampedTransform p2r;
				try {
					listener.waitForTransform("/pelvis","/l_foot",ros::Time(0),ros::Duration(0.2));
					listener.lookupTransform("/pelvis","/l_foot",ros::Time(0),p2r);
				} catch (tf::TransformException &ex) {
					ROS_ERROR("%s",ex.what());
				}

				move_pelvis.request.PositionDestination.x = req.PositionDestination.x;
				move_pelvis.request.PositionDestination.y = req.PositionDestination.y;
				move_pelvis.request.PositionDestination.z = req.PositionDestination.z;
				move_pelvis.request.AngleDestination.x = req.AngleDestination.x;
				move_pelvis.request.AngleDestination.y = req.AngleDestination.y;
				move_pelvis.request.AngleDestination.z = req.AngleDestination.z;
				move_pelvis.request.LinkToMove = "l_leg";
				ROS_INFO("Moving left left forward");
				if(!make_step_cli_.call(move_pelvis)){
					ROS_ERROR("Could not move left leg forward");
					return false;
				}
				//Move over to left leg
				leg_target.request.leg = "l_foot";
				ROS_INFO("Moving to left leg");
				if(!this->srv_CB(leg_target.request, leg_target.response)){
					ROS_ERROR("Could not move to left leg");
					return false;
				}


			}else{
				ROS_ERROR("Wrong link name");
				return false;
			}
		}

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
