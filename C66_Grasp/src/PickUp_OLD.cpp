#include <ros/ros.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/RobilTask.h>
#include <move_hand/matrix.h>
#include <move_hand/move_hand.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <boost/bind.hpp>
#include <string>
#include <boost/bind/bind.hpp>
#include <hand_grasp/grasp.h>
#include <move_hand/pelvis_move_hand.h>
#include <osrf_msgs/JointCommands.h>
#include <atlas_msgs/AtlasCommand.h>
#include <sensor_msgs/JointState.h>
#include <sstream>
#include <iostream>
#include <C23_ObjectRecognition/C23_orient.h>
#include <sandia_hand_msgs/SimpleGraspSrv.h>
#include <C67_CarManipulation/arm_path.h>
#include <Eigen/Dense>

static ros::ServiceClient sandia_client;
static sandia_hand_msgs::SimpleGraspSrv sandia_srv;
static boost::mutex mutex;
static boost::mutex send_mutex;

void update_comand(){
	boost::mutex::scoped_lock l(send_mutex);
	//	sandia_srv.request.grasp.name = "cylindrical";
	//	sandia_srv.request.grasp.closed_amount = 0.0;
	if (!sandia_client.call(sandia_srv))
	{
		//		ROS_INFO("%s: Sandia Hand Service Call Failed!", _name.c_str());
		//		retValue  = SandiaCallFail;
		//		return TaskResult(retValue, "ERROR");
	}
}

class PickUp{
private:
	ros::NodeHandle nh_;
	ros::ServiceClient pelvis_move_hand_matrix_cli_,pelvis_move_hand_cli_,move_hand_cli_,r_arm_path_cli_;
	actionlib::SimpleActionServer<C0_RobilTask::RobilTaskAction> as_; // NodeHandle instance must be created before this line.
	C0_RobilTask::RobilTaskFeedback feedback_;
	C0_RobilTask::RobilTaskResult result_;
	std::string action_name_;
	ros::NodeHandle* rosnode;
	ros::Publisher pub_hand_grasp_right,pub_hand_grasp_left,sandia_right_pub;
	ros::ServiceClient c23client;
	move_hand::matrix move_hand_matrix_msg;
	move_hand::pelvis_move_hand pel_msg;
	sandia_hand_msgs::SimpleGrasp sandia_msg;
	C67_CarManipulation::arm_path arm_path_msg;
public:
	PickUp(std::string name)
	:as_(nh_, name, false), action_name_(name){
		rosnode = new ros::NodeHandle();
		pelvis_move_hand_matrix_cli_ = nh_.serviceClient<move_hand::matrix>("C66_matrix");
		pelvis_move_hand_cli_ = nh_.serviceClient<move_hand::pelvis_move_hand>("pelvis_move_hand");
		move_hand_cli_ = nh_.serviceClient<move_hand::move_hand>("move_hand");
		r_arm_path_cli_ = nh_.serviceClient<C67_CarManipulation::arm_path>("rPath_srv");
		sandia_right_pub = rosnode->advertise<sandia_hand_msgs::SimpleGrasp>("/sandia_hands/r_hand/simple_grasp", 100, true);
		//Set callback functions
		as_.registerGoalCallback(boost::bind(&PickUp::goalCB, this));
		//as_.registerPreemptCallback(boost::bind(&PickUp::preemptCB, this));
		pub_hand_grasp_right = rosnode->advertise<hand_grasp::grasp>("/hand_grasp_right", 100, true);
		pub_hand_grasp_left= rosnode->advertise<hand_grasp::grasp>("/hand_grasp_left", 100, true);
		c23client = rosnode->serviceClient<C23_ObjectRecognition::C23_orient>("/C23/C66");
		{boost::mutex::scoped_lock l(send_mutex);
		sandia_srv.request.grasp.name = "cylindrical";
		sandia_srv.request.grasp.closed_amount = 0;
		}
		ROS_INFO("starting");
		as_.start();
		ROS_INFO("started");
	}
	~PickUp(){}

	bool getObjectData(std::string target, double *x, double *y, double *z, double *R, double *P, double *Y) {
		C23_ObjectRecognition::C23_orient c23srv;
		c23srv.request.target = target;

		ROS_INFO("Calling C23 perception service");
		if(c23client.call(c23srv))
		{
			*x = c23srv.response.x;
			*y = c23srv.response.y;
			*z = c23srv.response.z;
			*R = c23srv.response.R;
			*P = c23srv.response.P;
			*Y = c23srv.response.Y;

			ROS_INFO("Received data from majd: x= %f , y= %f , z= %f R= %f , P= %f , Y= %f",*x ,*y ,*z,*R,*P,*Y);
			//cout << "Received data from majd: " << c23srv.response.x << "," << c23srv.response.y << "," << c23srv.response.z << endl;
			return true;
		}
		else
		{
			ROS_INFO("No Message From Majd" );

		}
		return false;

	}

	void goalCB(){

		ROS_INFO("Start time: %f", ros::Time::now().toSec());
		hand_grasp::grasp grasp_msg;

		move_hand::move_hand move_hand_msg;
		std::string g = as_.acceptNewGoal()->parameters;
		char c = g.at(0);
		ROS_INFO("got command %c",c);
		double *X,*Y,*Z,*R,*P,*YA;
		switch (c) {
		case 'F':

			ros::Duration(0.3).sleep();
			ROS_INFO("requesting transformation from perception");
			//double *X,*Y,*Z,*R,*P,*YA;
			X = new double(0);
			Y = new double(0);
			Z = new double(0);
			R = new double(0);
			P = new double(0);
			YA = new double(0);
			/*close hand*/
			sandia_msg.name = "cylindrical";
			{boost::mutex::scoped_lock l(send_mutex);
			sandia_srv.request.grasp.name = "cylindrical";
			sandia_srv.request.grasp.closed_amount = 0.4;
			}

			arm_path_msg.request.PositionDestination.x = -0.128;
			arm_path_msg.request.PositionDestination.y = -0.414;
			arm_path_msg.request.PositionDestination.z = 0.07;
			arm_path_msg.request.AngleDestination.x = 2.749;
			arm_path_msg.request.AngleDestination.y = 0.76378;
			arm_path_msg.request.AngleDestination.z = -1.408;
			arm_path_msg.request.time =4;
			arm_path_msg.request.points =200;
			ROS_INFO("Calibrating Hands1");
			if(r_arm_path_cli_.call(arm_path_msg)){}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			arm_path_msg.request.PositionDestination.x = -0.3886;
			arm_path_msg.request.PositionDestination.y = -0.569;
			arm_path_msg.request.PositionDestination.z = 0.0;
			arm_path_msg.request.AngleDestination.x = 2.803;
			arm_path_msg.request.AngleDestination.y = 0.927;
			arm_path_msg.request.AngleDestination.z = -1.471;
			ROS_INFO("Calibrating Hands2");
			if(r_arm_path_cli_.call(arm_path_msg)){}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}
			arm_path_msg.request.PositionDestination.x = -0.389;
			arm_path_msg.request.PositionDestination.y = -0.582;
			arm_path_msg.request.PositionDestination.z = 0.552;
			arm_path_msg.request.AngleDestination.x = 2.734;
			arm_path_msg.request.AngleDestination.y = -0.862;
			arm_path_msg.request.AngleDestination.z = -2.309;
			ROS_INFO("Calibrating Hands3");
			if(r_arm_path_cli_.call(arm_path_msg)){}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}
			arm_path_msg.request.PositionDestination.x = 0.3;
			arm_path_msg.request.PositionDestination.y = -0.4;
			arm_path_msg.request.PositionDestination.z = 0.4;
			arm_path_msg.request.AngleDestination.x = 2.762;
			arm_path_msg.request.AngleDestination.y = 0.5;
			arm_path_msg.request.AngleDestination.z = 0.0;
			ROS_INFO("Calibrating Hands4");
			if(r_arm_path_cli_.call(arm_path_msg)){}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			if (getObjectData(g,X,Y,Z,R,P,YA)){
				//Grasp the other way if the grasp is over 90 degrees
				if (!((-M_PI/2<*YA)&&(*YA<M_PI/2))) {
					*YA=*YA+M_PI;
				}
				/*init vector for pipe*/
				double sr=sin(*R),
						cr=cos(*R),
						sp=sin(*P),
						cp=cos(*P),
						sy=sin(*YA),
						cy=cos(*YA);
				Eigen::Matrix4f origin,transform,cross;
				transform(0,0) =cp*cy; 	transform(0,1) = cy*sp*sr-cr*sy; 	transform(0,2) = sr*sy+cr*cy*sp; transform(0,3) = *X;
				transform(1,0) = cp*sy; transform(1,1) = cr*cy+cy*sp*sr;	transform(1,2) = cr*sp*sy-cy*sr; transform(1,3) = *Y;
				transform(2,0) = -sp; 	transform(2,1) = cp*sr; 			transform(2,2) = cp*cr; 		 transform(2,3) = *Z;
				transform(3,0) = 0; 	transform(3,1) = 0; 				transform(3,2) = 0; 			 transform(3,3) = 1;

				sr=sin(-3.13);
				cr=cos(-3.13);
				sp=sin(0.097);
				cp=cos(0.097);
				sy=sin(0.625);
				cy=cos(0.625);


				origin(0,0) =cp*cy; 	origin(0,1) = cy*sp*sr-cr*sy; 	origin(0,2) = sr*sy+cr*cy*sp; origin(0,3) = 0.502;
				origin(1,0) = cp*sy; 	origin(1,1) = cr*cy+cy*sp*sr;	origin(1,2) = cr*sp*sy-cy*sr; origin(1,3) = -0.073;
				origin(2,0) = -sp; 		origin(2,1) = cp*sr; 			origin(2,2) = cp*cr; 		  origin(2,3) = 0.32;
				origin(3,0) = 0; 		origin(3,1) = 0; 				origin(3,2) = 0; 			  origin(3,3) = 1;
				cross = transform*origin;
				arm_path_msg.request.PositionDestination.x = cross(0,3);
				arm_path_msg.request.PositionDestination.y = cross(1,3);
				arm_path_msg.request.PositionDestination.z = cross(2,3);
				arm_path_msg.request.AngleDestination.x = atan2((double)cross(2,1),(double)cross(2,2));
				arm_path_msg.request.AngleDestination.y = atan2((double)-cross(2,0),sqrt(pow((double)cross(2,1),2) +pow((double)cross(2,2),2) ));
				arm_path_msg.request.AngleDestination.z = atan2((double)cross(1,0),(double)cross(0,0));
				arm_path_msg.request.time =4;
				arm_path_msg.request.points =200;
				ROS_INFO("recieved matrix from perception");
				if (as_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("ERROR not recieve matrix from perception");
					ROS_ERROR("%s: Preempted", action_name_.c_str());
					// set the action state to preempted
					as_.setPreempted();
					return;
				}
				/*moving hand to requested object*/
				ROS_INFO("moving hand to requested object");

				if(r_arm_path_cli_.call(arm_path_msg)){

				}
				else{
					ROS_INFO("ERROR in arm_path service");
					as_.setAborted();
					return;
				}}
			else {
				ROS_INFO("ERROR in requesting transformation from perception");
				as_.setAborted();
				return;
			}

			arm_path_msg.request.PositionDestination.z -= 0.2;
			if(r_arm_path_cli_.call(arm_path_msg)){

			}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			ROS_INFO("closing right hand");
			{boost::mutex::scoped_lock l(send_mutex);
			sandia_srv.request.grasp.name = "cylindrical";
			sandia_srv.request.grasp.closed_amount = 1;
			}
			update_comand();
			ros::spinOnce();
			ros::Duration(1.2).sleep();
			/*lifting*/

			arm_path_msg.request.PositionDestination.z += 0.2;

			if(r_arm_path_cli_.call(arm_path_msg)){

			}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			break;



		case 'S':
			/*open hand*/
			ROS_INFO("opening right hand");
			/*grasp_msg.strength = 0;
						pub_hand_grasp_right.publish(grasp_msg);
						ros::Duration(0.3).sleep();
			 */

			ros::Duration(0.3).sleep();
			ROS_INFO("requesting transformation from perception");
			//double *X,*Y,*Z,*R,*P,*YA;
			X = new double(0);
			Y = new double(0);
			Z = new double(0);
			R = new double(0);
			P = new double(0);
			YA = new double(0);
			/*close hand*/
			sandia_msg.name = "cylindrical";
			{boost::mutex::scoped_lock l(send_mutex);
			sandia_srv.request.grasp.name = "cylindrical";
			sandia_srv.request.grasp.closed_amount = 0.4;
			}
			update_comand();
			ros::spinOnce();
			if (getObjectData(g,X,Y,Z,R,P,YA)){
				//Grasp the other way if the grasp is over 90 degrees
				if (!((-M_PI/2<*YA)&&(*YA<M_PI/2))) {
					*YA=*YA+M_PI;
				}
				/*init vector for pipe*/
				double sr=sin(*R),
						cr=cos(*R),
						sp=sin(*P),
						cp=cos(*P),
						sy=sin(*YA),
						cy=cos(*YA);
				Eigen::Matrix4f origin,transform,cross;
				transform(0,0) =cp*cy; 	transform(0,1) = cy*sp*sr-cr*sy; 	transform(0,2) = sr*sy+cr*cy*sp; transform(0,3) = *X;
				transform(1,0) = cp*sy; transform(1,1) = cr*cy+cy*sp*sr;	transform(1,2) = cr*sp*sy-cy*sr; transform(1,3) = *Y;
				transform(2,0) = -sp; 	transform(2,1) = cp*sr; 			transform(2,2) = cp*cr; 		 transform(2,3) = *Z;
				transform(3,0) = 0; 	transform(3,1) = 0; 				transform(3,2) = 0; 			 transform(3,3) = 1;

				sr=sin(-3.13);
				cr=cos(-3.13);
				sp=sin(0.097);
				cp=cos(0.097);
				sy=sin(0.625);
				cy=cos(0.625);


				origin(0,0) =cp*cy; 	origin(0,1) = cy*sp*sr-cr*sy; 	origin(0,2) = sr*sy+cr*cy*sp; origin(0,3) = 0.502;
				origin(1,0) = cp*sy; 	origin(1,1) = cr*cy+cy*sp*sr;	origin(1,2) = cr*sp*sy-cy*sr; origin(1,3) = -0.073;
				origin(2,0) = -sp; 		origin(2,1) = cp*sr; 			origin(2,2) = cp*cr; 		  origin(2,3) = 0.32;
				origin(3,0) = 0; 		origin(3,1) = 0; 				origin(3,2) = 0; 			  origin(3,3) = 1;
				cross = transform*origin;
				arm_path_msg.request.PositionDestination.x = cross(0,3);
				arm_path_msg.request.PositionDestination.y = cross(1,3);
				arm_path_msg.request.PositionDestination.z = cross(2,3);
				arm_path_msg.request.AngleDestination.x = atan2((double)cross(2,1),(double)cross(2,2));
				arm_path_msg.request.AngleDestination.y = atan2((double)-cross(2,0),sqrt(pow((double)cross(2,1),2) +pow((double)cross(2,2),2) ));
				arm_path_msg.request.AngleDestination.z = atan2((double)cross(1,0),(double)cross(0,0));
				arm_path_msg.request.time =4;
				arm_path_msg.request.points =200;
				ROS_INFO("recieved matrix from perception");
				if (as_.isPreemptRequested() || !ros::ok())
				{
					ROS_INFO("ERROR not recieve matrix from perception");
					ROS_ERROR("%s: Preempted", action_name_.c_str());
					// set the action state to preempted
					as_.setPreempted();
					return;
				}
				/*moving hand to requested object*/
				ROS_INFO("moving hand to requested object");

				if(r_arm_path_cli_.call(arm_path_msg)){

				}
				else{
					ROS_INFO("ERROR in arm_path service");
					as_.setAborted();
					return;
				}}
			else {
				ROS_INFO("ERROR in requesting transformation from perception");
				as_.setAborted();
				return;
			}

			arm_path_msg.request.PositionDestination.z -= 0.2;
			if(r_arm_path_cli_.call(arm_path_msg)){

			}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			ROS_INFO("closing right hand");
			{boost::mutex::scoped_lock l(send_mutex);
			sandia_srv.request.grasp.name = "cylindrical";
			sandia_srv.request.grasp.closed_amount = 1;
			}
			update_comand();
			ros::spinOnce();
			ros::Duration(1.2).sleep();
			/*lifting*/

			arm_path_msg.request.PositionDestination.z += 0.2;

			if(r_arm_path_cli_.call(arm_path_msg)){

			}
			else{
				ROS_INFO("ERROR in arm_path service");
				as_.setAborted();
				return;
			}

			break;



		case 'v': break;


		default: break;
		}
		C0_RobilTask::RobilTaskResult _res;
		_res.success = C0_RobilTask::RobilTask::SUCCESS;
		as_.setSucceeded(_res);
		ROS_INFO("End time: %f", ros::Time::now().toSec());
		return;
	}
	/*
	void preemptCB()
	{
		ROS_ERROR("%s: Preempted", action_name_.c_str());
		as_.setPreempted();
	}*/
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "C66_PickUp");
	PickUp* QuasiStaticWalker = new PickUp("PickUp");
	ROS_INFO("Running PickUp action");

	ros::NodeHandle* rosnode = new ros::NodeHandle();
	sandia_client = rosnode->serviceClient<sandia_hand_msgs::SimpleGraspSrv>("/sandia_hands/r_hand/simple_grasp");

	//ros::spin();

	ros::Rate r(2);
	while(ros::ok()){
		update_comand();
		r.sleep();
		ros::spinOnce();
	}


	return 0;
};

