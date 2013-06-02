#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <C22_GroundRecognitionAndMapping/C22C0_PATH.h>

class C22_transform{

/**
 * provide the location of the robot on the map provided by the C22 node
 * @param c22msg map provided by C22 node
 * @param resolution the size of the global area represented in the map 0.25 as default
 */
public: geometry_msgs::Point globalPosToMap(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,double resolution=0.25){
	geometry_msgs::Point msg;
	std::cout<<"c22msg.xOffset"<<c22msg.yOffset;
	msg.x=(c22msg.robotPos.x-c22msg.xOffset)*(1/resolution);
	msg.y=(c22msg.robotPos.y-c22msg.yOffset)*(1/resolution);
	msg.z=c22msg.robotPos.z;
	return msg;
}

/**
 * given a square coordinates from the C22 global map,returns it's estimated global position
 */
public: geometry_msgs::Point mapSquareToGlobal(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,int row,int column ,double resolution=0.25){
	geometry_msgs::Point msg;
	msg.x=(row+c22msg.xOffset)*resolution+resolution/2;
	msg.y=(column+c22msg.yOffset)*resolution+resolution/2;
	msg.z=c22msg.row.at(row).column.at(column).height;
	return msg;
}

/**
 * given a square coordinates from the C22 global map,returns the relative position to the robot
 */
public: geometry_msgs::Point mapSquareToRelative(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,int row,int column ,double resolution=0.25){

	geometry_msgs::Point squarePose=mapSquareToGlobal(c22msg,row,column);
	tf::Transform pelvisRelative;
	pelvisRelative.setOrigin(tf::Vector3(0,0,0));
	pelvisRelative.setRotation(tf::Quaternion(c22msg.robotOri.z,c22msg.robotOri.y,c22msg.robotOri.x));
	tf::Quaternion pelvisRot=pelvisRelative.getRotation();
	tf::Vector3 pos =quatRotate(pelvisRot,tf::Vector3(squarePose.x-c22msg.robotPos.x,squarePose.y-c22msg.robotPos.y,squarePose.z-c22msg.robotPos.z));
	geometry_msgs::Point msg;
	msg.x=pos.m_floats[0];
	msg.y=pos.m_floats[1];
	msg.z=pos.m_floats[2];
	return msg;
}

/**
 * given a global position, return it's square position on the map (WORKS ONLY WITH CHEATS_ON!!!!!!!!!!)
 */
geometry_msgs::Point globalPointToRelative(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,float x,float y,float z,double resolution=0.25){

	geometry_msgs::Point msg;
	msg.x=(x-c22msg.xOffset)*(1/resolution);
	msg.y=(y-c22msg.yOffset)*(1/resolution);
	msg.z=z;
	return msg;
}



};
