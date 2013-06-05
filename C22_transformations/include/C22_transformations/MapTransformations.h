#ifndef _MAP_TRANSFORMATION_HPP_
#define _MAP_TRANSFORMATION_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <C22_GroundRecognitionAndMapping/C22C0_PATH.h>
#include "Vec2d.hpp"


class C22_transform{

public:
	class MapProperties{
		friend class C22_transform;
	public:
#define D_OFFSET Vec2d(0,0)
#define D_RESOLUTION 0.25
#define D_ROTATION Vec2d::d2r(0)
#define D_SWAPXY true

		Vec2d offset;
		double resolution;
		double rotation;
		bool swapXY;
	public:
		MapProperties(const MapProperties& o):offset(o.offset),resolution(o.resolution),rotation(o.rotation),swapXY(o.swapXY){}

		MapProperties(const C22_GroundRecognitionAndMapping::C22C0_PATH& c22msg){
			offset = Vec2d(c22msg.robotPos.x, c22msg.robotPos.y);
			resolution = D_RESOLUTION;
			rotation = D_ROTATION;
			swapXY=D_SWAPXY;
		}
		MapProperties(){
			resolution = D_RESOLUTION;
			rotation = D_ROTATION;
			swapXY=D_SWAPXY;
		}
		MapProperties(Vec2d offset, double res=D_RESOLUTION, double rot= D_ROTATION, bool swapXY=D_SWAPXY): offset(offset), resolution(res), rotation(rot), swapXY(swapXY){}
	};

	C22_transform(){}
	C22_transform(const MapProperties& prop):mapprop(prop){}

private :

	MapProperties mapprop;

public:

template< class A, class B>
void GlobalToMap(const MapProperties& map, const A& original, B& transformed)const{
	Vec2d v(original.x, original.y);
	Vec2d t = (v-map.offset).rotate(- map.rotation).scale(1.0/map.resolution);
	if(map.swapXY){
		transformed.x = t.y;
		transformed.y = t.x;
	}else{
		transformed.x = t.x;
		transformed.y = t.y;
	}
}

template< class A, class B>
void MapToGlobal(const MapProperties& map, const A& original, B& transformed)const{
	Vec2d v = map.swapXY ?  Vec2d(original.y, original.x) : Vec2d(original.x, original.y);
	Vec2d t = v.scale(map.resolution).rotate(map.rotation) + map.offset;
	transformed.x = t.x;
	transformed.y = t.y;
}


template< class A, class B>
void MapToRelative(const MapProperties& map, double global_robot_x, double global_robot_y, double global_robot_heading, const A& original, B& transformed)const{
	Vec2d v = map.swapXY ?  Vec2d(original.y, original.x) : Vec2d(original.x, original.y);
	Vec2d g;
	MapToGlobal(map, v, g);
	Vec2d bot(global_robot_x, global_robot_y);
	Vec2d t = (g-bot).rotate(-global_robot_heading);
	transformed.x = t.x;
	transformed.y = t.y;
}

template< class A, class B>
void RelativeToMap(const MapProperties& map, double global_robot_x, double global_robot_y, double global_robot_heading, const A& original, B& transformed)const{
	Vec2d v(original.x, original.y);
	Vec2d bot(global_robot_x, global_robot_y);
	Vec2d g = v.rotate(global_robot_heading)+bot;
	Vec2d t;
	GlobalToMap(map, g, t);
	if(map.swapXY){
		transformed.x = t.y;
		transformed.y = t.x;
	}else{
		transformed.x = t.x;
		transformed.y = t.y;
	}
}


template< class A, class B>
void GlobalToMap(const A& original, B& transformed)const{
	GlobalToMap(mapprop, original,transformed);
}

template< class A, class B>
void MapToGlobal(const A& original, B& transformed)const{
	MapToGlobal(mapprop, original,transformed);
}


template< class A, class B>
void MapToRelative(double global_robot_x, double global_robot_y, double global_robot_heading, const A& original, B& transformed)const{
	MapToRelative(mapprop, global_robot_x,global_robot_y,global_robot_heading,original,transformed);
}

template< class A, class B>
void RelativeToMap(double global_robot_x, double global_robot_y, double global_robot_heading, const A& original, B& transformed)const{
	RelativeToMap(mapprop, global_robot_x,global_robot_y,global_robot_heading,original,transformed);
}


//
///**
// * given a square coordinates from the C22 global map,returns it's estimated global position
// */
//public: geometry_msgs::Point MapToGlobal(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,int row,int column ,double resolution=0.25){
//	geometry_msgs::Point msg;
//	msg.x=(row+c22msg.xOffset)*resolution+resolution/2;
//	msg.y=(column+c22msg.yOffset)*resolution+resolution/2;
//	msg.z=c22msg.row.at(row).column.at(column).height;
//	return msg;
//}
//
///**
// * given a square coordinates from the C22 global map,returns the relative position to the robot
// */
//public: geometry_msgs::Point mapSquareToRelative(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,int row,int column ,double resolution=0.25){
//
//	geometry_msgs::Point squarePose=mapSquareToGlobal(c22msg,row,column);
//	tf::Transform pelvisRelative;
//	pelvisRelative.setOrigin(tf::Vector3(0,0,0));
//	pelvisRelative.setRotation(tf::Quaternion(c22msg.robotOri.z,c22msg.robotOri.y,c22msg.robotOri.x));
//	tf::Quaternion pelvisRot=pelvisRelative.getRotation();
//	tf::Vector3 pos =quatRotate(pelvisRot,tf::Vector3(squarePose.x-c22msg.robotPos.x,squarePose.y-c22msg.robotPos.y,squarePose.z-c22msg.robotPos.z));
//	geometry_msgs::Point msg;
//	msg.x=pos.m_floats[0];
//	msg.y=pos.m_floats[1];
//	msg.z=pos.m_floats[2];
//	return msg;
//}
//
///**
// * given a global position, return it's square position on the map (WORKS ONLY WITH CHEATS_ON!!!!!!!!!!)
// */
//geometry_msgs::Point globalPointToRelative(C22_GroundRecognitionAndMapping::C22C0_PATH c22msg,float x,float y,float z,double resolution=0.25){
//
//	geometry_msgs::Point msg;
//	msg.x=(x-c22msg.xOffset)*(1/resolution);
//	msg.y=(y-c22msg.yOffset)*(1/resolution);
//	msg.z=z;
//	return msg;
//}



};


#endif
