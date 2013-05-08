/**************************************************************************************
 * This is a template for the C22_CompactGroundRecognitionAndMapping module for the robil project
 * The C22_CompactGroundRecognitionAndMapping module goal is to provide a mapping of the terrain that surrounds the robot
 * The mapping include the planes that reside on each square 0.25 meter and their coefficients
 **************************************************************************************/

#include "ros/ros.h"
#include "pclPlane.h"
#include "MPlane.h"
#include "MapMatrix.h"
#include "C22_CompactGroundRecognitionAndMapping/C22.h"
#include "C22_CompactGroundRecognitionAndMapping/C22C24.h"
#include <C21_VisionAndLidar/C21_C22.h>
#include "sensor_msgs/PointCloud.h"
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "C22_Node.h"
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_srvs/Empty.h"
//c24 added includes
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <boost/thread/mutex.hpp>
#include <pcl/filters/passthrough.h>
 #include <pcl/visualization/cloud_viewer.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/extract_indices.h>
/**
 * this class represent the C22_Node,
 * it subscribe to the drc multisense topic and provide the Eagle eye mapping
 **/

boost::mutex m;
C22_Node::C22_Node():
	pointCloud_sub(nh_,"/C21/C22Lidar",1),
	pos_sub(nh_,"/ground_truth_odom",1),
	sync( MySyncPolicy( 10 ),pointCloud_sub, pos_sub),
	cloudRecord(new pcl::PointCloud<pcl::PointXYZ>)
	{
		sync.registerCallback( boost::bind( &C22_Node::callback, this, _1, _2) );
		_myMatrix=new MapMatrix();
		_myPlanes=new std::vector<pclPlane*>();
		ROS_INFO("C22 Compact Online\n");
		C22_pub=nh2_.advertise<C22_CompactGroundRecognitionAndMapping::C22C0_PATH>("C22compact_pub",1);
		service = nh2_.advertiseService("C22compact", &C22_Node::proccess, this); //Specifying what to do when a reconstructed 3d scene is requested



	}

/**
 * The call back function executed when a service is requested
 * it must return true in order to work properly
 * @param req the request message, generated by the node requesting the service
 * @param res the response message, generated by the service node when a service is requested
 */
bool C22_Node::proccess(C22_CompactGroundRecognitionAndMapping::C22::Request  &req,
	C22_CompactGroundRecognitionAndMapping::C22::Response &res ){
	//ROS_INFO("received request, trying to fetch data\n");
	res.drivingPath.row.resize(_myMatrix->data->size());
	res.drivingPath.robotPos.x=robotPos.x;
	res.drivingPath.robotPos.y=robotPos.y;
	res.drivingPath.robotPos.z=robotPos.z;
	res.drivingPath.robotOri.x=robotOri.x;
	res.drivingPath.robotOri.y=robotOri.y;
	res.drivingPath.robotOri.z=robotOri.z;
	boost::mutex::scoped_lock l(_myMatrix->mutex);
	res.drivingPath.xOffset=_myMatrix->xOffset;
	res.drivingPath.yOffset=_myMatrix->yOffset;
	for(unsigned int i=0;i<_myMatrix->data->size();i++){
		res.drivingPath.row.at(i).column.resize(_myMatrix->data->at(i)->size());
		for(unsigned int j=0;j<_myMatrix->data->at(i)->size();j++){
			res.drivingPath.row.at(i).column.at(j).status=_myMatrix->data->at(i)->at(j)->square_status;
			res.drivingPath.row.at(i).column.at(j).planes.resize(_myMatrix->data->at(i)->at(j)->square_Planes->size());
			for(unsigned int k=0;k<_myMatrix->data->at(i)->at(j)->square_Planes->size();k++){
				res.drivingPath.row.at(i).column.at(j).planes.at(k).x=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_x;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).y=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_y;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).z=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_z;
				res.drivingPath.row.at(i).column.at(j).planes.at(k).d=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_d;
			}
		}
	}
  return true;
}


/**
 * The call back function executed when a new point cloud has arrived
 */
void C22_Node::callback(const C21_VisionAndLidar::C21_C22::ConstPtr& pclMsg,const nav_msgs::Odometry::ConstPtr& pos_msg){

	 pcl::PointCloud<pcl::PointXYZ>cloud;
	 pcl::fromROSMsg<pcl::PointXYZ>(pclMsg->cloud,cloud);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(cloud.makeShared());
	 //std::cout<<"points before:"<<cloud_filtered->points.size()<<std::endl;
	 //sor.setInputCloud (cloud_filtered);
	 //sor.setLeafSize (0.01f, 0.01f, 0.01f);
	 //sor.filter (*cloud_filtered);
	 //std::cout<<"points after:"<<cloud_filtered->points.size()<<std::endl;

	 tf::Quaternion q;
	 tf::quaternionMsgToTF(pos_msg->pose.pose.orientation, q);
	 tf::Quaternion oldQ;
	 tf::quaternionMsgToTF(lastPose.pose.pose.orientation, oldQ);
	 oldQ.setRPY(0,0,0);
	 tf::Transform trans;
	 tf::Transform trans2;
	 tf::Transform trans3;
	 trans.setOrigin(tf::Vector3(pos_msg->pose.pose.position.x,pos_msg->pose.pose.position.y,pos_msg->pose.pose.position.z));
	 trans.setRotation(tf::Quaternion(q));
	 trans3.setOrigin(tf::Vector3(0,0,0));
	 trans3.setRotation(tf::Quaternion(q).inverse());
	 Eigen::Matrix4f recordToPlace,recordToRobot,sensorToFrame;
	 pcl_ros::transformAsMatrix(trans, recordToPlace);
	 trans2.setOrigin(tf::Vector3(-pos_msg->pose.pose.position.x,-pos_msg->pose.pose.position.y,-pos_msg->pose.pose.position.z));
	 trans2.setRotation(tf::Quaternion(oldQ));
	 pcl_ros::transformAsMatrix(trans, recordToPlace);
	 pcl_ros::transformAsMatrix(trans2, recordToRobot);
	 pcl_ros::transformAsMatrix(trans3,sensorToFrame);
	 double roll, pitch, yaw;
	 tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	 tf::Quaternion myEuler(pos_msg->pose.pose.orientation.x,pos_msg->pose.pose.orientation.y,pos_msg->pose.pose.orientation.z,pos_msg->pose.pose.orientation.w);
	 robotPos.x=pos_msg->pose.pose.position.x;
	 robotPos.y=pos_msg->pose.pose.position.y;
	 robotPos.z=pos_msg->pose.pose.position.z;
	 robotOri.x=roll;
	 robotOri.y=pitch;
	 robotOri.z=yaw;
	 // transform pointcloud from sensor frame to fixed robot frame
	 //
	 pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, recordToPlace);

	 lastPose.pose.pose.position.x=pos_msg->pose.pose.position.x;
	 lastPose.pose.pose.position.y=pos_msg->pose.pose.position.y;
	 lastPose.pose.pose.position.z=pos_msg->pose.pose.position.z;
	 lastPose.pose.pose.orientation.x=pos_msg->pose.pose.orientation.x;
	 lastPose.pose.pose.orientation.y=pos_msg->pose.pose.orientation.y;
	 lastPose.pose.pose.orientation.z=pos_msg->pose.pose.orientation.z;
	 lastPose.pose.pose.orientation.w=pos_msg->pose.pose.orientation.w;


	 //transform the point cloud (mirror image)

	// std::cout<<"cloud record before addition:"<<cloudRecord->points.size()<<std::endl;
	 *cloudRecord+=*cloud_filtered;

	 pcl::transformPointCloud(*cloudRecord, *cloud_filtered, recordToRobot);
	 pcl::transformPointCloud(*cloud_filtered, *cloud_filtered, sensorToFrame);
	 if(cloud_filtered->points.size()>60000){
		 /*pcl::PointCloud<pcl::PointXYZ>empty;
		 pcl::PointCloud<pcl::PointXYZ>::Ptr em(empty.makeShared());
		 cloudRecord.swap(em);*/
		  pcl::PointCloud<int> sampled_indices;
		  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
		  uniform_sampling.setInputCloud (cloudRecord);
		  uniform_sampling.setRadiusSearch (0.01);
		  uniform_sampling.compute (sampled_indices);
		  //pcl::copyPointCloud (*scene, sampled_indices.points, *scene_keypoints);
		  std::cout << "Scene total points: " << cloudRecord->points.size() ;
		  pcl::copyPointCloud (*cloudRecord, sampled_indices.points, *cloudRecord);
		  std::cout<< "; Selected Keypoints: " << cloudRecord->points.size() << std::endl;
		  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		  sor.setInputCloud (cloudRecord);
		  sor.setMeanK (10);
		  sor.setStddevMulThresh (1.0);
		  sor.filter (*cloudRecord);

	 }

	_myMatrix->computeMMatrix(cloud_filtered,robotPos);
	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cloudRecord);
	  pass.setFilterFieldName ("y");
	  pass.setFilterLimits (pos_msg->pose.pose.position.y-5,pos_msg->pose.pose.position.y+5);
	  //pass.setFilterLimitsNegative (true);
	  pass.filter (*cloudRecord);
	  pass.setInputCloud (cloudRecord);
	  pass.setFilterFieldName ("x");
	  pass.setFilterLimits (pos_msg->pose.pose.position.x-5,pos_msg->pose.pose.position.x+5);
	  //pass.setFilterLimitsNegative (true);
	  pass.filter (*cloudRecord);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (pos_msg->pose.pose.position.z-2,pos_msg->pose.pose.position.z+2);
	  //pass.setFilterLimitsNegative (true);
	  pass.filter (*cloudRecord);
	  cloud_filtered.reset();
	  C22_CompactGroundRecognitionAndMapping::C22C0_PATH outMsg;
	  outMsg.row.resize(_myMatrix->data->size());
	  outMsg.robotPos.x=robotPos.x;
	  outMsg.robotPos.y=robotPos.y;
	  outMsg.robotPos.z=robotPos.z;
	  outMsg.robotOri.x=robotOri.x;
	  outMsg.robotOri.y=robotOri.y;
	  outMsg.robotOri.z=robotOri.z;
	  boost::mutex::scoped_lock l(_myMatrix->mutex);
	outMsg.xOffset=_myMatrix->xOffset;
	outMsg.yOffset=_myMatrix->yOffset;
	for(unsigned int i=0;i<_myMatrix->data->size();i++){
		outMsg.row.at(i).column.resize(_myMatrix->data->at(i)->size());
		for(unsigned int j=0;j<_myMatrix->data->at(i)->size();j++){
			outMsg.row.at(i).column.at(j).status=_myMatrix->data->at(i)->at(j)->square_status;
			outMsg.row.at(i).column.at(j).planes.resize(_myMatrix->data->at(i)->at(j)->square_Planes->size());
			for(unsigned int k=0;k<_myMatrix->data->at(i)->at(j)->square_Planes->size();k++){
				outMsg.row.at(i).column.at(j).planes.at(k).x=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_x;
				outMsg.row.at(i).column.at(j).planes.at(k).y=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_y;
				outMsg.row.at(i).column.at(j).planes.at(k).z=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_z;
				outMsg.row.at(i).column.at(j).planes.at(k).d=_myMatrix->data->at(i)->at(j)->square_Planes->at(k)->coefficient_d;
			}
		}
	}
	//_myMatrix->setAtlasPos(robotPos);
	  C22_pub.publish(outMsg);
}
void C22_Node::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,geometry_msgs::Point pose){
	boost::mutex::scoped_lock l(_myMatrix->mutex);
	_myMatrix->computeMMatrix(map_cloud,robotPos);
}

C22_Node *node22;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "c22_compactGroundReconition_and_mapping");
  node22=new C22_Node();
  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
