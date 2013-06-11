#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "geometry_msgs/Polygon.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include <pcl/ros/conversions.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <pcl_ros/transforms.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <iomanip>	//for setw() and setfill() in print
#include <vector>
//for saving pcd
#include <iostream>
#include <pcl/io/pcd_io.h>
//for pcl viewer
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "opencv2/stitching/stitcher.hpp"
#include "std_msgs/Empty.h"
#include <laser_geometry/laser_geometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>



namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;
using namespace tf;

float maxEnt=0;
float minEnt=999999;
float maxHist=1;
float varHist=1;
float divHist=1;
float distHist=1;
float minHist=999999;
unsigned int SIZEX=60,SIZEY=60;
unsigned int CAMERA_IMAGE_WIDTH=800, CAMERA_IMAGE_HEIGHT=800;

pcl::PointCloud<pcl::PointXYZRGB>* roadCloud;

vector< cv::Scalar> *colors;

cv::Scalar randomColor(){
	return CV_RGB(rand()&255, rand()&255, rand()&255 );
}
void generateRandomColors(vector< cv::Scalar> *colors){
	for (unsigned int i=0;i<SIZEX*SIZEY;i++){
		colors->push_back(randomColor());
	}
}

struct EntropyMatrixCell{
	float entropy;
	int label;
	geometry_msgs::Point32* p;
	EntropyMatrixCell(){label=0;entropy=0;p=new geometry_msgs::Point32();}
	~EntropyMatrixCell(){
//		cout<<"Deleted EntropyMatrixCell: "<<toString()<<endl;
		delete p;
	}
	EntropyMatrixCell(float e_ , int l_):entropy(e_),label(l_){p=new geometry_msgs::Point32();p->x=0;p->y=0;p->z=0;}
	EntropyMatrixCell(float e_):entropy(e_),label(-1){p=new geometry_msgs::Point32();p->x=0;p->y=0;p->z=0;}
	EntropyMatrixCell(int l_):entropy(0),label(l_){p=new geometry_msgs::Point32();p->x=0;p->y=0;p->z=0;}
	string toString(){
		stringstream ss;
		ss<<"("<<entropy<<", "<<label<<", ("<<p->x<<", "<<p->y<<", "<<p->z<<") )";
		return ss.str();
	}
};

void printLookup(vector< vector<EntropyMatrixCell*>*  > * lookup){
	cout<<"print lookup:"<<endl;
	for(unsigned int i=0;i<lookup->size();i++){
		cout<<setw(4)<<setfill(' ')<<i<<":\t";
		for(unsigned int j=0;j<lookup->at(i)->size();j++){
			  EntropyMatrixCell e= *lookup->at(i)->at(j);
			  cout<<setw(12)<<setfill(' ')<<e.toString()<<"\t";
		  }
		  cout<<endl;
	}
	cout<<"Lookup Done."<<endl;
}


class RoadExtract{
public:
	pcl::PointCloud<pcl::PointXYZRGB> faker;
	RoadExtract():
		it_(nh),
		left_image_sub_( it_, "/multisense_sl/camera/left/image_raw", 1 ),
		pointcloud(nh, "/multisense_sl/camera/points2",1),
		laser_sub_(nh, "/multisense_sl/laser/scan", 10),
		laser_notifier_(laser_sub_,listener_, "pelvis", 10),
		sync( MySyncPolicy( 10 ), left_image_sub_,pointcloud)
	{
		roadCloud= new pcl::PointCloud<pcl::PointXYZRGB>();
		sync.registerCallback( boost::bind( &RoadExtract::callback, this, _1, _2 ) ); //Specifying what to do with the data
		laser_notifier_.registerCallback(boost::bind(&RoadExtract::lidarCallback, this, _1));
		laser_notifier_.setTolerance(ros::Duration(0.01));
		pub = nh.advertise<geometry_msgs::Polygon>("/RoadExtract",1000);
		ROS_INFO("Entropy Road - Cloud Server Online\n");
	}

	int fixDups(vector< vector<EntropyMatrixCell*>*  > * lookup, int labelToKeep, int labelToChange){
		if(labelToKeep != labelToChange){
			while (lookup->at(labelToChange)->size() > 0){
				EntropyMatrixCell* tmp = lookup->at(labelToChange)->back();
				tmp->label= labelToKeep;
				lookup->at(labelToChange)->pop_back();
				lookup->at(labelToKeep)->push_back(tmp);
			}
			return 0;
		}
		else
			return -1;
	}

	int findMinLabel(vector< vector<EntropyMatrixCell*>*  > * lookup, unsigned int n=0){
		if (n < lookup->size())
			for(unsigned int j=n;j<lookup->size();j++){
				if (lookup->at(j)->size() == 0)
						return j;
				}
		return -1;
	}

	int minimizeLabels(vector< vector<EntropyMatrixCell*>*  > * lookup){
		int minLabel = findMinLabel(lookup);
		//cout<<"minLabel ="<<minLabel<<", lookup->size()= "<<lookup->size()<<endl;
		for (int k=(int)lookup->size()-1;k>=0;k--){
			//cout<<"k= "<<k<<endl;
			if (lookup->at(k)->size() > 0){
				//cout<<"fixDups(lookup,"<<minLabel<<","<<k<<")"<<endl;
				if ((minLabel<k) && (fixDups(lookup,minLabel,k) == -1))
					return k;
				minLabel= findMinLabel(lookup);
				//cout<<"minLabel ="<<minLabel<<endl;
			}
		}
		return findMinLabel(lookup);
	}


	vector<int>* calcEntropyHistogram(vector< vector<EntropyMatrixCell*>* > *entropized){
		vector<int>* hist=new vector<int>();
		for(unsigned int i=0;i<SIZEX+1;i++)	//maxEnt
			hist->push_back(0);
		for(unsigned int i=0;i<entropized->size();i++){
			for(unsigned int j=0;j<entropized->at(i)->size();j++){
				float normalEntropy = entropized->at(i)->at(j)->entropy /maxEnt *SIZEX;
				hist->at((int)normalEntropy)=hist->at((int)normalEntropy)+1;
				maxHist=fmax(maxHist,hist->at((int)normalEntropy));
			}
		}
		for(unsigned int i=0;i<SIZEX+1;i++)	//maxEnt
			if(hist->at(i)>0){
				minHist=fmin(hist->at(i),minHist);
				distHist++;
			}
		for(unsigned int i=0;i<SIZEX+1;i++)	//maxEnt
				if(hist->at(i)>0){
					varHist+=std::pow(hist->at(i)-SIZEX*SIZEX/distHist,2);
				}
		divHist=sqrt(varHist);
		return hist;
	}


	//returns number of labels distributed (maxLabel)
	int calcLabels(vector< vector<EntropyMatrixCell*>* > *entropized){
		int count = 0;
		vector<int> *EnthropyHistogram=calcEntropyHistogram(entropized);
		vector<EntropyMatrixCell*>* neighbors = new vector<EntropyMatrixCell*>();
		vector< vector<EntropyMatrixCell*>*  > * lookup= new vector< vector<EntropyMatrixCell*>*  >();
		for (unsigned int i=0;i<entropized->size();i++){
			for (unsigned int j=0;j<entropized->at(i)->size();j++){
				EntropyMatrixCell* cur = entropized->at(i)->at(j);
				if(i>0){
					neighbors->push_back(entropized->at(i-1)->at(j));
					if(j>0)
						neighbors->push_back(entropized->at(i-1)->at(j-1));
					if(j<entropized->at(i)->size()-1)
						neighbors->push_back(entropized->at(i-1)->at(j+1));
				}
				if(j>0)
					neighbors->push_back(entropized->at(i)->at(j-1));
				if(i<entropized->size()-1){
					neighbors->push_back(entropized->at(i+1)->at(j));
					if (j>0)
						neighbors->push_back(entropized->at(i+1)->at(j-1));
					if(j<entropized->at(i)->size()-1)
						neighbors->push_back(entropized->at(i+1)->at(j+1));
				}
				if(j<entropized->at(i)->size()-1)
					neighbors->push_back(entropized->at(i)->at(j+1));

				//loop over neighbors and find any similar cells
				for(unsigned int t=0;t<neighbors->size();t++){
					float curEntNormalized = cur->entropy / maxEnt;
					float neighTEntNormalized = neighbors->at(t)->entropy / maxEnt;
					if(cur->label==-1){
						if((neighbors->at(t)->label!=-1)){
							if((float(fmax(neighTEntNormalized,curEntNormalized))-fmin(neighTEntNormalized,curEntNormalized))*maxHist*1.6 < ((float(fmax(EnthropyHistogram->at(neighTEntNormalized*SIZEX),EnthropyHistogram->at(curEntNormalized*SIZEX)))-fmin(EnthropyHistogram->at(neighTEntNormalized*SIZEX),EnthropyHistogram->at(curEntNormalized*SIZEX))))){

								cur->label = neighbors->at(t)->label;
								lookup->at(cur->label)->push_back(cur);
							}
						}
					}
					else{
						if ((float(fmax(neighTEntNormalized,curEntNormalized))-fmin(neighTEntNormalized,curEntNormalized))*maxHist*1.6<  (float(fmax(EnthropyHistogram->at(neighTEntNormalized*SIZEX),EnthropyHistogram->at(curEntNormalized*SIZEX))-fmin(EnthropyHistogram->at(neighTEntNormalized*SIZEX),EnthropyHistogram->at(curEntNormalized*SIZEX)))) && (neighbors->at(t)->label!=-1) && (neighbors->at(t)->label!=cur->label)){
							fixDups(lookup,min(neighbors->at(t)->label,cur->label),max(neighbors->at(t)->label,cur->label));
						}
					}
				}

				if (cur->label==-1){//label wasn't updated in previous loop
					cur->label=count++;
					lookup->push_back(new vector<EntropyMatrixCell*>());
					lookup->at(cur->label)->push_back(cur);
				}
				neighbors->clear();
			}
		}

		int numOflabels = minimizeLabels(lookup);
		//printLookup(lookup);
		for (unsigned int i=0;i<lookup->size();i++)
			delete lookup->at(i);
		delete lookup;
		return numOflabels;
	}



	float calcEntropy(vector<int>* hist, Mat* dst){
		  float hist_sum=dst->rows*dst->cols;
		  float sumEnt=0;
		  for( int i=0;i<dst->cols;i++){
			  for( int j=0;j<dst->rows;j++){
				  int val=(int) dst->at<uchar>(i, j);
				  if((int)hist->at(val)==0){
					  cout << "got zero at : " <<val <<endl;
				  }
				  float p=float(hist->at(val))/hist_sum;
				  float logp=std::log(p);
				  float ent= (-1)*p*logp;
				  sumEnt+= ent;
			  }
		  }
		  maxEnt=std::max(maxEnt,sumEnt);
		  minEnt=std::min(minEnt,sumEnt);
		  return sumEnt;
	}


	vector<int>* calcHistogram(Mat * window){
		vector<int>* hist=new vector<int>();
		for(int i=0;i<256;i++)
			hist->push_back(0);
		for(int i=0;i<window->cols;i++){
			for(int j=0;j<window->rows;j++){
				hist->at((int)window->at<uchar>(i, j))=hist->at((int)window->at<uchar>(i, j))+1;
			}
		}
		return hist;
	}
	//Calculates the label that appears the most in the bottom half of the picture, which is assumed to be the floor.
	//Returns the biggest label found.
	unsigned int calcBiggestLabel(vector< vector<EntropyMatrixCell*>* > *entropized){
		vector<unsigned int>* labelCounter= new vector<unsigned int>();
		for (unsigned int i=0;i<((entropized->size())*entropized->at(0)->size());i++){
			labelCounter->push_back(0);
		}

		for (unsigned int i=0;i<entropized->size()/2;i++){
			for (unsigned int j=0;j<entropized->at(i)->size();j++){
				labelCounter->at(entropized->at(i)->at(j)->label)++;
			}
		}
		//search for biggest label
		unsigned int bigLabel=0;
		unsigned int count=0;
		for (unsigned int i=0;i<labelCounter->size();i++){
			if (labelCounter->at(i)>count){
				bigLabel=i;
				count=labelCounter->at(i);
			}
		}
		delete labelCounter;
//		cout<<"Floor label: "<<bigLabel<<" ("<<count<<" times)"<<endl;
		return bigLabel;
	}


	/*
	 * help method to display computed entropy matrix
	 */
	void printEntroprized(vector< vector<EntropyMatrixCell*>* > *entropized){
		cout<<"print entropized squares:"<<endl;
		for(unsigned int i=0;i<entropized->size();i++){
			  for(unsigned int j=0;j<entropized->at(i)->size();j++){
				  EntropyMatrixCell e= *entropized->at(j)->at(i);
				  cout<<setw(12)<<setfill(' ')<<e.toString()<<"\t";
			  }
			  cout<<endl;
		}
		cout<<"Done."<<endl;
	}

	Mat fromSensorMsg(const sensor_msgs::ImageConstPtr& msg)
	  {
	    cv_bridge::CvImagePtr cv_ptr;
	    try
	    {
	      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR16);
	    }
	    catch (cv_bridge::Exception& e)
	    {
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	      exit(1);
	    }
	    return cv_ptr->image;
	  }

	vector<int>* obj_process(int xMin,int yMin,int xMax,int yMax){
	  //ROS_INFO("recived request, tying to fetch data\n");
	  tf::StampedTransform transform;
	  static  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laser;
	  try{
		listener.lookupTransform("/head","/pelvis",
				   ros::Time(0), transform);
	  }
	  catch (tf::TransformException ex){
		return false;
	  }

	  /*tf::Transform trans;
			 trans.setOrigin(tf::Vector3(0.0,-0.002, 0.035 ));
			 trans.setRotation(tf::Quaternion(-1.57,3.14,1.57));
	  */

	  Eigen::Matrix4f sensorToPelvis;
	  pcl_ros::transformAsMatrix(transform.inverse(), sensorToPelvis);


	  double x=0;
	  double y=0;
	  double z=0;
	  double counter=0;
	  pcl::PointCloud<pcl::PointXYZRGB> t;

	  for(int i=yMin;i<=yMax;i++){
		  for(int j=xMin;j<=xMax;j++){
			  pcl::PointXYZRGB p=faker.at(i,j);
			  t.push_back(p);
		  }
	  }

	   z=x=y=counter=0;


	   pcl::transformPointCloud(t, t, sensorToPelvis);


	   double  x_ret,y_ret,z_ret;
	   for(unsigned int i=0;i<t.points.size();i++){
		   if(t.points.at(i).x!=t.points.at(i).x)
			   continue;
		   if(x_ret==t.points.at(i).x && y_ret==t.points.at(i).y && z_ret==t.points.at(i).z)
			   break;
		   x_ret=t.points.at(i).x;
		   y_ret=t.points.at(i).y;
		   z_ret=t.points.at(i).z;
	   }

	   for(unsigned int i=0;i<t.points.size();i++){
		   if(t.points.at(i).x!=t.points.at(i).x)
			   continue;
		   if(x_ret==t.points.at(i).x && y_ret==t.points.at(i).y && z_ret==t.points.at(i).z)
			   continue;


		   x+=t.points.at(i).x;
		   y+=t.points.at(i).y;
		   z+=t.points.at(i).z;
		   counter++;
	   }
	   vector<int>* pos=new vector<int>();

	   pos->push_back(((double)x)/counter);
	   pos->push_back(((double)y)/counter);
	   pos->push_back(((double)z)/counter);
	   return pos;
	}

	void viewCloud(vector< vector<EntropyMatrixCell*>* > * entropized,int biggestLabel){

		pcl::PointCloud<pcl::PointXYZRGB> cloud;


		for (unsigned int i=0;i<entropized->size();i++){
			for (unsigned int j=0;j<entropized->at(i)->size();j++){
				EntropyMatrixCell* curCell = entropized->at(i)->at(j);
				if (curCell->label == biggestLabel){

					  tf::StampedTransform transform;
					  static  pcl::PointCloud<pcl::PointXYZRGB>::Ptr laser;
					  try{
						listener.lookupTransform("/head","/pelvis",
								   ros::Time(0), transform);
					  }
					  catch (tf::TransformException ex){
						//return false;
					  }

					  Eigen::Matrix4f sensorToPelvis;
					  pcl_ros::transformAsMatrix(transform.inverse(), sensorToPelvis);

					  pcl::PointCloud<pcl::PointXYZRGB> t;
					  double xMin=(CAMERA_IMAGE_HEIGHT/SIZEX)*i;
					  double yMin=(CAMERA_IMAGE_WIDTH/SIZEY)*j;
					  double xMax=(CAMERA_IMAGE_HEIGHT/SIZEX)*(i+1);
					  double yMax=(CAMERA_IMAGE_WIDTH/SIZEY)*(j+1);

					  for(int i=yMin;i<=yMax;i++){
						  for(int j=xMin;j<=xMax;j++){
							  pcl::PointXYZRGB p=faker.at(i,j);
							  t.push_back(p);
						  }
					  }
					   pcl::transformPointCloud(t, t, sensorToPelvis);

					   cloud+=t;
					}
				}
			}

		   //... populate cloud
		   pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
		   viewer.showCloud(cloud.makeShared());
		   while (!viewer.wasStopped ())
		   {
		   }

	}

	void callback(const sensor_msgs::ImageConstPtr& imgMsg,const sensor_msgs::PointCloud2::ConstPtr &pclMsg){
//		cout<<"entering callback"<<endl;
		Mat src,canny,blurred,cdst;
			  vector<vector<Mat> *> *windows=new vector<vector<Mat> *>();
			  for(unsigned int i=0;i<SIZEX;i++){
				  windows->push_back(new vector<Mat>());
			  }

			  // Load image
			  src = fromSensorMsg(imgMsg);
			  leftImage=fromSensorMsg(imgMsg);

			  colors = new vector< cv::Scalar >();
			  generateRandomColors(colors);

			  int oldType=src.type();
			  GaussianBlur(src, src,Size(15,15),15);
			  Mat *back=new Mat(src.rows,src.cols,oldType);
			  cv::cvtColor(src,src,COLOR_RGB2GRAY);
			  for(unsigned int i=0;i<SIZEY;i++){
				  for(unsigned int j=0;j<SIZEX;j++){
					  Rect roi(src.cols*(i)/SIZEX,src.rows*(j)/SIZEX,src.cols/SIZEX,src.rows/SIZEX);	//needs to be top-left corner according to docs
					  Mat dst_m = Mat(src, roi);
					  windows->at(i)->push_back(dst_m);
				  }
			  }

			  vector< vector<EntropyMatrixCell*>* > *entropized=new vector<vector<EntropyMatrixCell*> *>();

			//  bool uniform = true; bool accumulate = false;

			  vector<EntropyMatrixCell*> *entsFound = new vector<EntropyMatrixCell*>;
			  // Compute the histograms:

			  for(unsigned int i=0;i<SIZEY;i++){
				  entropized->push_back(new vector<EntropyMatrixCell*>());	//create vector* in place i
				  for(unsigned int j=0;j<SIZEX;j++){
					  vector<int> *hist=calcHistogram(&windows->at(i)->at(j));

					  float sumEnt = calcEntropy(hist,&windows->at(i)->at(j));
					  delete hist;
					  EntropyMatrixCell *emc;
					  emc = new EntropyMatrixCell(sumEnt);
					  entsFound->push_back(emc);
					  entropized->at(i)->push_back(emc);	//essentially push sumEnt into place j of sub-vector
				  }
			  }


			  int numOfLabels = calcLabels(entropized);
//			  cout<<"Number of labels distributed: "<<numOfLabels<<endl;
//			  printEntroprized(entropized);

			  //blend with color

			  for (unsigned int i=0;i<entropized->size();i++){
				  for (unsigned int j=0;j<entropized->at(i)->size();j++){
					  cv::Rect roi( cv::Point(windows->at(i)->at(j).cols*i,windows->at(i)->at(j).rows*j), cv::Size(windows->at(i)->at(j).cols,windows->at(i)->at(j).rows));
					  Mat resetgray;
					  cvtColor(windows->at(i)->at(j),resetgray,COLOR_GRAY2RGB);
					  Mat coloredWin(windows->at(i)->at(j).rows,windows->at(i)->at(j).cols,oldType);
//					  cout<<"colors->size= "<<colors->size()<<endl;
//					  cout<<"entropized->at(i)->at(j)->label= "<<entropized->at(i)->at(j)->label<<endl;
					  coloredWin=colors->at(entropized->at(i)->at(j)->label);
					  addWeighted( resetgray, 0.5, coloredWin, 0.5, 0.0, coloredWin);
					  coloredWin.copyTo( Mat(*back, roi) );
				  }
			  }

			  unsigned int biggestLabel=calcBiggestLabel(entropized);

				pcl::PointCloud<pcl::PointXYZRGB>cloudORG;
				pcl::fromROSMsg<pcl::PointXYZRGB>(*pclMsg,cloudORG);

				pcl::PointCloud<pcl::PointXYZRGB>cloud;
				static tf::StampedTransform transform;
				  Eigen::Matrix4f sensorTopelvis;
				  tf::Transform trans;
				  trans.setOrigin(tf::Vector3(0,0,0));
				  trans.setRotation(tf::Quaternion(0,0,0));
				  pcl_ros::transformAsMatrix(transform, sensorTopelvis);
				  pcl::transformPointCloud(cloudORG, cloud, sensorTopelvis);


				geometry_msgs::Polygon roadCloudMsg;
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr roadCloud_ptr(roadCloud->makeShared());
				if(faker.points.size()<20)
					return;
				//viewCloud(entropized,biggestLabel);
				for (unsigned int i=0;i<entropized->size();i++){
					for (unsigned int j=0;j<entropized->at(i)->size();j++){
						EntropyMatrixCell* curCell = entropized->at(i)->at(j);
						if (curCell->label == biggestLabel){
							vector<int>* pos = obj_process((CAMERA_IMAGE_HEIGHT/SIZEX)*i,(CAMERA_IMAGE_WIDTH/SIZEY)*j,(CAMERA_IMAGE_HEIGHT/SIZEX)*(i+1),(CAMERA_IMAGE_WIDTH/SIZEY)*(j+1));
							curCell->p->x=pos->at(0);
							curCell->p->y=pos->at(1);
							curCell->p->z=pos->at(2);
							geometry_msgs::Point32 p1;
							p1.x=pos->at(0)+0.2;
							p1.y=pos->at(1);
							p1.z=pos->at(2);
							roadCloudMsg.points.push_back(p1);
							geometry_msgs::Point32 p2;
							p2.x=pos->at(0)-0.2;
							p2.y=pos->at(1);
							p2.z=pos->at(2);
							roadCloudMsg.points.push_back(p2);
							geometry_msgs::Point32 p3;
							p3.x=pos->at(0);
							p3.y=pos->at(1)+0.2;
							p3.z=pos->at(2);
							roadCloudMsg.points.push_back(p3);
							geometry_msgs::Point32 p4;
							p4.x=pos->at(0);
							p4.y=pos->at(1)-0.2;
							p4.z=pos->at(2);
							roadCloudMsg.points.push_back(p4);
							roadCloudMsg.points.push_back(*curCell->p);
						}
					}
				}

//				cout<<"Publishing..."<<endl;
				pub.publish(roadCloudMsg);


			  ////////////////////////////////
			  //		cleanup			//////
			  ////////////////////////////////
				delete back;

			  for(unsigned int i=0;i<entsFound->size();i++){
				  delete entsFound->at(i);
			  }

			  delete entsFound;

			  for(unsigned int i=0;i<entropized->size();i++){
				  vector<EntropyMatrixCell*> tmp;
				  entropized->at(i)->swap(tmp);
				delete entropized->at(i);
			   }

			  vector<vector<EntropyMatrixCell*>*> tmp;
			  entropized->swap(tmp);
			  delete entropized;

			  for(unsigned int i=0;i<windows->size();i++){
				  vector<Mat> tmp;
				  windows->at(i)->swap(tmp);
				  delete windows->at(i);
			   }
			  vector<vector<Mat> *> tmpWindows;
			  windows->swap(tmpWindows);
			  delete windows;


			  vector<cv::Scalar> tmpColors;
			  colors->swap(tmpColors);
			  delete colors;
//			  cout<<"Exiting callback"<<endl;
	}


	void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in){
			      sensor_msgs::PointCloud2 cloud;
			      try{
			        projector_.transformLaserScanToPointCloud("/left_camera_frame",*scan_in,
			  		          cloud,listener_);
			      }
			      catch (tf::TransformException ex){
			        return;
			      }

			  		pcl::PointCloud<pcl::PointXYZ> temp;
			  		pcl::fromROSMsg<pcl::PointXYZ>(cloud,temp);
			  		pcl::PointCloud<pcl::PointXYZRGB> cloud2;

			  		faker.width    = 800;
			  		faker.height   = 800;
			  		faker.is_dense = false;
			  		faker.points.resize (faker.width * faker.height);
			  		for(unsigned int i=0;i<temp.points.size();i++){
			  			if(scan_in->ranges.at(i)<29 && scan_in->ranges.at(i)>0.5)
			  				{

							  pcl::PointXYZRGB p;
							  p.x=temp.points.at(i).x;
							  p.y=temp.points.at(i).y;
							  p.z=temp.points.at(i).z;
							  int xpix,ypix;
							  xpix=400+(int)(-482*p.z/p.x);
							  ypix=400+(int)(-482*p.y/p.x);
							  //	stay in frame
							  xpix=xpix>0?xpix:-1;
							  xpix=xpix<799?xpix:800;
							  ypix=ypix>0?ypix:-1;
							  ypix=ypix<799?ypix:800;

							  p.r=leftImage.at<cv::Vec3b>(xpix,ypix)[0];


							  p.g=leftImage.at<cv::Vec3b>(xpix,ypix)[1];

							  p.b=leftImage.at<cv::Vec3b>(xpix,ypix)[2];

							  if(ypix>799 || ypix<0 || xpix>799 ||xpix<0)
							    {p.r=255;p.b=255;p.g=255;}
							    else faker.at(xpix,ypix)=p;
			  				  // pcl::io::savePCDFile ("test_pcd.pcd",* laser);

			  				  //cloud2.points.push_back(p);//temp.points.at(i));
			  				}
			  		}


	}
private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub;

	image_transport::ImageTransport it_;
	typedef image_transport::SubscriberFilter ImageSubscriber;
	ImageSubscriber left_image_sub_;
	cv::Mat leftImage;
	laser_geometry::LaserProjection projector_;
	tf::TransformListener listener_;
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud;
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
	tf::TransformListener listener;
	typedef message_filters::sync_policies::ApproximateTime<
	sensor_msgs::Image,sensor_msgs::PointCloud2
	> MySyncPolicy;
	message_filters::Synchronizer< MySyncPolicy > sync;

};

/**
 * @function main
 */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "RoadExtract");
  RoadExtract* re = new RoadExtract();
  ros::spin();
  delete re;
  return 0;
}


