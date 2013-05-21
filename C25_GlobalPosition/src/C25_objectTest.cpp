#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/subscriber_filter.h>
#include "geometry_msgs/Point.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
namespace enc=sensor_msgs::image_encodings;
using namespace tf;
using namespace cv;

Mat *myImage;
void my_mouse_callback( int event, int x, int y, int flags, void* param );

Rect box;
bool drawing_box = false;
bool box_chosen = false;
int x1,y2;
double minx,maxx,miny,maxy;


void draw_box( Mat* img, Rect rect ){
	cv::rectangle( *img, rect,
				Scalar(255,100,100) );
}

// Implement mouse callback
void my_mouse_callback( int event, int x, int y, int flags, void* param ){
	Mat* image = myImage;
	switch( event ){
		case CV_EVENT_MOUSEMOVE:
			if( drawing_box ){
				box.width = x-box.x;
				box.height = y-box.y;
			}
			break;

		case CV_EVENT_LBUTTONDOWN:
			box_chosen = false;
			drawing_box = true;
			box = cvRect( x, y, 0, 0 );
			x1=x;
			y2=y;
			break;

		case CV_EVENT_LBUTTONUP:
			drawing_box = false;
			if( box.width < 0 ){
				box.x += box.width;
				box.width *= -1;
			}
			if( box.height < 0 ){
				box.y += box.height;
				box.height *= -1;
			}
			minx=std::min(x1,x);
			maxx=std::max(x1,x);
			miny=std::min(y2,y);
			maxy=std::max(y2,y);
			box_chosen = true;
			break;
	}
}


class C25_objectTest{

public:

	C25_objectTest() :
		it_(nh_)
		{
				left_image_sub_= it_.subscribe("/multisense_sl/camera/left/image_color", 1,&C25_objectTest::imagecallback,this );
				objpub= nh_.advertise<C23_ObjectRecognition::C23C0_ODIM>("C23/object_deminsions", 1);
				objsub=nh_.subscribe("C23/objectLocation", 1,&C25_objectTest::c25Callback,this);
		}

	void c25Callback(const geometry_msgs::Point::Ptr & msg){
		std::cout<<"the object is at x:"<<msg->x<<" y:"<<msg->y<<" z:"<<msg->z<<std::endl;
	}

	void imagecallback(const sensor_msgs::ImageConstPtr& left_msg){
		cv_bridge::CvImagePtr left;
		try
		{
		left = cv_bridge::toCvCopy(left_msg,enc::RGB8);
		}
		catch (cv_bridge::Exception& e)
		{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}
		myImage=&left->image;
		if(box_chosen)
			draw_box( myImage, box );
		imshow("Window",*myImage);

		if(!box_chosen)
			return;

		C23_ObjectRecognition::C23C0_ODIM msg;
		msg.x=minx;
		msg.y=miny;
		msg.height=maxy-miny;
		msg.width=maxx-minx;
		objpub.publish(msg);
	}

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  typedef image_transport::Subscriber ImageSubscriber;
  ImageSubscriber left_image_sub_;
  ros::Publisher objpub;
  ros::Subscriber objsub;
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv,"cloud_fetch");
	const char* name = "Window";
	box = cvRect(-1,-1,0,0);
	C25_objectTest *cf=new C25_objectTest;

	cvNamedWindow( "Window" );
	cvStartWindowThread();
	// Set up the callback
	cvSetMouseCallback( name, my_mouse_callback, (void*) myImage);
	while(ros::ok()){
		ros::spin();
	}
	// Main loop
	cvDestroyWindow( name );

	return 0;
}
