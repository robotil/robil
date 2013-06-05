#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <pcl/correspondence.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <image_transport/subscriber_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>

#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include "C23_Detector.hpp"
#include <C21_VisionAndLidar/C21_obj.h>
#include <math.h>
#include <std_srvs/Empty.h>
#include <ros/package.h>


#define MIN(x,y) (x < y ? x : y)
#define MAX(x,y) (x > y ? x : y)



bool C23_Detector::pictureCoordinatesToGlobalPosition(double x1, double y1, double x2, double y2, double* x, double* y, double*z) {
    C21_VisionAndLidar::C21_obj c21srv;
    c21srv.request.sample.x1 = x1;
    c21srv.request.sample.y1 = y1;
    c21srv.request.sample.x2 = x2;
    c21srv.request.sample.y2 = y2;
    
    if(c21client.call(c21srv))
    {
        if(x != NULL) *x = round(c21srv.response.point.x);
        if(y != NULL) *y = round(c21srv.response.point.y);
        if(z != NULL) *z = round(c21srv.response.point.z);
        cout << "Received data: " << c21srv.response.point.x << "," << c21srv.response.point.y << "," << c21srv.response.point.z << endl;
        return true;
    }
    return false;
    
}

bool C23_Detector::pointCloudCoordinatesToGlobalPosition(double x, double y, double z, double* px, double* py, double *pz) {
    C21_VisionAndLidar::C21_obj c21srv;
    c21srv.request.sample.x1 = x;
    c21srv.request.sample.y1 = z;
    c21srv.request.sample.x2 = y;
    c21srv.request.sample.y2 = 0;
    
    if(c21client.call(c21srv))
    {
        if(px != NULL) *px = round(c21srv.response.point.x);
        if(py != NULL) *py = round(c21srv.response.point.y);
        if(pz != NULL) *pz = round(c21srv.response.point.z);
        cout << "Received data: " << c21srv.response.point.x << "," << c21srv.response.point.y << "," << c21srv.response.point.z << endl;
        return true;
    }
    return false;
    
}

bool C23_Detector::averagePointCloud(int x1, int y1, int x2, int y2, const sensor_msgs::PointCloud2::ConstPtr &cloud, double* px, double* py, double *pz) {
    
    
    int xMin=std::min(x1,x2);
    int yMin=std::min(y1,y2);
    int xMax=std::max(x1,x2);
    int yMax=std::max(y1,y2);
    double tmp_x = 0;
    double tmp_y =0;
    double tmp_z  =0;
    pcl::PointCloud<pcl::PointXYZ>detectionCloud;
    pcl::fromROSMsg<pcl::PointXYZ>(*cloud,detectionCloud);
    double _x=0;
    double _y=0;
    double _z=0;
    int counter=0;
    pcl::PointCloud<pcl::PointXYZ> t;
    pcl::PointXYZ p;
    for(int i=xMin;i<=xMax;i++) {
        for(int j=yMin;j<=yMax;j++){
            p=detectionCloud.at(i,j);
            if(p.x!=p.x)
                continue;
            //if(p.x>0.3 && p.y>0.3) {
	      //cout<<"Here"<<endl;
	      _x+=p.x;
	      _y+=p.y;
	      _z+=p.z;
	      
	     // cout<<"x,y,z: "<<x<<", "<<y<<", "<<z<<endl;
	      //cout<<"px, py, pz: "<<p.x<<", "<<p.y<<", "<<p.z<<endl;
	      counter++;   
             //   break;
              //  cout << "Found a fucking point" << endl;
          //  }
        }
    }
    cout<<"Counter: "<<counter<<endl;
   //Calculate the average point
   tmp_x=_x/(counter);
   tmp_y=_y/(counter);
   tmp_z=_z/(counter);
   
    cout << "Point is: " << tmp_x << "," <<tmp_y << "," << tmp_z << endl;
    C21_VisionAndLidar::C21_obj c21srv;
    
    if(_target==HANDBRAKE || _target == INSIDE_STEERINGWHEEL || _target ==GEAR)
    {
      *px = tmp_x;
      *py = tmp_y;
      *pz = tmp_z;
      
    }
    else{
       c21srv.request.sample.x1 = tmp_x;
      c21srv.request.sample.x2 =tmp_y;
      c21srv.request.sample.y1 =tmp_z;
      c21srv.request.sample.y2 = 0;
      
      if(c21client.call(c21srv))
      {
	if(px != NULL) *px = round(c21srv.response.point.x);
	if(py != NULL) *py = round(c21srv.response.point.y);
	if(pz != NULL) *pz = round(c21srv.response.point.z);
	cout << "Received data: " << c21srv.response.point.x << "," << c21srv.response.point.y << "," << c21srv.response.point.z << endl;
	return true;
      }
    }
    
    return false;
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


C23_Detector::C23_Detector(const char* left_cam, const char* right_cam, const char* pointc):
it_(nh),
left_image_sub_( it_, left_cam, 1 ),
pointcloud(nh,pointc,1),
_target(NONE),
sync( MySyncPolicy( 10 ), left_image_sub_,pointcloud)
{
    
    sync.registerCallback( boost::bind( &C23_Detector::callback, this, _1, _2 ) ); //Specifying what to do with the data
    objectDetectedPublisher = nh.advertise<C23_ObjectRecognition::C23C0_OD>("C23/object_detected", 1);
    objectDeminsionsPublisher = nh.advertise<C23_ObjectRecognition::C23C0_ODIM>("C23/object_deminsions", 1);
    c21client = nh.serviceClient<C21_VisionAndLidar::C21_obj>("C21/C23"); //Subscribe to the service node to get the absolute coordinates of a point
    c23_start_posecontroller = nh.serviceClient<std_srvs::Empty>("/PoseController/start");
    c23_stop_posecontroller = nh.serviceClient<std_srvs::Empty>("/PoseController/stop");
    
    ROS_INFO("Started...");
    //	gates = new vector<Gate*>();
    
}


bool C23_Detector::detect(const string target) {
    
    //   ROS_INFO(target.c_str());
    
    if(!target.compare("Gate")) {
        _target = GATE;
        //  ROS_INFO("We are looking for a gate...");
    } else if (!target.compare("Car")) {
        _generalDetector.initialize("carEntry");
        _target = CAR;
        ROS_INFO("We are looking for a Car...");
    } else if (!target.compare("Path")) {
        _target = PATH;
        ROS_INFO("We are looking for a path...");
    } else if (!target.compare("Valve")) {
        _target = VALVE;
        ROS_INFO("We are looking for a valve...");
    } else if (!target.compare("Standpipe")) {
        _target = STANDPIPE;
        ROS_INFO("We are looking for a valve...");
    } else if (!target.compare("Firehose")) {
        _target = FIREHOSE;
        ROS_INFO("We are looking for a firehose...");
    } else if (!target.compare("FirehoseGrip")) {
        _target = FIREHOSE_GRIP;
        ROS_INFO("We are looking for a firehose...");
    } else if (!target.compare("Table")) {
        _target = TABLE;
        ROS_INFO("We are looking for a Table...");
    } else if (!target.compare("Picture")) {
        _target = PICTURE;
        ROS_INFO("We are taking a picture ... say cheese ...");
    }
    else if (!target.compare("InsideSteeringWheel")) {
        _target = INSIDE_STEERINGWHEEL;
        ROS_INFO("We are looking for the steering wheel while inside the car...");
    }
    else if (!target.compare("OutsideSteeringWheel")) {
        _target = OUTSIDE_STEERINGWHEEL;
        ROS_INFO("We are looking for the steering wheel while outside the car...");
    }
    else if (!target.compare("Handbrake")) {
        _target = HANDBRAKE;
        ROS_INFO("We are looking for the handbrake while inside the car...");
    }
    else if (!target.compare("Gear")) {
        _target = GEAR;
        ROS_INFO("We are looking for the gear inside the car...");
    }
    return true;
    
}
void C23_Detector::publishMessage(bool isFound) {
    //   ROS_INFO("Publishing message..");
    C23_ObjectRecognition::C23C0_OD msg;
    C23_ObjectRecognition::C23C0_ODIM msg2;
    string target;
    switch (_target) {
        
        case CAR:
            target = "Car";
            break;
        case GATE:
            target = "Gate";
            break;
    }
    if(!isFound) {
        x= - 1;
    }
    msg.ObjectDetected = isFound ? 1 : 0;
    msg2.x = x;
    msg2.y = y;
    msg2.width = width;
    msg2.height = height;
    msg2.Object = target;
    msg.Object = target;
    //  ROS_INFO("Publishing message..");
    objectDeminsionsPublisher.publish(msg2);
    objectDetectedPublisher.publish(msg);
    
    
}
void C23_Detector::callback(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    //    ROS_INFO("Receiving image..");
    Mat srcImg = fromSensorMsg(msg);
    bool res;
    switch (_target) {
        case PATH:
            res = detectPath(srcImg);
            publishMessage(res);
            break;
        case CAR:
            ROS_INFO("CAR");
            res = detectCar(srcImg,cloud);
            publishMessage(res);
            break;
        case GATE:
            ROS_INFO("GATE");         
            res = detectGate(srcImg,cloud);
            publishMessage(res);
            break;
        case VALVE:
            ROS_INFO("VALVE");
            res = detectValve(srcImg,cloud);
            publishMessage(res);
            break;
        case FIREHOSE:
            ROS_INFO("FIREHOSE");
            res = detectFirehose(srcImg,cloud);
            publishMessage(res);
            break;
            
        case STANDPIPE:
            ROS_INFO("STANDPIPE");
            res = detectStandpipe(srcImg,cloud);
            publishMessage(res);
            break;
        case FIREHOSE_GRIP:
            ROS_INFO("FIREHOSE_GRIP");
            res = detectFirehoseGrip(srcImg,cloud);
            publishMessage(res);
            break;
        case TABLE:
            ROS_INFO("TABLE");
            res = detectTable(srcImg,cloud);
            publishMessage(res);
            break;
        case PICTURE:
            ROS_INFO("TAKE_PICTURE");
            takePictures(srcImg);
            _target=NONE;
            break;
        case INSIDE_STEERINGWHEEL:
            ROS_INFO("INSIDE_STEERINGWHEEL");
            res = detectSteeringWheel(srcImg,cloud,0);//Value of 1 for detection from outside the car. Value of 0 for detection within the car
            publishMessage(res);
            break;
        case OUTSIDE_STEERINGWHEEL:
            ROS_INFO("OUTSIDE_STEERINGWHEEL");
            res = detectSteeringWheel(srcImg,cloud,1);
            publishMessage(res);
            break;
        case HANDBRAKE:
            ROS_INFO("HANDBRAKE");
            res = detectHandbrake(srcImg,cloud,0);
            publishMessage(res);
            break;
        case GEAR:
            ROS_INFO("GEAR");
            res = detectGear(srcImg,cloud,1);
            publishMessage(res);
            break; 
            
    }
    srcImg.release();
    
}


bool C23_Detector::takePictures(Mat srcImg){
    
    
    imwrite("picture.jpg",srcImg);
    
}


//Use for template matching with the car
bool C23_Detector::templateMatching( Mat img, Mat templ, int matching_method, cv::Point *matchLoc, const sensor_msgs::PointCloud2::ConstPtr &cloud)
{
    // Source image to display
    Mat img_display;
    Mat result; //The result matrix
    img.copyTo( img_display );
    
    // Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;
    
    result.create( result_cols, result_rows, CV_32FC1 );
    
    // Do the Matching and Normalize
    matchTemplate( img, templ, result, matching_method );
    normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, Mat() );
    
    // Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    //cv::Point matchLoc;
    
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
    // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( matching_method  == 0 || matching_method == 1 )
    { *matchLoc = minLoc; }
    else
    { *matchLoc = maxLoc; }
    
    // Show me what you got
    rectangle( img_display, *matchLoc, cv::Point( matchLoc->x + templ.cols , matchLoc->y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    //rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
    
    cout<<" Before Gear x,y,z: "<<x<<", "<<y<<", "<<z<<endl;
    averagePointCloud(matchLoc->x, matchLoc->y, matchLoc->x + templ.cols, matchLoc->y + templ.rows, cloud, &x, &y, &z); 
    
    if (_target==HANDBRAKE){
      //Offset from the gear to the handbrake
      cout<<" Gear x,y,z: "<<x<<", "<<y<<", "<<z<<endl;
      x = x - 0.03;
      y = y + 0.09;
      z = z - 0.03;
      cout<<"Handbrake x,y,z: "<<x<<", "<<y<<", "<<z<<endl;
    }
      
    //pictureCoordinatesToGlobalPosition(matchLoc->x,matchLoc->y,matchLoc->x + templ.cols,matchLoc->y + templ.rows,&x,&y,NULL);
    
    //imshow( "Source Image", img_display );
    //waitKey(0);
    //imshow( "Result Window", result );
    
    return true;
}

//Detect the car steering wheel
bool C23_Detector::detectSteeringWheel(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location){
    cv::Point matchLoc;
    bool res = false;
    if (location==0)//Robot is inside the car
{
  
      //------------------------------------------------------------------
      // Load the object templates specified in the object_templates.txt file
      char basePath[1000],imageName[1000];

      sprintf(basePath,"%s/template_matching_images/%c",ros::package::getPath("C23_ObjectRecognition").c_str(),'\0');

      sprintf(imageName,"%s/steering_wheel_template_qual.jpg%c",basePath,'\0');
      std::cout<<imageName<<endl;
    //-----------------------------------------------------------------
  
  
    //Load the image template for the steering wheel
    Mat steeringwheelTemplate = imread(imageName);
    res  = templateMatching(srcImg, steeringwheelTemplate, 1, &matchLoc, cloud);
    
    
    imshow("Steering wheel template", steeringwheelTemplate);
    waitKey(0);
}
else{
    //Robot is outside the car
    
    
}




return res;
}



//Detect the car handbrake
bool C23_Detector::detectHandbrake(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location){
    cv::Point matchLoc;
    //Load the image template for the steering wheel
    //------------------------------------------------------------------
      // Load the object templates specified in the object_templates.txt file
      char basePath[1000],imageName[1000];

      sprintf(basePath,"%s/template_matching_images/%c",ros::package::getPath("C23_ObjectRecognition").c_str(),'\0');

      sprintf(imageName,"%s/gear_template_qual.jpg%c",basePath,'\0');
      std::cout<<imageName<<endl;
    //-----------------------------------------------------------------
    
    
    Mat gearTemplate = imread(imageName);
    

    
    //imshow("Hand brake template", handbrakeTemplate);
    //waitKey(0);
    
    bool res =  templateMatching(srcImg, gearTemplate, 1, &matchLoc, cloud);
    
    imshow("New Src Image", gearTemplate);
    waitKey();
    
   /* cout<<"Coordinates: "<<matchLoc.x<<", "<<matchLoc.y<<endl;
    const int OFFSET = 100;
    matchLoc.x = matchLoc.x - OFFSET;
    int y1 = gearTemplate.rows;
    matchLoc.y = matchLoc.y + y1;
    int x1 = gearTemplate.cols;
    
    cout<<"Coordinates New: "<<matchLoc.x<<", "<<matchLoc.y<<", "<<x1+ OFFSET<<", "<<y1<<endl;
    Rect rect(matchLoc.x, matchLoc.y, x1+ OFFSET, y1);
    
    Mat handbrakeTemplate = imread("template_matching_images/hand_brake_template_qual.jpg");
    Mat newSrcImg(srcImg,rect);
    //imshow("New Src Image", newSrcImg);
    //waitKey();
    res =  templateMatching(newSrcImg, handbrakeTemplate, 1, &matchLoc, cloud);*/
 
    return res;
}

//Detect the car gear
bool C23_Detector::detectGear(Mat srcImg,const sensor_msgs::PointCloud2::ConstPtr &cloud,int location){
    cv::Point matchLoc;
    //Load the image template for the steering wheel
        //------------------------------------------------------------------
      // Load the object templates specified in the object_templates.txt file
      char basePath[1000],imageName[1000];

      sprintf(basePath,"%s/template_matching_images/%c",ros::package::getPath("C23_ObjectRecognition").c_str(),'\0');

      sprintf(imageName,"%s/gear_template_qual.jpg%c",basePath,'\0');
      std::cout<<imageName<<endl;
    //-----------------------------------------------------------------
    
    Mat gearTemplate = imread(imageName);
    imshow("Gear template", gearTemplate);
    waitKey(0);
    
    bool res  = templateMatching(srcImg, gearTemplate, 1, &matchLoc, cloud);
    
    
    return res;
}



bool C23_Detector::detectValve(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    ROS_INFO("Detecting a valve..");
    RNG rng(12345);
    
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(60,30,30),Scalar(80,255,255),imgThreshed);
    //namedWindow("TESTING");
    // imshow("TESTING",imgThreshed);
    // waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgThreshed,bw,10,255,CV_THRESH_BINARY);
    // imshow("TESTING",bw);
    // waitKey(0);
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(120, 120, 0);
    colors[1] = cv::Scalar(120, 255, 0);
    colors[2] = cv::Scalar(0, 100, 255);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    size_t idx;
    for (idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(srcImg, contours, idx, colors[idx % 3]);
    }
    
    //  drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    // imshow("TESSTING",srcImg);
    waitKey(0);
    vector<RotatedRect> minEllipse( contours.size() );
    int biggest_size = 0;
    int biggest = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
        if( contours[i].size() > biggest_size )
        { 
            biggest_size = contours[i].size();
            biggest = i;
            
        }
    }
    if(biggest_size > 30 && biggest_size < 350) {
        
        
        
        
        /* for( int i = 0; i < contours.size(); i++ )
         *            {
         *                cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
         *                if( contours[i].size() > 5 )
         *                    { 
         *                        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         *                        
         }
         }*/
        RotatedRect minRect =  minAreaRect( Mat(contours[biggest]));
        Point2f rect_points[4]; minRect.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( srcImg, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,0,0), 1, 8 );
        // ellipse( srcImg, fitEllipse( Mat(contours[biggest]) ), CV_RGB(255,0,0), 2, 8 );
            /// Draw contours + rotated rects + ellipses
            // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            /* for( int i = 0; i< contours.size(); i++ )
             *            {
             *                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             *                // contour
             *            //  drawContours( bw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Pointf() );
             *                // ellipse
             *                ellipse( imgThreshed, minEllipse[i], color, 2, 8 );
             }*/
            imshow("TESTING",srcImg);
            waitKey(0);
            pictureCoordinatesToGlobalPosition(minRect.center.x-10,minRect.center.y+10,minRect.center.x+10,minRect.center.y+10,&x,&y,NULL);
            return true;
    }
    return false;
    
}

bool C23_Detector::detectFirehose(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    ROS_INFO("Detecting a Firehose..");
    RNG rng(12345);
    
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(0, 200, 80), Scalar(1, 255, 250),imgThreshed);
    //namedWindow("TESTING");
    imshow("TESTING",imgThreshed);
    waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat imgDilated;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*18 + 1, 2*18+1 ),
                                         cv::Point( 18, 18 ) );
    /// Apply the dilation operation
    dilate( imgThreshed, imgDilated, element );
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgDilated,bw,10,255,CV_THRESH_BINARY);
    imshow("TESTING",bw);
    waitKey(0);
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(120, 120, 0);
    colors[1] = cv::Scalar(120, 255, 0);
    colors[2] = cv::Scalar(0, 100, 255);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    size_t idx;
    for (idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(srcImg, contours, idx, colors[idx % 3]);
    }
    
    //  drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    // imshow("TESSTING",srcImg);
    waitKey(0);
    vector<RotatedRect> minEllipse( contours.size() );
    int biggest_size = 0;
    int biggest = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
        if( contours[i].size() > biggest_size )
        { 
            biggest_size = contours[i].size();
            biggest = i;
            
        }
    }
    if(biggest_size > 50 && biggest_size < 180) {
        
        
        
        
        /* for( int i = 0; i < contours.size(); i++ )
         *            {
         *                cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
         *                if( contours[i].size() > 5 )
         *                    { 
         *                        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         *                        
         }
         }*/
        RotatedRect minRect =  minAreaRect( Mat(contours[biggest]));
        Point2f rect_points[4]; minRect.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( srcImg, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,0,0), 1, 8 );
        // ellipse( srcImg, fitEllipse( Mat(contours[biggest]) ), CV_RGB(255,0,0), 2, 8 );
            /// Draw contours + rotated rects + ellipses
            // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            /* for( int i = 0; i< contours.size(); i++ )
             *            {
             *                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             *                // contour
             *            //  drawContours( bw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Pointf() );
             *                // ellipse
             *                ellipse( imgThreshed, minEllipse[i], color, 2, 8 );
             }*/
            imshow("TESTING",srcImg);
            waitKey(0);
            pictureCoordinatesToGlobalPosition(minRect.center.x-100,minRect.center.y+100,minRect.center.x+100,minRect.center.y+100,&x,&y,NULL);
            return true;
    }
    return false;
    
}

bool C23_Detector::detectFirehoseGrip(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    ROS_INFO("Detecting a Firehose..");
    RNG rng(12345);
    
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(110, 200, 80), Scalar(125, 255, 250),imgThreshed);
    //namedWindow("TESTING");
    imshow("TESTING",imgThreshed);
    waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat imgDilated;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*18 + 1, 2*18+1 ),
                                         cv::Point( 18, 18 ) );
    /// Apply the dilation operation
    dilate( imgThreshed, imgDilated, element );
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgDilated,bw,10,255,CV_THRESH_BINARY);
    imshow("TESTING",bw);
    waitKey(0);
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(120, 120, 0);
    colors[1] = cv::Scalar(120, 255, 0);
    colors[2] = cv::Scalar(0, 100, 255);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    size_t idx;
    for (idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(srcImg, contours, idx, colors[idx % 3]);
    }
    
    //  drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    // imshow("TESSTING",srcImg);
    waitKey(0);
    vector<RotatedRect> minEllipse( contours.size() );
    int biggest_size = 0;
    int biggest = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
        if( contours[i].size() > biggest_size )
        { 
            biggest_size = contours[i].size();
            biggest = i;
            
        }
    }
    if(biggest_size > 50 && biggest_size < 250) {
        
        
        
        
        /* for( int i = 0; i < contours.size(); i++ )
         *            {
         *                cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
         *                if( contours[i].size() > 5 )
         *                    { 
         *                        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         *                        
    }
    }*/
        RotatedRect minRect =  minAreaRect( Mat(contours[biggest]));
        Point2f rect_points[4]; minRect.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( srcImg, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,0,0), 1, 8 );
        // ellipse( srcImg, fitEllipse( Mat(contours[biggest]) ), CV_RGB(255,0,0), 2, 8 );
            /// Draw contours + rotated rects + ellipses
            // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            /* for( int i = 0; i< contours.size(); i++ )
             *            {
             *                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             *                // contour
             *            //  drawContours( bw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Pointf() );
             *                // ellipse
             *                ellipse( imgThreshed, minEllipse[i], color, 2, 8 );
    }*/
            imshow("TESTING",srcImg);
            waitKey(0);
            pictureCoordinatesToGlobalPosition(minRect.center.x-100,minRect.center.y+100,minRect.center.x+100,minRect.center.y+100,&x,&y,NULL);
            return true;
    }
    return false;
    
}

bool C23_Detector::detectStandpipe(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    ROS_INFO("Detecting a Standpipe..");
    RNG rng(12345);
    
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(0, 120, 50), Scalar(3, 180, 70),imgThreshed);
    //namedWindow("TESTING");
    imshow("TESTING",imgThreshed);
    waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat imgDilated;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*18 + 1, 2*18+1 ),
                                         cv::Point( 18, 18 ) );
    /// Apply the dilation operation
    dilate( imgThreshed, imgDilated, element );
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgDilated,bw,10,255,CV_THRESH_BINARY);
    imshow("TESTING",bw);
    waitKey(0);
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(120, 120, 0);
    colors[1] = cv::Scalar(120, 255, 0);
    colors[2] = cv::Scalar(0, 100, 255);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    size_t idx;
    for (idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(srcImg, contours, idx, colors[idx % 3]);
    }
    
    //  drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    // imshow("TESSTING",srcImg);
    waitKey(0);
    vector<RotatedRect> minEllipse( contours.size() );
    int biggest_size = 0;
    int biggest = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
        if( contours[i].size() > biggest_size )
        { 
            biggest_size = contours[i].size();
            biggest = i;
            
        }
    }
    if(biggest_size > 50 && biggest_size < 180) {
        
        
        
        
        /* for( int i = 0; i < contours.size(); i++ )
         *            {
         *                cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
         *                if( contours[i].size() > 5 )
         *                    { 
         *                        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         *                        
    }
    }*/
        RotatedRect minRect =  minAreaRect( Mat(contours[biggest]));
        Point2f rect_points[4]; minRect.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( srcImg, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,0,0), 1, 8 );
        // ellipse( srcImg, fitEllipse( Mat(contours[biggest]) ), CV_RGB(255,0,0), 2, 8 );
            /// Draw contours + rotated rects + ellipses
            // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            /* for( int i = 0; i< contours.size(); i++ )
             *            {
             *                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             *                // contour
             *            //  drawContours( bw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Pointf() );
             *                // ellipse
             *                ellipse( imgThreshed, minEllipse[i], color, 2, 8 );
    }*/
            imshow("TESTING",srcImg);
            waitKey(0);
            pictureCoordinatesToGlobalPosition(minRect.center.x-100,minRect.center.y+100,minRect.center.x+100,minRect.center.y+100,&x,&y,NULL);
            return true;
    }
    return false;
    
}


bool C23_Detector::detectTable(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    ROS_INFO("Detecting a Standpipe..");
    RNG rng(12345);
    pcl::PointCloud<pcl::PointXYZ>detectionCloud;
    pcl::fromROSMsg<pcl::PointXYZ>(*cloud,detectionCloud);
    pcl::io::savePCDFileASCII ("test_pcd.pcd", detectionCloud);
    
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(12, 180, 200), Scalar(18, 192, 220),imgThreshed);
    //namedWindow("TESTING");
    imshow("TESTING",imgThreshed);
    waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat imgDilated;
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                         Size( 2*18 + 1, 2*18+1 ),
                                         cv::Point( 18, 18 ) );
    /// Apply the dilation operation
    dilate( imgThreshed, imgDilated, element );
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgDilated,bw,10,255,CV_THRESH_BINARY);
    imshow("TESTING",bw);
    waitKey(0);
    cv::Scalar colors[3];
    colors[0] = cv::Scalar(120, 120, 0);
    colors[1] = cv::Scalar(120, 255, 0);
    colors[2] = cv::Scalar(0, 100, 255);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    size_t idx;
    for (idx = 0; idx < contours.size(); idx++) {
        cv::drawContours(srcImg, contours, idx, colors[idx % 3]);
    }
    
    //  drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    // imshow("TESSTING",srcImg);
    waitKey(0);
    vector<RotatedRect> minEllipse( contours.size() );
    int biggest_size = 0;
    int biggest = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
        if( contours[i].size() > biggest_size )
        { 
            biggest_size = contours[i].size();
            biggest = i;
            
        }
    }
    if(biggest_size > 50 && biggest_size < 180) {
        
        
        
        
        /* for( int i = 0; i < contours.size(); i++ )
         *            {
         *                cout << "Contour: " << i << ", Size: " << contours[i].size() << endl;
         *                if( contours[i].size() > 5 )
         *                    { 
         *                        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
         *                        
    }
    }*/
        RotatedRect minRect =  minAreaRect( Mat(contours[biggest]));
        Point2f rect_points[4]; minRect.points( rect_points );
        for( int j = 0; j < 4; j++ )
            line( srcImg, rect_points[j], rect_points[(j+1)%4], CV_RGB(255,0,0), 1, 8 );
        // ellipse( srcImg, fitEllipse( Mat(contours[biggest]) ), CV_RGB(255,0,0), 2, 8 );
            /// Draw contours + rotated rects + ellipses
            // Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
            /* for( int i = 0; i< contours.size(); i++ )
             *            {
             *                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
             *                // contour
             *            //  drawContours( bw, contours, i, color, 1, 8, vector<Vec4i>(), 0, Pointf() );
             *                // ellipse
             *                ellipse( imgThreshed, minEllipse[i], color, 2, 8 );
    }*/
            imshow("TESTING",srcImg);
            waitKey(0);
            pictureCoordinatesToGlobalPosition(minRect.center.x-100,minRect.center.y+100,minRect.center.x+100,minRect.center.y+100,&x,&y,NULL);
            return true;
    }
    return false;
    
}


bool C23_Detector::detectCar(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    
    //------------------Move Robots Neck---------------------------------------
    /* PoseController::delta_neck_movement neck;
     * double total_time = 1.0;
     * int segments = 50;
     * //ROS_INFO("Got back_lbz %f", positions[joints["back_lbz"]]);
     * double velocity = (direction - positions[joints["back_lbz"]])/total_time;
     * std_srvs::Empty e;
     * start_posecontroller_cli_.call(e);
     * //ROS_INFO("velocity %f", velocity);
     * for(int i = 0; i < segments; i++){
     * ros::spinOnce();
     * neck.request.back_lbz = velocity/segments;
     * //ROS_INFO("velocity #%d: %f", i+1, back.request.back_lbz);
     * //ROS_INFO("#%d joint: %f", i+1, positions[joints["back_lbz"]]);
     * //ROS_INFO("#%d direction: %f", i+1, direction);
     * if(!backz_cli.call(neck)){
     * ROS_ERROR("%s: aborted", action_name_.c_str());
     * // set the action state to preempted
     * as_.setAborted();
     * break;
     }
     ros::Duration(total_time/segments).sleep();
     }
     stop_posecontroller_cli_.call(e);*/
    //----------------------------------------------------------
    
    //Define the service client to be accessed
    C21_VisionAndLidar::C21_obj c21srv;
    
    pcl::PointCloud<pcl::PointXYZ>pclcloud;
    pcl::fromROSMsg<pcl::PointXYZ>(*cloud,pclcloud);
    
    _generalDetector.detect(srcImg);
    cout << "Done detection" << endl;
    // 215,365,182,114
    /*_generalDetector._x = 215;
     * _generalDetector._y = 365;
     * _generalDetector._width = 182;
     * _generalDetector._height = 114;
     */
    
    /*Rect carRect(_generalDetector._x, _generalDetector._y, _generalDetector._width, _generalDetector._height);
     * Mat carImage(srcImg, carRect);
     * imshow("Car patch", carImage);
     * waitKey(0);*/
    
    Point2f minImagePoint;
    minImagePoint.x = -1;
    minImagePoint.y = -1;
    if(_generalDetector._x != -1) {
        x = _generalDetector._x;
        y = _generalDetector._y;
        width = _generalDetector._width;
        height = _generalDetector._height;
        cout << "Calculating point cloud" << endl;
        int min = 100000;
        pcl::PointXYZ minPoint, absolutePoint;
        
        
        
        //Add a transformation matrix to convert the camera coordinates to absolute coordinates
        //TO DO....
        geometry_msgs::Pose pose;
        
        //Get the closest point from the robot to the car
        cout << x << "," << y << "," << width << "," << height << endl;
        for(int i = x; i < x + width; i++) {
            for(int j = y; j < y + height; j++) {
                //     cout << "(" << i << "," << j << "," << pclcloud.at(i,j).z << ")" << endl;
                if(sqrt(pclcloud.at(i,j).x*pclcloud.at(i,j).x+pclcloud.at(i,j).y*pclcloud.at(i,j).y+pclcloud.at(i,j).z*pclcloud.at(i,j).z*10000) < min) {
                    min = sqrt(pclcloud.at(i,j).x*pclcloud.at(i,j).x+pclcloud.at(i,j).y*pclcloud.at(i,j).y+pclcloud.at(i,j).z*pclcloud.at(i,j).z*10000);
                    minPoint = pcl::PointXYZ(pclcloud.at(i,j).x,pclcloud.at(i,j).y,pclcloud.at(i,j).z);//Check
                    minImagePoint.x = i;
                    minImagePoint.y = j;
                }
            }
        }
        
        
        //Expand a bounding box to fit the car
        int x1,y1;
        int x0,y0;
        int flag = 0;
        float depth = 0;
        double THRESHOLD = 2;
        int nanColumnCounter = 0;
        bool nanColumnFlag = true;
        cout<<"Max depth: "<<minPoint.x<<endl;
        for(int i = 1; i < 500 && i + minImagePoint.x < srcImg.cols; i +=10) {
            flag = 0;
            // cout << "Checking column in right direction: " << i << endl;
            nanColumnFlag = true;
            for(int j= 150; j >= -150 && (minImagePoint.y -j >0 && minImagePoint.y -j <srcImg.rows); j--) {
                depth = pclcloud.at(minImagePoint.x+i,minImagePoint.y-j).x;
                
                //cout<<"Depth: "<<depth<<endl;
                
                
                if (depth!=depth)//If all values are nan then it will stop
{
    //ROS_INFO("Nan values obtained. Cannot determine the distance to the car");
    continue;
}

nanColumnFlag = false;

if(depth < minPoint.x+THRESHOLD && depth > minPoint.x-THRESHOLD) {
    
    x1 = minImagePoint.x+i;
    //cout<<"X1: "<<x1<<endl;
    flag = 1;
}
            }
            if(nanColumnFlag) nanColumnCounter++;
            
            
            if(!flag && nanColumnCounter>50) {
                break;
            }
            
        }
        cout<<"X1: "<<x1<<", Xmin: "<<minImagePoint.x<<endl;
        //Draw the right boundary column of the car
        //circle( srcImg, Point2f(x1,minImagePoint.y),10, 200, -1, 8, 0 );
        
        nanColumnCounter = 0;
        nanColumnFlag = true;
        depth = 0;
        for(int i = 0; i < 500 && minImagePoint.x-i >0; i +=10) {
            flag = 0;
            //cout << "Checking column in left direction: " << i << endl;
            nanColumnFlag = true;
            for(int j= 150; j >= -150 && (minImagePoint.y -j >0 && minImagePoint.y -j <srcImg.rows); j--) {
                depth = pclcloud.at(minImagePoint.x-i,minImagePoint.y-j).x;
                
                if (depth!=depth)
                {
                    //ROS_INFO("Nan values obtained. Cannot determine the distance to the car");
                    continue;
                }
                
                nanColumnFlag = false;
                
                
                if(depth < minPoint.x+THRESHOLD && depth > minPoint.x-THRESHOLD) {
                    x0 = minImagePoint.x-i;
                    flag = 1;
                }
            }
            if(nanColumnFlag) nanColumnCounter++;
            
            
            if(!flag && nanColumnCounter>50) {
                break;
            }
            
        }
        cout << "X0: " << x0 << " X1: " << x1 << endl;
        //The left boundary column of the car
        //circle( srcImg, Point2f(x0,minImagePoint.y),10, 300, -1, 8, 0 );
        
        //Display the sub-region of the car found
        int carHeight = x1  - x0; //Asssume height and width are approximately the same
        
        y0 = minImagePoint.y - carHeight/2;
        y1 = minImagePoint.y + carHeight/2;
        
        
        int srcImgWidth = srcImg.cols;
        int srcImgHeight = srcImg.rows;
        
        //Make sure the points are within the image bounds
        if(y1 > srcImgHeight)
            y1 = srcImgHeight-1;
        if(y0 < 0)
            y0 = 1;
        if (x0<0)
            x0 = 1;
        if(x1>srcImgWidth)
            x1 = srcImgWidth-1;
        
        
        cout<<"The image dimensions are "<<srcImgWidth<<", "<<srcImgHeight<<endl;
        
        std::vector<cv::Point> bounds;
        line( srcImg, cv::Point(x0, y0), cv::Point(x0, y1), Scalar(0,0,255), 3, CV_AA);
        line( srcImg, cv::Point(x0, y1), cv::Point(x1, y1), Scalar(0,0,255), 3, CV_AA);
        line( srcImg, cv::Point(x0, y0), cv::Point(x1, y0), Scalar(0,0,255), 3, CV_AA);
        line( srcImg, cv::Point(x1, y0), cv::Point(x1, y1), Scalar(0,0,255), 3, CV_AA);
        
        //Get the absolute coordinates of the closest point on the car
        //--------------------------------------------------------------
        
        //Heuristic to contain the car frame
        int yTop =  minImagePoint.y - carHeight/3;
        if(yTop<0)
            yTop  =1;
        
        //Use the service to get the new coordinates
            c21srv.request.sample.x1 = x0;
            c21srv.request.sample.y1 = yTop;
            c21srv.request.sample.x2 = x1;
            c21srv.request.sample.y2 = y1;
            
            cout << "X1: " << x0 << " X2: " << x1 <<" Y1: "<<yTop<<" Y2:"<< y1<< endl;
            
            
            pictureCoordinatesToGlobalPosition(x0, yTop, x1, y1, &x, &y, NULL);
            
            /* if(c21client.call(c21srv)){
             * ROS_INFO("Service initiated");
             * ROS_INFO("Before...\n");
             * absolutePoint.x = (float)c21srv.response.point.x;
             * absolutePoint.y = (float)c21srv.response.point.y;
             * absolutePoint.z = (float)c21srv.response.point.z;
             * ROS_INFO("The points after are %f, %f, %f\n",absolutePoint.x , absolutePoint.y , absolutePoint.z);
            }
            else{
                ROS_INFO("HERE");
                return false;
            }
            
            //The absolute coordinates of the center of the car
            cout << "Real World Points: "<<absolutePoint.x << "," << absolutePoint.y << "," << absolutePoint.z << endl;   
            
            //Check if the points are valid
            if (absolutePoint.x!=absolutePoint.x && absolutePoint.y!=absolutePoint.y&&absolutePoint.z!=absolutePoint.z)
            {
                ROS_INFO("Nan values obtained. Cannot determine the distance to the car");
                //return false; //*****This should be uncommented
            }
            
            
            //Draw the closest point to the car
            cout << "Best point: " << sqrt(absolutePoint.x*absolutePoint.x+absolutePoint.y*absolutePoint.y+absolutePoint.z*absolutePoint.z) << endl;
            
            */
            
            
            //The closest point from the robot to the car
            //circle( srcImg, Point2f(minImagePoint.x,minImagePoint.y), 5, 60, -1, 8, 0 ); 
            //The average
            //x = absolutePoint.x;
            //y = absolutePoint.y;
            
            //imshow("Testing",srcImg);
            //waitKey(0);
            
            //Determine if the robot is facing the driver or passenger side
            detectPassengerDriver(srcImg, x0, yTop, x1,  y1, minPoint, pclcloud);
            
            
            
            return true;
    }
    x = -1;
    return false;
    
}



// comparison function object
bool compareContourAreas ( vector<cv::Point> contour1, vector<cv::Point> contour2 ) {
    
    double i =  fabs(contourArea(contour1, false));
    double j =  fabs(contourArea(contour2,false));
    cout<<"Area1: "<<i<<" Area2: "<<j<<endl;
    
    return ( i < j );
}

bool C23_Detector::detectPassengerDriver(Mat srcImg, int x1,int y1,int x2,int y2, pcl::PointXYZ minPoint, pcl::PointCloud<pcl::PointXYZ> pclcloud){
    
    //To determine where the driver/passenger sides are using blob detection set to true.
    bool detectBlobs = true;
    
    //Extract the segment of the car that has been detected
    ROS_INFO("Extracting the car");
    cout<<"X1: "<<x1<<" Y1: "<<y1<<"X2: "<<x2<<" Y2: "<<y2<<endl;
    Rect carRect(x1, y1, x2-x1, y2-y1);
    Mat carImage(srcImg, carRect);
    imshow("Car", carImage);
    waitKey(0);
    
    //Covert the image to HSV colour space
    Mat imgHsvCar, imgThresholdedCar, imgCarOpened, imgCarDilated;
    cvtColor(carImage, imgHsvCar, CV_BGR2HSV);
    //imshow("HSV", imgHsvCar);
    //waitKey(0);
    
    
    if(detectBlobs)
    {
        
        //Detect the blue blobs on the car
        inRange(imgHsvCar, Scalar(100, 100, 30), Scalar(120, 255, 150), imgThresholdedCar);
        
        //Open the image to remove noise
        Mat element3(3,3,CV_8U, Scalar(1));
        
        morphologyEx(imgThresholdedCar, imgCarOpened, MORPH_OPEN,element3); 
        
        // Apply the dilation operation
        dilate( imgCarOpened, imgCarDilated, element3 );
        
        //Now find the contours of the blobs
        vector<vector<cv::Point> > blobContours;
        findContours(imgCarDilated,blobContours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        drawContours(carImage,blobContours,-1,CV_RGB(255,0,0),2); 
        
        //imshow("Thresholded Image", imgThresholdedCar);
        //waitKey(0);
        
        //imshow("Opened Image", imgCarOpened);
        //waitKey(0);
        
        //imshow("Dilated Image", imgCarDilated);
        //waitKey(0);
        
        //imshow("Car Image", carImage);
        //waitKey(0);
        
        
        cout<<"Number of contours found "<<blobContours.size()<<endl;
        if(blobContours.size()>1)
        {  
            //Sort the contours based on their area
            sort(blobContours.begin(), blobContours.end(), compareContourAreas);
            
            //Find the area of the contours
            double maxArea = -1;
            int maxContourIndex = -1;
            double currentArea = -1;
            vector<double> blobAreas;
            vector<int> blobIndices;
            for(int ii=0; ii<blobContours.size(); ii++)
            {
                blobAreas.push_back(contourArea(blobContours[ii], false));
                blobIndices.push_back(ii);
                currentArea = contourArea(blobContours[ii], false);
                
            }
            
            //Find the center of mass of the contours
            // Get the moments
            vector<Moments> mu(blobContours.size());
            for( int ii = 0; ii < blobContours.size(); ii++ )
            { mu[ii] = moments( blobContours[ii], false ); }
            
            //  Get the mass centers:
            vector<Point2f> mc( blobContours.size() );
            for( int ii = 0; ii < blobContours.size() ; ii++ )
            { mc[ii] = Point2f( mu[ii].m10/mu[ii].m00 , mu[ii].m01/mu[ii].m00 ); 
            cout<<"CM: "<<mc[ii]<<", Area: "<<blobAreas[ii]<<endl;
            }
            
            //Determine the largest and second largest contour
            cout<<"Largest: "<<mc[mc.size()-1].x<<", Second Largest: "<<mc[mc.size()-2].x<<endl;
            if(mc[mc.size()-1].x > mc[mc.size()-2].x){
                ROS_INFO("Robot is facing passenger side"); 
            }else{
                ROS_INFO("Robot is facing driver side"); 
            }
            
        }else{
            
            ROS_INFO("Not enough contours found");
            
        }
        
    }else
    {
        //----------------------------Method 1-----------------------------------
        /*
         * //Try template matching in order to determine where the passenger side is
         * Mat bootImageTemplate = imread("template_matching_images/boot_template_right.jpg",1);
         * //imshow("Boot", bootImageTemplate);
         * //waitKey(0);
         * cout<<"Size is "<<bootImageTemplate.rows<<", "<<bootImageTemplate.cols<<endl;
         * templateMatching( carImage, bootImageTemplate, 1 );
         */
        
        
        //This is an alternative algorithm to determining whether the robot is facing the passenger or driver
        
        //--------------------Method 2----------------------------------
        /*
         * //Find the car based on a thresholding approach
         * //Mat destImage(carImage.size(), CV_8U);
         * Point2f point;
         * 
         * inRange(imgHsvCar, Scalar(0, 0, 0), Scalar(20, 255, 255), imgThresholdedCar);
         * 
         * //cout<<"The image type is "<<destImage.type()<<endl;
         * //waitKey(0);
         * float depth = 0;
         * cout<<"The min depth is: "<<minPoint.x<<endl;
         * int xpos = 0;
         * int ypos = 0;
         * 
         * cv::Scalar intensity;
         * 
         * cout<<"Variables: "<<x1<<", "<<x2<<", "<<y1<<", "<<y2<<endl;
         * 
         * for(int ii=x1; ii<x2 && xpos<imgThresholdedCar.cols; ii++){
         * ypos = 0;
         * for(int jj=y1; jj<y2 && ypos<imgThresholdedCar.rows;jj++){
         * 
         * depth = pclcloud.at(ii,jj).x;
         * //intensity = imgThresholdedCar.at<uchar>(xpos, ypos);
         * //intensity.val[0]  =255;
         * //cout<<"Xpos, yPos: "<<xpos<<", "<<ypos<<endl;
         * //cout<<"The pixel value: "<<intensity.val[0]<<endl;
         * //point.x  =xpos;
         * //point.y = ypos;
         * //cout<<"Depth: "<<depth<<" at X,Y "<<ii<<", "<<jj<<endl;
         * cout<<"Depth: "<<depth<< "Min: "<<minPoint.x<<endl;
         * if (depth < minPoint.x+2 && depth > minPoint.x-2){
         * imgThresholdedCar.at<uchar>(ypos, xpos) = 255;
         * intensity = imgThresholdedCar.at<uchar>(ypos, xpos);
         * cout<<"Setting to white:   "<<intensity.val[0]<<endl;
         * 
         }
         else{
             imgThresholdedCar.at<uchar>(ypos, xpos) = 0;
             intensity = imgThresholdedCar.at<uchar>(ypos, xpos);
             //cout<<"Setting to black:   "<<intensity.val[0]<<endl;
         }
         
         
         
         
         ypos++;
         }
         xpos++;
         }
         //_detectionMutex->unlock();
         imshow("car image", imgThresholdedCar);
         waitKey(0);
         
         
         
         Mat dst, cdst;
         
         inRange(imgHsvCar, Scalar(0, 0, 0), Scalar(20, 255, 255), imgThresholdedCar);
         
         Canny(imgThresholdedCar, dst, 50, 300, 3);
         
         imshow("After Canny", dst);
         waitKey();
         cvtColor(dst, cdst, CV_GRAY2BGR);
         
         vector<Vec4i> lines;
         HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
         for( size_t i = 0; i < lines.size(); i++ )
         {
             Vec4i l = lines[i];
             line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
         }
         
         imshow("source", carImage);
         waitKey();
         imshow("detected lines", cdst);
         waitKey();
         
         //-------------------------------------------------------
         */
        
        
        //-------------------Method 3---------------------------------------
        /*
         * //Detect the car
         * inRange(imgHsvCar, Scalar(0, 0, 0), Scalar(20, 255, 255), imgThresholdedCar);
         * 
         * imshow("Thresholded Image Routine 2", imgThresholdedCar);
         * waitKey(0);
         * 
         * //Open the image to remove noise
         * Mat element3(5,5,CV_8U, Scalar(1));
         * morphologyEx(imgThresholdedCar, imgCarOpened, MORPH_OPEN,element3);  
         * 
         * imshow("Opened Image Routine 2", imgCarOpened);
         * waitKey(0);
         * 
         * //Now find the contours of the blobs
         * vector<vector<cv::Point> > blobContours;
         * findContours(imgCarOpened,blobContours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
         * drawContours(carImage,blobContours,-1,CV_RGB(255,0,0),2);
         * 
         * imshow("Contours Routine 2", carImage);
         * waitKey(0);
         */
        
    }
    
    return true;
}



bool C23_Detector::detectPath(Mat srcImg) {
    // IplImage* img = new IplImage(srcImg);
    Mat imgHSV, imgThreshed;
    cvtColor(srcImg,imgHSV,CV_BGR2HSV);
    inRange(imgHSV,Scalar(20,30,30),Scalar(40,255,255),imgThreshed);
    //  namedWindow("TESTING");
    // imshow("TESTING",imgThreshed);
    //waitKey(0);
    // imwrite("test12.jpg",imgThreshed);
    Mat bw;
    vector<vector<cv::Point> > contours;
    threshold(imgThreshed,bw,10,255,CV_THRESH_BINARY);
    findContours(bw,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
    drawContours(srcImg,contours,-1,CV_RGB(255,0,0),2);
    imshow("TESSTING",srcImg);
    //waitKey(0);
    Mat dst,cdst;
    Canny(bw, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    
    
    //Probabilistic Hough Line Transform
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 5, CV_PI/180, 50, 100, 10 );
    int max = 0;
    Vec4i maxVec;
    if(lines.size() == 0) {
        return false;
    }
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        cout << l[0] << "," << l[1] << "---" << l[2]  << "," << l[3] << endl;
        if(l[1] > max || l[3] > max) {
            max = l[1] > l[3] ? l[1] : l[3];
            maxVec = l;
        }
        // line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
    }
    line( cdst, cv::Point(maxVec[0], maxVec[1]), cv::Point(maxVec[2], maxVec[3]), Scalar(0,0,255), 3, CV_AA);
    imshow("TESTING",cdst);
    waitKey(0);
    if(maxVec[1] < maxVec[3]) {
        x = maxVec[0];
        y = maxVec[1];
    } else {
        x = maxVec[2];
        y = maxVec[3];
    }
    return true;
    
    
}
bool C23_Detector::detectGate(Mat srcImg, const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    
    pcl::PointCloud<pcl::PointXYZ>pclcloud;
    C21_VisionAndLidar::C21_obj c21srv;
    pcl::fromROSMsg<pcl::PointXYZ>(*cloud,pclcloud);
    static pcl::PointXYZ leftC;
    static pcl::PointXYZ rightC;
    static pcl::PointXYZ centerC;
    bool res = false;
    bool left=false, right=false;
    IplImage* img = new IplImage(srcImg);
    IplImage* imgHSV = cvCreateImage(cvGetSize(img), 8, 3);
    
    cvCvtColor(img, imgHSV, CV_BGR2HSV);
    IplImage* imgThreshed = cvCreateImage(cvGetSize(img), 8, 1);
    cvInRangeS(imgHSV, cvScalar(0, 30, 20), cvScalar(5, 255, 255), imgThreshed); //filter red
    
    Mat threshMat(imgThreshed);
    
   // imshow("Threshed red",threshMat);
   // waitKey(0);
    //  ROS_INFO("Gate!");
    
    
    //contoursR
    Mat bw,gray;
    
    //  ROS_INFO("1");
    ////findng center mass of right column
    threshold(threshMat, bw, 10, 255, CV_THRESH_BINARY);
    
    vector<vector<cv::Point> > contoursR;
    findContours(bw, contoursR, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    cout<<"The Contour right size: "<< contoursR.size()<<endl;
    vector<Moments> muR(contoursR.size() );
    vector<Point2f> mcR(contoursR.size() );
    int biggstR=0;
    if(contoursR.size() != 0) {
        drawContours(srcImg, contoursR, -1, CV_RGB(255,0,0), 2);
        
        /// Get the moments and mass centers:
       //  imshow("Contours", srcImg);
       //   waitKey(0);
        
        
        for(unsigned int i = 0; i < contoursR.size(); i++ )
        {
            muR[i] = moments( contoursR[i], false );
            mcR[i] = Point2f( muR[i].m10/muR[i].m00 , muR[i].m01/muR[i].m00 );
            if (muR[biggstR].m00<=muR[i].m00)
                biggstR=i;
            
        }
        //Draw a circle indicating the center of mass on the right pole
        circle( srcImg, mcR[biggstR], 16, 60, -1, 8, 0 );
        right = true;
        cout << "We found red!" << endl;
        if (pclcloud.at(mcR[biggstR].x,mcR[biggstR].y).x<50 && pclcloud.at(mcR[biggstR].x,mcR[biggstR].y).y <50 && pclcloud.at(mcR[biggstR].x,mcR[biggstR].y).x !=0)
        {
            rightC=pclcloud.at(mcR[biggstR].x,mcR[biggstR].y);
            
        }
    }
    
    
    /////finding center mass of left column
    cvInRangeS(imgHSV, cvScalar(90, 130, 40), cvScalar(150, 255, 250), imgThreshed);//filter blue
    
    
    Mat threshMatL(imgThreshed),bwL;
    
    threshold(threshMatL, bwL, 10, 255, CV_THRESH_BINARY);
    
  //  imshow("Threshed blue",threshMatL);
  //  waitKey(0);
    vector<vector<cv::Point> > contoursL;
    //	  cout<<"next2"<<endl;
    findContours(bwL, contoursL, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    vector<Moments> muL(contoursL.size() );
    vector<Point2f> mcL(contoursL.size() );
    int biggstL=0;
    if(contoursL.size() !=0) {
        drawContours(srcImg, contoursL, -1, CV_RGB(0,0,255), 2);
      //  imshow("Contours", srcImg);
       // waitKey(0);
        
        
        for(unsigned int i = 0; i < contoursL.size(); i++ )
        {
            muL[i] = moments( contoursL[i], false );
            mcL[i] = Point2f( muL[i].m10/muL[i].m00 , muL[i].m01/muL[i].m00 );
            if (muL[biggstL].m00<=muL[i].m00)
                biggstL=i;
            
        }
        left = true;
        cout << "Left is true! " << endl;
        circle( srcImg, mcL[biggstL], 16, Scalar(0,0,255), -1, 8, 0 );
        if (pclcloud.at(mcL[biggstL].x,mcL[biggstL].y).x<50 && pclcloud.at(mcL[biggstL].x,mcL[biggstL].y).y <50 && pclcloud.at(mcL[biggstL].x,mcL[biggstL].y).x !=0)
        {
            leftC=pclcloud.at(mcL[biggstL].x,mcL[biggstL].y);
        }
        
    }
    
    if(left && right) {
        
        
        line( srcImg,mcL[biggstL],mcR[biggstR],Scalar(0,0,0),5,8);
        
        //	cout <<"\nleft :"<<leftC.x<<" ,"<<leftC.y<<endl;
        //	cout<<"\nright :"<<rightC.x<<" ,"<<rightC.y<<endl;
        
        double gate =sqrt(pow((leftC.x-rightC.x),2)+pow((leftC.y-rightC.y),2));
        //cout<<gate<<endl;
        res = false;
        if (gate > 4.5 && gate < 5.8)
        {
            res = true;
            
        }
        cout << "Res is: " << res << endl;
        if(!res) {
            left = leftC.z < rightC.z;
            right = leftC.z > rightC.z;
            
        } else {
            //find lines
            Mat dst,cdst;
            // ROS_INFO("4.4");
            Canny(threshMat, dst, 50, 200, 3);
            // ROS_INFO("4.5");
            cvtColor(dst, cdst, CV_GRAY2BGR);
            //  ROS_INFO("5");
            //Probabilistic Hough Line Transform
            vector<Vec4i> lines;
            HoughLinesP(dst, lines, 5, CV_PI/180, 50, 100, 10 );
            for( size_t i = 0; i < lines.size(); i++ )
            {
                Vec4i l = lines[i];
                line( cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
            }
            //   ROS_INFO("6");
            
            //cout<<"left"<<mcL[biggstL]<<"right"<<mcR[biggstR]<<endl;
            //These points are the central coordinates of the gate
            
            
            //   x = (mcL[biggstL].x + mcR[biggstR].x)/2;
            //    y = (mcL[biggstL].y + mcR[biggstR].y)/2;
          /*  int x1,y1,x2,y2;
            cout << "Sending left: " << mcL[biggstL].x << "," << mcL[biggstL].y << endl;
            cout << "Sending right: " << mcR[biggstR].x << "," <<mcR[biggstR].y << endl;
            
            c21srv.request.sample.x1 = MIN(mcL[biggstL].x, mcR[biggstR].x);
            c21srv.request.sample.y1 = MIN(mcL[biggstL].y, mcR[biggstR].y);
            c21srv.request.sample.x2 = MAX(mcL[biggstL].x, mcR[biggstR].x);
            c21srv.request.sample.y2 = MAX(mcL[biggstL].y, mcR[biggstR].y);
            
            if(c21client.call(c21srv)){
                cout << "left..." << endl;
                x= (float)c21srv.response.point.x;
                y = (float)c21srv.response.point.y;
                cout << "Got data: " << x << "," << y << endl;
                
            }*/
          
            double x1,y1,x2,y2;
            cout << "Sending left: " << mcL[biggstL].x << "," << mcL[biggstL].y << endl;
            cout << "Sending right: " << mcR[biggstR].x << "," <<mcR[biggstR].y << endl;
            
            c21srv.request.sample.x1 = mcL[biggstL].x-10;
            c21srv.request.sample.y1 = mcL[biggstL].y-100;;
            c21srv.request.sample.x2 = mcL[biggstL].x+10;
            c21srv.request.sample.y2 = mcL[biggstL].y+100;;
            
            if(c21client.call(c21srv)){
                cout << "left..." << endl;
                x1= (float)c21srv.response.point.x;
                y1 = (float)c21srv.response.point.y;
                cout << "Got data: " << x << "," << y << endl;
                
            }
            
            c21srv.request.sample.x1 = mcR[biggstR].x-10;
            c21srv.request.sample.y1 = mcR[biggstR].y-100;;
            c21srv.request.sample.x2 = mcR[biggstR].x+10;
            c21srv.request.sample.y2 = mcR[biggstR].y+100;;
            
            if(c21client.call(c21srv)){
                cout << "left..." << endl;
                x2= (float)c21srv.response.point.x;
                y2 = (float)c21srv.response.point.y;
                cout << "Got data: " << x << "," << y << endl;
                
            }
             x =  (x1+x2)/2.0;
             y = (y1+y2)/2.0;
          // 
           
         //   return true;
          //  /
          Point2f a((float) (mcL[biggstL].x + mcR[biggstR].x)/2,(float)(mcL[biggstL].y + mcR[biggstR].y)/2);
          circle( srcImg, a, 16, Scalar(0,0,255), -1, 8, 0 );
          imshow("TESTING",srcImg);
          waitKey(0);
          return true;
          /*
          double x1,y1,z1,x2,y2,z2;
          averagePointCloud(mcL[biggstL].x-10, mcL[biggstL].y-100, mcL[biggstL].x+10, mcL[biggstL].y+100, cloud, &x1, &y1,&z1);
          averagePointCloud(mcR[biggstR].x-10, mcR[biggstR].y-100, mcR[biggstR].x+10, mcR[biggstR].y+100, cloud, &x2, &y2,&z2);
          return true;
         // x = (x1+x2)/2.0;
        //  y = (y1+y2)/2.0;
          */
        }
    }
    
    cvInRangeS(imgHSV, cvScalar(60,30,30),cvScalar(80,255,255), imgThreshed);//filter blue
    
    
    Mat threshMatM(imgThreshed),bwM;
    threshold(threshMatM, bwM, 10, 255, CV_THRESH_BINARY);
    vector<vector<cv::Point> > contoursM;
    //	  cout<<"next2"<<endl;
    findContours(bwM, contoursM, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    vector<Moments> muM(contoursM.size() );
    vector<Point2f> mcM(contoursM.size() );
    int biggstM=0;
    
    if(contoursM.size() !=0) {
        drawContours(srcImg, contoursM, -1, CV_RGB(0,0,255), 2);
        //imshow("TESTING",srcImg);
        //waitKey(0);
        
        
        for(unsigned int i = 0; i < contoursM.size(); i++ )
        {
            muM[i] = moments( contoursM[i], false );
            mcM[i] = Point2f( muM[i].m10/muM[i].m00 , muM[i].m01/muM[i].m00 );
            if (muM[biggstM].m00<=muM[i].m00)
                biggstM=i;
            
        }
        
        circle( srcImg, mcM[biggstM], 16, Scalar(0,0,255), -1, 8, 0 );
        if (pclcloud.at(mcM[biggstM].x,mcM[biggstM].y).x<50 && pclcloud.at(mcM[biggstM].x,mcM[biggstM].y).y <50 && pclcloud.at(mcM[biggstM].x,mcM[biggstM].y).x !=0)
        {
            //set the center of mass of the green gate
            centerC=pclcloud.at(mcM[biggstM].x,mcM[biggstM].y);
        }
        
        
        //Add the check to make sure that the upper green part of the same gate is detected
        double gateUpperDistance =0;
        double contourOffset = 0;
        
        if(right) {
            //Find the distance between the center of mass of green and center of mass of red gate
            gateUpperDistance =sqrt(pow((centerC.x-rightC.x),2)+pow((centerC.y-rightC.y),2));
            
            //If the distance is half the length of the gate pole, then they are part of the same gate
            if(gateUpperDistance > 2.25 && gateUpperDistance < 2.9){      
                x = mcR[biggstR].x - (mcR[biggstR].y - mcM[biggstM].y);
                y =  mcR[biggstR].y;
            }
            else{
                //Find the smallest value in the corresponding contour to set the correct offset for the center of the gate
                double minContourOffsetR = mcR[biggstR].y;
                for(unsigned int i=0; i<contoursR[biggstR].size();i++){
                    
                    if(contoursR[biggstR][i].y < minContourOffsetR)
                    {
                        minContourOffsetR =  contoursR[biggstR][i].y;
                    }
                    
                }
                //Add the contour offset to get the correct coordinate in the image
                int x_pic = mcR[biggstR].x - (mcR[biggstR].y - minContourOffsetR);
                int y_pic =  mcR[biggstR].y;
                
                c21srv.request.sample.x1 = mcR[biggstR].x-10;
                c21srv.request.sample.y1 = mcR[biggstR].y-100;
                c21srv.request.sample.x2 = mcR[biggstR].x+10;
                c21srv.request.sample.y2 = mcR[biggstR].y+100;
                
                if(c21client.call(c21srv)){
                    ROS_INFO("Service initiated");
                    // minPoint.x = c21srv.response.x;
                    //minPoint.y = c21srv.response.y;
                    //minPoint.z = c21srv.response.z;
                    //absolutePoint = (pcl::PointXYZ)c21srv.response.point;
                    ROS_INFO("Before...\n");
                    x = (float)c21srv.response.point.x-3;
                    y = (float)c21srv.response.point.y;
                    
                    
                }
                cout<<"Point is: " <<x<<", " <<y <<endl;
                cout << "Detected right" << endl;
                circle( srcImg, Point2f(x_pic,y_pic), 16, Scalar(0,255,255), -1, 8, 0 );
            }
            
            //      imshow("Testing" , srcImg);
            //   waitKey(0);
            return mcM[biggstM].x < mcR[biggstR].x ? true : false;
            
        } else {
            //Find the distance between the cm of green and cm of blue gate
            gateUpperDistance =sqrt(pow((centerC.x-leftC.x),2)+pow((centerC.y-leftC.y),2));   
            
            if(gateUpperDistance > 2.25 && gateUpperDistance < 2.9){
                x = mcL[biggstL].x + (mcL[biggstL].y - mcM[biggstM].y);
                y =  mcL[biggstL].y;
            }else{
                //Find the smallest value in the corresponding contour
                double minContourOffsetL = mcL[biggstL].y;
                for(unsigned int i=0; i<contoursL[biggstL].size();i++){
                    
                    if(contoursL[biggstL][i].y < minContourOffsetL)
                    {
                        minContourOffsetL =  contoursL[biggstL][i].y;
                    }
                    
                }
                int x_pic = mcL[biggstL].x + (mcL[biggstL].y - minContourOffsetL);
                int y_pic =  mcL[biggstL].y;
                cout<<"Point is: " <<x<<", " <<y <<endl;
            }
            cout << "Detected left" << endl;
            circle( srcImg, Point2f(x,y), 16, Scalar(0,255,255), -1, 8, 0 );
            //   imshow("Testing" , srcImg);
            //   waitKey(0);
            c21srv.request.sample.x1 = mcL[biggstL].x-10;
            c21srv.request.sample.y1 = mcL[biggstL].y-100;
            c21srv.request.sample.x2 = mcL[biggstL].x+10;
            c21srv.request.sample.y2 = mcL[biggstL].y+100;
            
            if(c21client.call(c21srv)){
                ROS_INFO("Service initiated");
                // minPoint.x = c21srv.response.x;
                //minPoint.y = c21srv.response.y;
                //minPoint.z = c21srv.response.z;
                //absolutePoint = (pcl::PointXYZ)c21srv.response.point;
                ROS_INFO("Before...\n");
                x = (float)c21srv.response.point.x+3;
                y = (float)c21srv.response.point.y;
                
                
            }
            return mcM[biggstM].x > mcL[biggstL].x ? true : false;
            
        }
    }
    return false;
    
    
}





