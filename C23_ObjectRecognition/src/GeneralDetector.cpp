/**************************************************************************************
 * This is a template for the C23_ObjectRecognition module for the robil project
 * The C23_ObjectRecognition module goal is to provide a recognition of the object searched in the given frame
 *
 * There is no actual input/output at this time, this goes according to the current milestone
 * set at the robil managment meeting
 * 
 **************************************************************************************/
#include "GeneralDetector.hpp"
#include <ros/package.h>
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include <string>
#include "ros/ros.h"

using namespace std;
using namespace cv;


GeneralDetector::GeneralDetector() {
  extractor = new SiftDescriptorExtractor();
  matcher = new FlannBasedMatcher;
  
  bowide = new BOWImgDescriptorExtractor(extractor,matcher);
  



}

void GeneralDetector::initialize(const string& object) {


 if(! object.compare("carEntry")) {
  cout << "----------- Loading Data ---------" << endl;
  
  cout << "Loading svm data ..." << endl;
  classes_classifiers["background"].load((ros::package::getPath("C23_ObjectRecognition") +"/training/background_svm.dat").c_str()); 
  classes_classifiers["car_driver"].load((ros::package::getPath("C23_ObjectRecognition") + "/training/car_driver_svm.dat").c_str()); 
  classes_classifiers["car_passenger"].load((ros::package::getPath("C23_ObjectRecognition") +"/training/car_passenger_svm.dat").c_str()); 
  classes_classifiers["car_front"].load((ros::package::getPath("C23_ObjectRecognition") +"/training/car_front_svm.dat").c_str()); 
  classes_classifiers["car_rear"].load((ros::package::getPath("C23_ObjectRecognition") +"/training/car_rear_svm.dat").c_str()); 

  cout << "Done loading svm data." << endl;
  cout << "Test: " << (ros::package::getPath("C23_ObjectRecognition") +"/training/background_svm.dat").c_str() << endl;
  
  cout << "Loading vocabulary ... " << endl;
  cv::FileStorage fs2((ros::package::getPath("C23_ObjectRecognition") +"/training/vocabulary.yml").c_str(), cv::FileStorage::READ);
  fs2["vocabulary"] >> vocabulary;
  fs2.release();
  bowide->setVocabulary(vocabulary);
  cout << "Done loading vocabulary." << endl;
  
  cout << "-------- Done loading data ------" << endl;
  
 } else if (!object.compare("Pedals")) {
  
 
 
 } else if (!object.compare("Wheel")) {
 
 
 } else if (!object.compare("Seat")) {
 
 
 } else if (!object.compare("Bar")) {
 
 
 } else if (!object.compare("HandbrakeBrake")) {
 
 
 } else if (!object.compare("Gate")) {
 
 
 } else if (!object.compare("Hose")) {
 
 
 } else if (!object.compare("Pipe")) {
 
 
 }
  



}

void GeneralDetector::detect(Mat img) {
  int  minScale = -2;//-4
  int  maxScale = -1;//2
  int  minSize = 25;
  int objWidth = 315;
  int objHeight = 198;
  int scanAreaW = img.cols;
  int scanAreaH = img.rows;
  Size *scales = new Size[maxScale - minScale + 1];
  int scaleIndex = 0;
  
  for(int i = minScale; i <= maxScale; i++)
  {
    float scale = pow(1.2, i);
    int w = (int)objWidth * scale;
    int h = (int)objHeight * scale;
    int ssw, ssh;


    if(w < minSize || h < minSize || w > scanAreaW || h > scanAreaH) continue;

    scales[scaleIndex].width = w;
    scales[scaleIndex].height = h;

    scaleIndex++;


  }
  int numScales = scaleIndex;
  cout << "Number of scales: " << scaleIndex << endl;
  int min = 9;
  int max = 0;
  int x_,y_,width,height;
  int count2 = 0;
  int flag=0;
  float best_res=9;
  float best_bg_res = 0;
  float bg_res = 0;
  int number_of_patches = 0;
  Mat response_hist;
  string bestMatch;
  for(scaleIndex = 0; scaleIndex < numScales; scaleIndex++)
  {
    int w = scales[scaleIndex].width;
    int h = scales[scaleIndex].height;

    for(int y = 5; y + h <= scanAreaH -1; y += 30)
    {
      for(int x = 5; x + w <= scanAreaW -1; x += 30)
      {
        //  cout << "Image: " << count2 << endl;
        //  cout << "y,x: " << y << "," << x << "h,w: " << h << "," << w << endl;

        Mat subImg = img(cv::Range(y,y+h), cv::Range(x,x+w));
        // cout << "Done subimage" << endl;
        detector.detect(subImg, keypoints);
        bowide->compute(subImg, keypoints, response_hist);
        //  cout << "Done subimage 2" << endl;
        char buff[1000];
      //  sprintf(buff,"training/majd/image%d.jpg%c",count2,'\0');
       // imwrite(buff,subImg);
        if(response_hist.empty()) continue;
        int i =0;
        for (map<string,CvSVM>::iterator it = classes_classifiers.begin(); it != classes_classifiers.end(); ++it) {
          float res = (*it).second.predict(response_hist,true);
         //      cout << " class: " << (*it).first << ", response: " << res << endl;
          if(i == 0) {
            bg_res = res;
          }
          if((*it).first.compare("background") && res < -0.2 && res> -0.4 && bg_res > 0.55 && res < best_res && bg_res > best_bg_res) {
          

            min = res;
            x_ = x;
            y_ = y;
            width = w;
            height = h;
            best_res = res;
            best_bg_res = bg_res;
            cout << "Car: " << (*it).first << ", res: " << res << ",Bg: " << best_bg_res << endl;
            number_of_patches++;
            bestMatch = (*it).first;
	    
          }
          i++;

        }
        count2++;
        subImg.release();
      }
    }

  }
  if( best_res == 9) {
    cout << "No car found!" << endl;
    car_target = NONE;
    ROS_INFO("No car detected");
    _x = -1;
    return ;
  }
  
	  if(!bestMatch.compare("car_driver"))
	  {
	    car_target = CAR_DRIVER; 
	    ROS_INFO("Car driver detected");
	  }
	  else if(!bestMatch.compare("car_passenger")){
	    car_target = CAR_PASSENGER; 
	    ROS_INFO("Car passenger detected");
	  }
	  else if(!bestMatch.compare("car_front")){
	    car_target = CAR_FRONT; 
	    ROS_INFO("Car front detected");
	  }
	  else if(!bestMatch.compare("car_rear")){
	    car_target = CAR_REAR; 
	    ROS_INFO("Car rear detected");
	  }
  
  
  
  
  Mat subImg = img(cv::Range(y_,y_+height), cv::Range(x_,x_+width));
  Mat output2;
  detector.detect(subImg, keypoints);
  cv::drawKeypoints(subImg, keypoints, output2);
  //imwrite("daniel.jpg",output);
  cv::imwrite("sift_result.jpg", output2);
  cout << "Best match: " << bestMatch << endl;
  _x = x_;
  _y = y_;
  _width = width;
  _height = height;
  imwrite("best.jpg",subImg);
  
  // cout <<"done "<< best_res << endl;
  
}
