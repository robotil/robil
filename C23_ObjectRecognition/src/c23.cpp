#include "C23_Node.hpp"

#include "LearnObjectServer.hpp"
#include "SearchObjectServer.hpp"
#include "TrackObjectServer.hpp"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <dirent.h>
#include <fstream>
#include <stdexcept>

//#define SVM
//#define LEARNING
#define NODE

#ifdef NODE

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c23_objectReconition");
 
  if(argc!=3){
	  printf("usage: C23_module <left camera topic> <right camera topic>");
  }
  
  C23_Node my_node(argv[1],argv[2]);

  TrackObjectServer trackObject(my_node);
  SearchObjectServer searchObject(my_node);
  LearnObjectServer learnObject(my_node);
  ROS_INFO("C23 made topic at %s %s \n",argv[1],argv[2]);
 // SURFDetection surf(argv[1],argv[2]); 
  
 
  ros::spin();

  return 0;
}
#endif
/*


#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


using namespace cv;
static const Size trainingPadding = Size(0, 0);
static const Size winStride = Size(8, 8);
Mat get_hogdescriptor_visu(Mat& origImg, vector<float>& descriptorValues)
{   
    Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);
 
    float zoomFac = 3;
    Mat visu;
    resize(color_origImg, visu, Size(color_origImg.cols*zoomFac, color_origImg.rows*zoomFac));
 
    int blockSize       = 16;
    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?
 
    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = 64 / cellSize;
    int cells_in_y_dir = 128 / cellSize;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;
 
            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }
 
    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
 
    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;
 
    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }
 
                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;
 
                    gradientStrengths[celly][cellx][bin] += gradientStrength;
 
                } // for (all bins)
 
 
                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;
 
            } // for (all cells)
 
 
        } // for (all block x pos)
    } // for (all block y pos)
 
 
    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
 
            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
 
            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }
 
 
    cout << "descriptorDataIdx = " << descriptorDataIdx << endl;
 
    // draw cells
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;
 
            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;
 
            rectangle(visu, Point(drawX*zoomFac,drawY*zoomFac), Point((drawX+cellSize)*zoomFac,(drawY+cellSize)*zoomFac), CV_RGB(100,100,100), 1);
 
            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];
 
                // no line to draw?
                if (currentGradStrength==0)
                    continue;
 
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better
 
                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
 
                // draw gradient visualization
                line(visu, Point(x1*zoomFac,y1*zoomFac), Point(x2*zoomFac,y2*zoomFac), CV_RGB(0,255,0), 1);
 
            } // for (all bins)
 
        } // for (cellx)
    } // for (celly)
 
 
    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
      for (int x=0; x<cells_in_x_dir; x++)
      {
           delete[] gradientStrengths[y][x];            
      }
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;
 
    return visu;
 
} // get_hogdescriptor_visu


    Mat imageData2 = imread(imageFilename, 0);
    Mat imageData; 
    Size s(64,128);
    
    resize(imageData2,imageData, s);
    imwrite("majd.jpg",imageData);
   // cv::resize(imageData2, imageData, dst.size());
    
    if (imageData.empty()) {
        featureVector.clear();
        printf("Error: HOG image '%s' is empty, features calculation skipped!\n", imageFilename.c_str());
        return;
    }
    // Check for mismatching dimensions
    if (imageData.cols != hog.winSize.width || imageData.rows != hog.winSize.height) {
        featureVector.clear();
        printf("Error: Image '%s' dimensions (%u x %u) do not match HOG window size (%u x %u)!\n", imageFilename.c_str(), imageData.cols, imageData.rows, hog.winSize.width, hog.winSize.height);
        return;
    }
    vector<Point> locations;
    hog.compute(imageData, featureVector, winStride, trainingPadding, locations);
    imageData.release(); // Release the image again after features are extracted
}

*/


#ifdef PATCHES
int main() {
    Mat img;
    //544
    int count = 1017;
    for(int k =1; k < 206; k++)  {
      char buff2[1000];
      sprintf(buff2,"training/car_driver/yair/image%d.jpg%c",k,'\0');
      img = imread(buff2);
      if(img.empty() ) continue;
      ROS_INFO(buff2);
      
      for(int j = 100; j < 489-198-1; j+=100) {
        for(int i = 1; i < 921-315-1; i+=250) {
          ROS_INFO(" %d %d ",i,j);
           Rect roi(i, j, 315, 198);

          Mat image_roi = img(roi);
          char buff[1000];
          count++;
          sprintf(buff,"training/car_driver/negative/image%d.jpg%c",count,'\0');
          imwrite(buff,image_roi);
          image_roi.release();
         }
       }
       img.release();
     }
     return 0;
}

#endif
/*

int main(int argc, char* argv[]) {
  HOGDescriptor hog;
 /*
  static vector<string> positiveTrainingImages;
    static vector<string> negativeTrainingImages;
    static vector<string> validExtensions;
    
    validExtensions.push_back("jpg");
    getFilesInDirectory(posSamplesDir, positiveTrainingImages, validExtensions);
    getFilesInDirectory(negSamplesDir, negativeTrainingImages, validExtensions);
    /// Retrieve the descriptor vectors from the samples
    unsigned long overallSamples = positiveTrainingImages.size() + negativeTrainingImages.size();
    // </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="Calculate HOG features and save to file">
    // Make sure there are actually samples to train
    if (overallSamples == 0) {
        printf("No training sample files found, nothing to do!\n");
        return EXIT_SUCCESS;
    }
    
   setlocale(LC_ALL, "C"); // Do not use the system locale
    setlocale(LC_NUMERIC,"C");
    setlocale(LC_ALL, "POSIX");
    
     float percent;
    /*
     fstream File;
    File.open(featuresFile.c_str(), ios::out);
    if (File.good() && File.is_open()) {
        File << "# Use this file to train, e.g. SVMlight by issuing $ svm_learn -i 1 -a weights.txt " << featuresFile.c_str() << endl; // Remove this line for libsvm which does not support comments
        // Iterate over sample images
        for (unsigned long currentFile = 0; currentFile < overallSamples; ++currentFile) {
            storeCursor();
            vector<float> featureVector;
            // Get positive or negative sample image file path
            const string currentImageFile = (currentFile < positiveTrainingImages.size() ? positiveTrainingImages.at(currentFile) : negativeTrainingImages.at(currentFile - positiveTrainingImages.size()));
            // Output progress
            if ( (currentFile+1) % 10 == 0 || (currentFile+1) == overallSamples ) {
                percent = ((currentFile+1) * 100 / overallSamples);
                printf("%5lu (%3.0f%%):\tFile '%s'", (currentFile+1), percent, currentImageFile.c_str());
                fflush(stdout);
                resetCursor();
            }
            // Calculate feature vector from current image file
            calculateFeaturesFromInput(currentImageFile, featureVector, hog);
            if (!featureVector.empty()) {
               

                File << ((currentFile < positiveTrainingImages.size()) ? "+1" : "-1");
                // Save feature vector components
                for (unsigned int feature = 0; feature < featureVector.size(); ++feature) {
                    File << " " << (feature + 1) << ":" << featureVector.at(feature);
                }
                File << endl;
            }
        }
        printf("\n");
        File.flush();
        File.close();
    } else {
        printf("Error opening file '%s'!\n", featuresFile.c_str());
        return EXIT_FAILURE;
    }
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Pass features to machine learning algorithm">
    /// Read in and train the calculated feature vectors
    printf("Calling SVMlight\n");
    SVMlight::getInstance()->read_problem(const_cast<char*> (featuresFile.c_str()));
    SVMlight::getInstance()->train(); // Call the core libsvm training procedure
    printf("Training done, saving model file!\n");
    SVMlight::getInstance()->saveModelToFile(svmModelFile);
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Generate single detecting feature vector from calculated SVM support vectors and SVM model">
    printf("Generating representative single HOG feature vector using svmlight!\n");
    vector<float> descriptorVector;
    vector<unsigned int> descriptorVectorIndices;
    // Generate a single detecting feature vector (v1 | b) from the trained support vectors, for use e.g. with the HOG algorithm
    SVMlight::getInstance()->getSingleDetectingVector(descriptorVector, descriptorVectorIndices);
    // And save the precious to file system
    saveDescriptorVectorToFile(descriptorVector, descriptorVectorIndices, descriptorVectorFile);
    // </editor-fold>

    
    // <editor-fold defaultstate="collapsed" desc="Test detecting vector">
    printf("Testing custom detection using camera\n");
    
    vector<float> descriptorVector2 = loadDescriptorVectorFromFile(descriptorVectorFile);
    
   // saveDescriptorVectorToFile(descriptorVector2, descriptorVectorIndices, "majd.dat");
    printf("%d\n",descriptorVector2.size());
  
    hog.setSVMDetector(descriptorVector2); // Set our custom detecting vector
 //   VideoCapture cap(0); // open the default camera

   
     printf("loading image\n");
    Mat testImage = imread(argv[1]);
    cvtColor(testImage, testImage, CV_BGR2GRAY); // If you want to work on grayscale images
    printf("Detecting..");
        detectTest(hog, testImage);
                cvNamedWindow("Good Matches",CV_WINDOW_AUTOSIZE);
          printf("Showing..");
        imshow("Good Matches", testImage);
        imwrite("majd100.jpg",testImage);
        printf("WTF?!");
       // getchar();
    //    cvWaitKey(10);
    return 0;
}
 
  
     
*/
static string toLowerCase(const string& in) {
    string t;
    for (string::const_iterator i = in.begin(); i != in.end(); ++i) {
        t += tolower(*i);
    }
    return t;
}
#ifndef NODE
static void getFilesInDirectory(const string& dirName, vector<string>& fileNames, const vector<string>& validExtensions) {
    ROS_INFO("Opening directory %s\n", dirName.c_str());
    struct dirent* ep;
    size_t extensionLocation;
    DIR* dp = opendir(dirName.c_str());
    if (dp != NULL) {
        while ((ep = readdir(dp))) {
            // Ignore (sub-)directories like . , .. , .svn, etc.
            if (ep->d_type & DT_DIR) {
                continue;
            }
            extensionLocation = string(ep->d_name).find_last_of("."); // Assume the last point marks beginning of extension like file.ext
            // Check if extension is matching the wanted ones
            string tempExt = toLowerCase(string(ep->d_name).substr(extensionLocation + 1));
            if (find(validExtensions.begin(), validExtensions.end(), tempExt) != validExtensions.end()) {
            //    ROS_INFO("Found matching data file '%s'", ep->d_name);
                fileNames.push_back((string) dirName + ep->d_name);
            } else {
             //   ROS_INFO("Found file does not match required file type, skipping: '%s'", ep->d_name);
            }
        }
        (void) closedir(dp);
    } else {
        ROS_INFO("Error opening directory '%s'!\n", dirName.c_str());
    }
    return;
}
#endif
//#define LEARNING

#ifdef SVM
int main(int argc, char** argv)
{
 
  SiftFeatureDetector detector;
  vector<KeyPoint> keypoints;	

  // computing descriptors
  Ptr<DescriptorExtractor > extractor(new SiftDescriptorExtractor());// extractor;
  Mat descriptors;
  Mat training_descriptors(1,extractor->descriptorSize(),extractor->descriptorType());
  Mat img;

	//cout << "------- build vocabulary ---------\n";

 
  
  string carDriverSamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/car_driver/positive/";
  string carPassengerSamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/car_passenger/positive/";
  string backgroundSamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/car_driver/negative/";    
  string trainingamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/training_set/";    
  
  string carFrontSamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/car_front/positive/";    
  string carRearSamplesDir = "/home/isl/darpa/robil/robil/C23_ObjectRecognition/training/car_rear/positive/";    
  static vector<string> carRearTrainingImages;
  static vector<string> carFrontTrainingImages;
  static vector<string> trainingImages;
  static vector<string> carDriverTrainingImages;
  static vector<string> carPassengerTrainingImages;
  static vector<string> backgroundTrainingImages;
  static vector<string> validExtensions;
  validExtensions.push_back("jpg");
  #ifdef LEARNING
 // getFilesInDirectory(posSamplesDir, positiveTrainingImages, validExtensions);
//  getFilesInDirectory(negSamplesDir, negativeTrainingImages, validExtensions);
  getFilesInDirectory(trainingamplesDir, trainingImages, validExtensions);
  int count = 0;
  cout << "extract descriptors.."<<endl;
  for(int i = 0; i < trainingImages.size() ; i++) {
    
    string fileName = trainingImages.at(i);

    Mat img = imread(fileName.c_str());
  	detector.detect(img, keypoints);
    extractor->compute(img, keypoints, descriptors);

    training_descriptors.push_back(descriptors);
   // cout << ".";
    img.release();
    count++;

  }
  
 /* cv::FileStorage fs3("training/car_driver/training_descriptors.yml", FileStorage::READ);
  fs3 ["training_descriptors" ] >> training_descriptors;
  fs3.release();
  
  */
  
	//cout << endl;
	cout << "Count: " << count << endl;
	cout << "Total descriptors: " << training_descriptors.rows << endl;
	count = 0;
	BOWKMeansTrainer bowtrainer(1000); //num clusters
	bowtrainer.add(training_descriptors);
  cout << "cluster BOW features" << endl;
  
  Mat vocabulary2 = bowtrainer.cluster();
  cv::FileStorage fs("training/vocabulary.yml", FileStorage::WRITE);
  fs <<  "vocabulary" << vocabulary2;
  fs.release();
 
  cout << " Loading vocabulary ... " << endl;
 #endif
  Mat vocabulary;
  
  cv::FileStorage fs2("training/vocabulary.yml", cv::FileStorage::READ);
  
  fs2["vocabulary"] >> vocabulary;
  fs2.release();
  
 // Ptr<DescriptorMatcher> matcher(new BruteForceMatcher<L2<float> >());
 
  FlannBasedMatcher *matcher = new FlannBasedMatcher;
  
  BOWImgDescriptorExtractor bowide(extractor,matcher);
  
  bowide.setVocabulary(vocabulary);

  	//setup training data for classifiers
  #ifdef LEARNING
  map<string,Mat> classes_training_data; classes_training_data.clear();

  cout << "------- train SVMs ---------\n";
  #endif
  Mat response_hist;
  #ifdef LEARNING
  cout << "look in train data"<<endl;
  count = 0;
  int total_samples = 0;
  getFilesInDirectory(carDriverSamplesDir,carDriverTrainingImages, validExtensions);
  for(int i = 0; i < carDriverTrainingImages.size() ; i++) {

    string fileName = carDriverTrainingImages.at(i);
    Mat img = imread(fileName.c_str());

    detector.detect(img, keypoints);
    bowide.compute(img, keypoints, response_hist);
    if(response_hist.rows || response_hist.cols) {
      cout << " POS ERRRORRR !!!!! " << endl;
    }
    string c_ = "car_driver";
    if(classes_training_data.count(c_) == 0) { //not yet created...
      classes_training_data[c_].create(0,response_hist.cols,response_hist.type());
    }
    classes_training_data[c_].push_back(response_hist);
    total_samples++;
    img.release();

  }
  getFilesInDirectory(backgroundSamplesDir, backgroundTrainingImages, validExtensions);
  for(int i = 0; i < backgroundTrainingImages.size() ; i++) {

    string fileName = backgroundTrainingImages.at(i);
    Mat img = imread(fileName.c_str());
    detector.detect(img, keypoints);
    bowide.compute(img, keypoints, response_hist);
    if(response_hist.rows || response_hist.cols) {
      cout << " NEG ERRRORRR !!!!! " << endl;
    }
    string c_ = "background";
    if(classes_training_data.count(c_) == 0) { //not yet created...

      classes_training_data[c_].create(0,response_hist.cols,response_hist.type());
    }
    classes_training_data[c_].push_back(response_hist);
    total_samples++;
    img.release();

  }
 
  getFilesInDirectory(carPassengerSamplesDir, carPassengerTrainingImages, validExtensions);
  for(int i = 0; i < carPassengerTrainingImages.size() ; i++) {

    string fileName = carPassengerTrainingImages.at(i);
    Mat img = imread(fileName.c_str());
    detector.detect(img, keypoints);
    bowide.compute(img, keypoints, response_hist);
    if(response_hist.rows || response_hist.cols) {
      cout << " NEG ERRRORRR !!!!! " << endl;
    }
    string c_ = "car_passenger";
    if(classes_training_data.count(c_) == 0) { //not yet created...

      classes_training_data[c_].create(0,response_hist.cols,response_hist.type());
    }
    classes_training_data[c_].push_back(response_hist);
    total_samples++;
    img.release();

  }
  
  getFilesInDirectory(carFrontSamplesDir, carFrontTrainingImages, validExtensions);
  for(int i = 0; i < carFrontTrainingImages.size() ; i++) {

    string fileName = carFrontTrainingImages.at(i);
    Mat img = imread(fileName.c_str());
    detector.detect(img, keypoints);
    bowide.compute(img, keypoints, response_hist);
    if(response_hist.rows || response_hist.cols) {
      cout << " NEG ERRRORRR !!!!! " << endl;
    }
    string c_ = "car_front";
    if(classes_training_data.count(c_) == 0) { //not yet created...

      classes_training_data[c_].create(0,response_hist.cols,response_hist.type());
    }
    classes_training_data[c_].push_back(response_hist);
    total_samples++;
    img.release();

  }
  
  getFilesInDirectory(carRearSamplesDir, carRearTrainingImages, validExtensions);
  for(int i = 0; i < carRearTrainingImages.size() ; i++) {

    string fileName = carRearTrainingImages.at(i);
    Mat img = imread(fileName.c_str());
    detector.detect(img, keypoints);
    bowide.compute(img, keypoints, response_hist);
    if(response_hist.rows || response_hist.cols) {
      cout << " NEG ERRRORRR !!!!! " << endl;
    }
    string c_ = "car_rear";
    if(classes_training_data.count(c_) == 0) { //not yet created...

      classes_training_data[c_].create(0,response_hist.cols,response_hist.type());
    }
    classes_training_data[c_].push_back(response_hist);
    total_samples++;
    img.release();

  }
  cout << endl;
  cout << "Done..." << endl;
	//train 1-vs-all SVMs
  map<string,CvSVM> classes_classifiers;
  for (map<string,Mat>::iterator it = classes_training_data.begin(); it != classes_training_data.end(); ++it) {
    string class_ = (*it).first;
    cout << "training class: " << class_ << ".." << endl;

    Mat samples(0,response_hist.cols,response_hist.type());
    
    Mat labels(0,1,CV_32FC1);

    //copy class samples and label
    samples.push_back(classes_training_data[class_]);
    Mat class_label = Mat::ones(classes_training_data[class_].rows, 1, CV_32FC1);
    labels.push_back(class_label);

    //copy rest samples and label
    for (map<string,Mat>::iterator it1 = classes_training_data.begin(); it1 != classes_training_data.end(); ++it1) {
      string not_class_ = (*it1).first;
      if(not_class_[0] == class_[0]) continue;
      samples.push_back(classes_training_data[not_class_]);
      class_label = Mat::zeros(classes_training_data[not_class_].rows, 1, CV_32FC1);
      labels.push_back(class_label);
    }

    Mat samples_32f; samples.convertTo(samples_32f, CV_32F);
    classes_classifiers[class_].train(samples_32f,labels);
  }
  
	cout << "------- test ---------\n";

	int i =0;
  for (map<string,CvSVM>::iterator it = classes_classifiers.begin(); it != classes_classifiers.end(); ++it) {

      cout << "Saving car data" << endl;
      string name = "training/";
      string path = name + (*it).first + "_svm.dat";
      (*it).second.save(path.c_str()); 
  }
  #endif
  #ifndef LEARNING
  cout << "Loading svm data ..." << endl;
  map<string,CvSVM> classes_classifiers;
  classes_classifiers["car_driver"].load("training/car_driver_svm.dat"); 
  classes_classifiers["car_passenger"].load("training/car_passenger_svm.dat"); 
  classes_classifiers["car_front"].load("training/car_front_svm.dat"); 
  classes_classifiers["car_rear"].load("training/car_rear_svm.dat"); 
  classes_classifiers["background"].load("training/background_svm.dat"); 
  cout << "Done loading .. " << endl;
  #endif
  
 
//test vs. SVMs

    Mat img2 = imread(argv[1]);

    cout << "Computing descriptors .. " << endl;
    detector.detect(img2, keypoints);
    Mat output;
    cv::drawKeypoints(img2, keypoints, output);
   // imwrite("daniel.jpg",output);
    cv::imwrite("sift_result.jpg", output);
    bowide.compute(img2, keypoints, response_hist);
    
    cout << "Done computing .. " << endl;
    cout << response_hist.rows << ",  " << response_hist.cols << endl;
   
 for (map<string,CvSVM>::iterator it = classes_classifiers.begin(); it != classes_classifiers.end(); ++it) {
      float res = (*it).second.predict(response_hist,false);
      cout << " class: " << (*it).first << ", response: " << res << endl;
      img.release();

    }
   
   int  minScale = -4;
   int  maxScale = 4;
   int  minSize = 25;
   int objWidth = 315;
   int objHeight = 198;
   int scanAreaW = img2.cols;
   int scanAreaH = img2.rows;
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
             
                Mat subImg = img2(cv::Range(y,y+h), cv::Range(x,x+w));
               // cout << "Done subimage" << endl;
                    detector.detect(subImg, keypoints);
                bowide.compute(subImg, keypoints, response_hist);
              //  cout << "Done subimage 2" << endl;
                 char buff[1000];
                sprintf(buff,"training/majd/image%d.jpg%c",count2,'\0');
                imwrite(buff,subImg);
                if(response_hist.empty()) continue;
              int i =0;
               for (map<string,CvSVM>::iterator it = classes_classifiers.begin(); it != classes_classifiers.end(); ++it) {
                float res = (*it).second.predict(response_hist,true);
           //     cout << " class: " << (*it).first << ", response: " << res << endl;
                if(i == 0) {
                  bg_res = res;
                  }
                if(i ==3 && res < -0.2 && bg_res > 0.55 && res < best_res && bg_res > best_bg_res) {
                 
                  
                  min = res;
                  x_ = x;
                  y_ = y;
                  width = w;
                  height = h;
                  best_res = res;
                  best_bg_res = bg_res;
                  cout << "Car: " << res << ",Bg: " << best_bg_res << endl;
                  number_of_patches++;
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
      return 0;
    }
    Mat subImg = img2(cv::Range(y_,y_+height), cv::Range(x_,x_+width));
     Mat output2;
      detector.detect(subImg, keypoints);
    cv::drawKeypoints(subImg, keypoints, output2);
   // imwrite("daniel.jpg",output);
    cv::imwrite("sift_result.jpg", output2);
    
    imwrite("best.jpg",subImg);

 // cout <<"done "<< best_res << endl;
  return 0;
  
 
 
} 
#endif
