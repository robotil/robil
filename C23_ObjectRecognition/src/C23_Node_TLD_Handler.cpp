#include "C23_Node_TLD_Handler.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "TLDUtil.h"
#include "ros/ros.h"
using namespace tld;

static string window_name;
static CvFont font;
static IplImage *img0;
static IplImage *img1;
static CvPoint point;
static CvRect *bb;
static int drag = 0;

C23_Node_TLD_Handler::C23_Node_TLD_Handler(TldMode mode, const char* modelPath) {
    _mode = NOT_INITIALIZED;
    tld = new tld::TLD();
    tld->learningEnabled = true;
    _modelPath = modelPath;
    
    
    _initialBB = NULL;
    _mode = mode;
    _window = "Image window";
    //showOutput = 1;
    //showNotConfident = true;

   // reinit = 0;

    //loadModel = false;

   // exportModelAfterRun = false;
   // modelExportFile = "model";
    //seed = 0;
    
}
TldRetval C23_Node_TLD_Handler::init(IplImage* img) {
    Mat grey(img->height, img->width, CV_8UC1);
    cvtColor(cv::Mat(img), grey, CV_BGR2GRAY);
    ROS_INFO("Init func: 1\n");
    tld->detectorCascade->imgWidth = grey.cols;
    tld->detectorCascade->imgHeight = grey.rows;
    tld->detectorCascade->imgWidthStep = grey.step;
    ROS_INFO("Init func: 2\n");
    if(_mode == LEARNING && img != NULL)
    {
    
        CvRect box;
        ROS_INFO("Init func: 3\n");
        if(getBBFromUser(img, box) == BB_ERROR)
        {
            return BB_ERROR;
        }
        ROS_INFO("Init func: 4\n");
        if(_initialBB == NULL)
        {
            _initialBB = new int[4];
        }

        _initialBB[0] = box.x;
        _initialBB[1] = box.y;
        _initialBB[2] = box.width;
        _initialBB[3] = box.height;
    
    }
   
    //FILE *resultsFile = NULL;

    //if(printResults != NULL)
   // {
   //     resultsFile = fopen(printResults, "w");
   // }
    ROS_INFO("Init func: 5\n");
    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;
    
    if(_mode ==TRACKING && _modelPath != NULL)
    {
        ROS_INFO("Loading model..\n");
        tld->readFromFile(_modelPath);
        reuseFrameOnce = true;
    }
    else if(_initialBB != NULL)
    {
        Rect bb = tldArrayToRect(_initialBB);

        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);

        tld->selectObject(grey, &bb);
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }
}

/*
  
    
    
}
*/

TldRetval C23_Node_TLD_Handler::saveCurrentModel(const char* path) {
    tld->writeToFile(path);
    
}

TldRetval C23_Node_TLD_Handler::processFrame(IplImage* img, int *x, int *y, int *width, int *height, float *confident) {


    double tic = cvGetTickCount();
    Mat grey(img->height, img->width, CV_8UC1);

    cvtColor(cv::Mat(img), grey, CV_BGR2GRAY);
    tld->processImage(img);
    char key = cvWaitKey(10);
    if(key == 's') {
        tld->writeToFile(_modelPath);
        ROS_INFO("Saving!\n");
        
    }
/*
        if(printResults != NULL)
        {
            if(tld->currBB != NULL)
            {
                fprintf(resultsFile, "%d %.2d %.2d %.2d %.2d %f\n", imAcq->currentFrame - 1, tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
            }
            else
            {
                fprintf(resultsFile, "%d NaN NaN NaN NaN NaN\n", imAcq->currentFrame - 1);
            }
        }
*/
        double toc = (cvGetTickCount() - tic) / cvGetTickFrequency();

        toc = toc / 1000000;

        float fps = 1 / toc;

        //int confident = (tld->currConf >= threshold) ? 1 : 0;
        if(tld->currBB != NULL) {
            *confident = tld->currConf;
            *x = tld->currBB->x;
            *y =  tld->currBB->y;
            *width =  tld->currBB->width;
            *height =  tld->currBB->height;
           // return SUCCESS;
        } else {
            *x = -1;
            *y = -1;
            *width = -1;
            *height = -1;
            *confident = 0;
        }
       
    
        CvScalar yellow = CV_RGB(255, 255, 0);
        CvScalar blue = CV_RGB(0, 0, 255);
        CvScalar black = CV_RGB(0, 0, 0);
        CvScalar white = CV_RGB(255, 255, 255);

        if(tld->currBB != NULL) {
             CvScalar rectangleColor = (confident) ? blue : yellow;
             cvRectangle(img, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 8, 8, 0);
        }
     
///////    My adding   /////////////////////

img1 = (IplImage *) cvClone(img);
cvShowImage("Image window", img1);
cvReleaseImage(&img1);

///////////////////////////////////////////

        
         return SUCCESS;
}

static void mouseHandler(int event, int x, int y, int flags, void *param)
{
    ROS_INFO("Mouse event 1\n");
    char *window = "Image window";
    /* user press left button */
    if(event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        ROS_INFO("Mouse event 2\n");
        point = cvPoint(x, y);
        drag = 1;
    }

    /* user drag the mouse */
    if(event == CV_EVENT_MOUSEMOVE && drag)
    {
        ROS_INFO("Mouse event 3\n");
        img1 = (IplImage *) cvClone(img0);

        cvRectangle(img1, point, cvPoint(x, y), CV_RGB(255, 0, 0), 1, 8, 0);

        cvShowImage(window, img1);
        cvReleaseImage(&img1);
    }

    /* user release left button */
    if(event == CV_EVENT_LBUTTONUP && drag)
    {
        ROS_INFO("Mouse event 4\n");
        *bb = cvRect(point.x, point.y, x - point.x, y - point.y);
        drag = 0;
    }
    
}


int C23_Node_TLD_Handler::getBBFromUser(IplImage *img, CvRect &rect)
{
   
    img0 = (IplImage *) cvClone(img);
    rect = cvRect(-1, -1, -1, -1);
    bb = &rect;
    bool correctBB = false;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, 8);
    ROS_INFO("Setting callback...\n");
    cvNamedWindow(_window, CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback(_window, mouseHandler, (void*)_window);
    cvPutText(img0, "Draw a bounding box and press Enter", cvPoint(0, 60),
              &font, cvScalar(255, 255, 0));
    cvShowImage(_window, img0);
    cvSetMouseCallback(_window, mouseHandler, NULL);
    while(!correctBB)
    {
        char key = cvWaitKey(0);
        ROS_INFO("Got key\n");
        if(tolower(key) == 'q')
        {
            return SUCCESS;
        }

        if(((key == '\n') || (key == '\r') || (key == '\r\n')) && (bb->x != -1) && (bb->y != -1))
        {
            ROS_INFO("Got enter..\n");
            correctBB = true;
        }
    }
    ROS_INFO("Destroying ..");
   // cvDestroyWindow(_window);

    if(rect.width < 0)
    {
        rect.x += rect.width;
        rect.width = abs(rect.width);
    }

    if(rect.height < 0)
    {
        rect.y += rect.height;
        rect.height = abs(rect.height);
    }

    cvSetMouseCallback(_window, NULL, NULL);
    
    cvReleaseImage(&img0);
    cvReleaseImage(&img1);

    return SUCCESS;
}
