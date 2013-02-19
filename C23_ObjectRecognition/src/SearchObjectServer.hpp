#ifndef __SEARCH_OBJECT__HPP
#define __SEARCH_OBJECT__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include "C23_Node.hpp"
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
using namespace std;
using namespace C0_RobilTask;

class SearchObjectServer: public RobilTask {

protected:
    //ros::NodeHandle _node;
    //Server _server;
    string _name;
    C23_Node *_detector;
    ros::Publisher objectDetectedPublisher;
    ros::Publisher objectDeminsionsPublisher;
    ros::NodeHandle nh_;
   // FEEDBACK _feedback;
   // RESULT _result;

public:
    TrackObjectServer(C23_Node &detector):
        RobilTask("/SearchObject"),
        _detector(&detector),
        _name("/SearchObject")
        //, boost::bind(&TrackObjectServer::dataChanged, this, _1)
    {
        ROS_INFO("instance of TrackObjectServer started.");
        objectDetectedPublisher = nh_.advertise<C23_ObjectRecognition::C23C0_OD>("C23/object_detected", 1);
        objectDeminsionsPublisher = nh_.advertise<C23_ObjectRecognition::C23C0_ODIM>("C23/object_deminsions", 1);
    }
    void dataChangeD() {
    
    }
    TaskResult task(const string& name, const string& uid, Arguments& args) {
        ROS_INFO("I was called\n");
        
        if (isPreempt()){
            _detector->stopDetection();
            return TaskResult::Preempted();
        }
        if(args.find("object") != args.end() ) {
            std::stringstream numbers; numbers<<args["object"];
            int x;
            numbers>>x;
            switch (x) {
                case 0:
                    _detector->detectAndTrack(CAR_DRIVER);
                    break;
                case 1:
                _detector->detectAndTrack(CAR_PASSENGER);
                    break;
                case 2:
                    _detector->detectAndTrack(CAR_FRONT);
                    break;
                case 3:
                    _detector->detectAndTrack(CAR_BEHIND);
                    break;
                default:
                    break;
            }
            _detector->startDetection();
            C23_ObjectRecognition::C23C0_OD msg;
            C23_ObjectRecognition::C23C0_ODIM msg2;
            while(!isPreempt()) {
                if(_detector->x != -1) {
                    msg.ObjectDetected = 1;
                    msg2.x = _detector->x;
                    msg2.y = _detector->y;
                    msg2.width = _detector->width;
                    msg2.height = _detector->height;
                    objectDeminsionsPublisher.publish(msg2);
               } else {
                    msg.ObjectDetected = 0;
                }
                objectDetectedPublisher.publish(msg);
               
            }
            _detector->stopDetection();
            return TaskResult::Preempted();
        } else {
            char* str = "SearchObject was called without an object";
            ROS_INFO("%s.\n", str);
            return TaskResult(FAULT, str);
        }
        
        return TaskResult(SUCCESS, "OK");
    }

};
#endif
