#ifndef __SEARCH_OBJECT__HPP
#define __SEARCH_OBJECT__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include "C23_Node.hpp"

using namespace std;
using namespace C0_RobilTask;

class SearchObjectServer: public RobilTask {

protected:

    string _name;
    C23_Node *_detector;
    ros::Publisher objectDetectedPublisher;
    ros::Publisher objectDeminsionsPublisher;
    ros::NodeHandle nh_;


public:
    SearchObjectServer(C23_Node &detector):
        RobilTask("/searchObject"),
        _detector(&detector),
        _name("/searchObject")
    {
        ROS_INFO("instance of TrackObjectServer started.");
       
    }

    TaskResult task(const string& name, const string& uid, Arguments& args) {
        ROS_INFO("searchObject::I was called\n");
        
        if (isPreempt()){
            ROS_INFO("searchObject::was preepmpted\n");
            _detector->stopDetection();
            return TaskResult::Preempted();
        }
        if(args.find("target") != args.end() ) {
            std::stringstream target; target<<args["target"];
            string target_str = target.str();
            
            ROS_INFO("searchObject called with: %s",target_str.c_str());
             if(!strcmp(target_str.c_str(),"CAR_PASSENGER")) {
                 _detector->detectAndTrack(CAR_PASSENGER);
             }
             if(!strcmp(target_str.c_str(),"CAR_DRIVER")) {
                 _detector->detectAndTrack(CAR_DRIVER);
             }
             if(!strcmp(target_str.c_str(),"CAR_FRONT")) {
                 _detector->detectAndTrack(CAR_FRONT);
             }
             if(!strcmp(target_str.c_str(),"CAR_REAR")) {
                 _detector->detectAndTrack(CAR_DRIVER);
             }
            _detector->startDetection();
            
            int count = 0;
            while(!isPreempt()) {
                count++;
                if(_detector->x != -1) {
                    ROS_INFO("searchObject:: object detected!");
                   
                    count = 0;
                    return TaskResult(SUCCESS, "OK");
                    
                    
               } else {
                   
                    char* str = "searchObject didnt' detect any object";
                    if (count > 1000000) {
                        ROS_INFO("searchObject object not detect - %d\n ", count);
                        _detector->stopDetection();
                        return TaskResult(FAULT, str);
                    }
                }
                
                
               
            }
            return TaskResult::Preempted();
        } else {
            char* str = "searchObject was called without an object";
            ROS_INFO("%s.\n", str);
             _detector->stopDetection();
            return TaskResult(FAULT, str);
        }
        return TaskResult(SUCCESS, "OK");
    }

};
#endif
