#ifndef __TRACK_OBJECT__HPP
#define __TRACK_OBJECT__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include "C23_Node.hpp"
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
using namespace std;
using namespace C0_RobilTask;

class TrackObjectServer: public RobilTask {

protected:

    string _name;
    C23_Node *_detector;
    ros::NodeHandle nh_;
   

public:
    TrackObjectServer(C23_Node &detector):
        RobilTask("/trackObject"),
        _detector(&detector),
        _name("/trackObject")
    {
        ROS_INFO("instance of TrackObjectServer started.");

    }
    void dataChangeD() {
    
    }
    TaskResult task(const string& name, const string& uid, Arguments& args) {
        ROS_INFO("trackObject::I was called\n");
        if (isPreempt()){
            ROS_INFO("trackObject::was preepmpted\n");
            _detector->stopDetection();
            return TaskResult::Preempted();
        }
        if(args.find("target") != args.end() ) {
            std::stringstream msg; 
           
            msg << args["target"];
           
            string target_str = msg.str();
            const char* target = target_str.c_str();
            
            char buffer[1000];
            sprintf(buffer,"%s/models/%s.mdl%c",ros::package::getPath("C23_ObjectRecognition").c_str(),target,'\0');
            /* Check if the target model exists under /models */
            if( access( buffer, F_OK ) == -1 ) {
                char* str = "trackObject was called with unknown target";
                return TaskResult(FAULT, str);
            }
            

            _detector->detectAndTrack(target);
            
            int count = 0;
            while(!isPreempt()) {
                count++;
                if(_detector->x != -1) {
                    ROS_INFO("trackObject:: object detected!");
                   
                    count = 0;
                    return TaskResult(SUCCESS, "OK");
                    
                    
               } else {
                   
                    char* str = "trackObject didnt' detect any object";
                    if (count > 1000000) {
                        ROS_INFO("trackObject object not detect - %d\n ", count);
                        _detector->stopDetection();
                        return TaskResult(FAULT, str);
                    }
                }
                
                
               
            }
            return TaskResult::Preempted();
        } else {
            char* str = "trackObject was called without an object";
            ROS_INFO("%s.\n", str);
             _detector->stopDetection();
            return TaskResult(FAULT, str);
        }
        return TaskResult(SUCCESS, "OK");
    }

};
#endif
