#ifndef __LEARN_OBJECT__HPP
#define __LEARN_OBJECT__HPP

#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include "C23_Node.hpp"
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
using namespace std;
using namespace C0_RobilTask;

class LearnObjectServer: public RobilTask {

protected:

    string _name;
    C23_Node *_detector;
    ros::NodeHandle nh_;
   

public:
    LearnObjectServer(C23_Node &detector):
        RobilTask("/learnObject"),
        _detector(&detector),
        _name("/learnObject")
    {
        ROS_INFO("instance of LearnObjectServer started.");

    }
    TaskResult task(const string& name, const string& uid, Arguments& args) {
         ROS_INFO("learnObject::I was called\n");
        if(args.find("target") != args.end()) {
            std::stringstream msg; 
           
            msg << args["target"];
           
            string target_str = msg.str();

            
            const char* target = target_str.c_str();
            
            ROS_INFO("Learning object: %s.",target);
            _detector->learnObject(target);
           
            return TaskResult(SUCCESS, "OK");
           
        } else {
            char* str = "learnObject was called without a target or filename 2";
            ROS_INFO("%s.\n", str);
             _detector->stopDetection();
            return TaskResult(FAULT, str);
        }
        return TaskResult(SUCCESS, "OK");
    }

};
#endif
