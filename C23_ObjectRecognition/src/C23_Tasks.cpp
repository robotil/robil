#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include "C23_Detector.hpp"
#include <PoseController/neck_movement.h>
#include <PoseController/back_lbz_neck_ay.h>
#include <std_srvs/Empty.h>


using namespace std;
using namespace C0_RobilTask;

string getValueFromArgument(Arguments& args, const char* key) {
	if (args.find(key) != args.end()) {
		std::stringstream msg;

		msg << args[key];

		string target_str = msg.str();
		return target_str;
	} else {
		return "";
	}
}
class C23_SearchObject: public RobilTask {
public:
	C23_SearchObject(C23_Detector *detector) :
			RobilTask("/searchObject"), _detector(detector) {
		ROS_INFO("Instance of SearchObjectServer has started.");
        c23_start_posecontroller = nh.serviceClient<std_srvs::Empty>("/PoseController/start");
        c23_move_neck = nh.serviceClient<PoseController::back_lbz_neck_ay>("/PoseController/back_lbz_neck_ay");
    //    c23_move_back = nh.serviceClient<PoseController::back_movement>("/PoseController/back_movement");
		_detector->is_search = true;
	}

	TaskResult task(const string& name, const string& uid, Arguments& args) {
		ROS_INFO("searchObject::I was called\n");
		if (isPreempt()) {
			/* SHOULD STOP EVERYTHING RUNNING! */
			ROS_INFO("searchObject::I've been preempted");
            _detector->stopDetection();
			return TaskResult::Preempted();
		}
		string target = getValueFromArgument(args, "target");
		bool res;
        int state =0;
        double back_angle = 0;
        double angle = -0.3;
		while (!isPreempt()) {
        //    ros::Rate loop_rate(10);
            res = _detector->detect(target);
            ros::Duration(0.5).sleep();
            
           if (_detector->_found) {
          //   if (false) {
                return TaskResult(SUCCESS, "OK");
            } else {
                _detector->stopDetection();
                return TaskResult(FAULT, "Object isn't detected");
                std_srvs::Empty e;
                PoseController::back_lbz_neck_ay msg;
              //  PoseController::back_movement msg2;
                if(angle > 0.7 && state == 4) {
                        return TaskResult(FAULT, "Object isn't detected");
                }
                if(angle > 0.7 && state != 4) {
                        angle = -0.3;
                        state++;
                    
                } 
                switch (state) {
                    case 0:
                        back_angle = 0;
                        break;
                    case 1:
                        back_angle = 0.3;
                        break;
                    case 2:
                        back_angle = -0.3;
                        break;
                    case 3:
                        back_angle = 0.61;
                        break;
                    case 4:
                        back_angle = -0.61;
                        break;
                }
              
              //  msg.request.neck_ay = angle;
              msg.request.neck_ay = angle;
              msg.request.back_lbz = back_angle;
                angle+=0.1;
               // msg2.request.back_lbz = back_angle;
                
              //  c23_start_posecontroller.call(e);
                if(c23_move_neck.call(msg))
                {
                
               //    cout << "Worked" << endl;
                }         
              //  if(c23_move_back.call(msg2))
              //  {
                    
                   
             //   }         
              //  return TaskResult(FAULT, "Object isn't detected");
            }   
        //   loop_rate.sleep();
		}
		_detector->stopDetection();
	  return TaskResult(SUCCESS, "OK");
	}

private:
	C23_Detector *_detector;
    ros::ServiceClient  c23_start_posecontroller;
    ros::ServiceClient  c23_move_neck,c23_move_back;
    ros::NodeHandle nh;
    

};

class C23_TrackObject: public RobilTask {
public:
	C23_TrackObject(C23_Detector *detector) :
			RobilTask("/trackObject"), _detector(detector) {
		ROS_INFO("Instance of SearchObjectServer has started.");
		_detector->is_search = false;

	}

	TaskResult task(const string& name, const string& uid, Arguments& args) {
		ROS_INFO("trackObject::I was called\n");
		if (isPreempt()) {
			/* SHOULD STOP EVERYTHING RUNNING! */
            _detector->stopDetection();
			ROS_INFO("trackObject::I've been preempted");
			return TaskResult::Preempted();
		}
		string target = getValueFromArgument(args, "target");
		bool res;
        int count = 0;
        res = _detector->detect(target);
        while (!isPreempt()) {
            ros::Duration(0.5).sleep();
			if (!(_detector->_found) && ++count >= 100) {
                return TaskResult(FAULT, "Object isn't detected");
			
          
			} else {
                    count = 0;
            }
            
        }
		
		
	}
private:
	C23_Detector *_detector;

};

class C23_TakePicture: public RobilTask {
public:
	C23_TakePicture(C23_Detector *detector) :
			RobilTask("/takePicture"), _detector(detector) {
		ROS_INFO("Instance of takePictureTask has started.");
		_detector->is_search = true;
	}

	TaskResult task(const string& name, const string& uid, Arguments& args) {
		ROS_INFO("takePicture::I was called\n");
		if (isPreempt()) {
			/* SHOULD STOP EVERYTHING RUNNING! */
			ROS_INFO("takePicture::I've been preempted");
			return TaskResult::Preempted();
		}
		bool res;
	      	res = _detector->detect("Picture");
	        return TaskResult(SUCCESS, "OK");
	}

private:
	C23_Detector *_detector;

};
