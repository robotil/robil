#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>
#include "C23_Detector.hpp"


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
		_detector->is_search = true;
	}

	TaskResult task(const string& name, const string& uid, Arguments& args) {
		ROS_INFO("searchObject::I was called\n");
		if (isPreempt()) {
			/* SHOULD STOP EVERYTHING RUNNING! */
			ROS_INFO("searchObject::I've been preempted");
			return TaskResult::Preempted();
		}
		string target = getValueFromArgument(args, "target");
		bool res;
		while (!isPreempt()) {
      res = _detector->detect(target);
      if (_detector->x != -1) {
          return TaskResult(SUCCESS, "OK");
			} else {
          return TaskResult(FAULT, "Object isn't detected");
			}
		}
	  return TaskResult(SUCCESS, "OK");
	}

private:
	C23_Detector *_detector;

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
			ROS_INFO("trackObject::I've been preempted");
			return TaskResult::Preempted();
		}
		string target = getValueFromArgument(args, "target");
		bool res;
			res = _detector->detect(target);
			if (_detector->x != -1) {
          return TaskResult(SUCCESS, "OK");
			} else {
          return TaskResult(FAULT, "Object isn't detected");
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
