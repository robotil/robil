#ifndef _PATH_PLANNING_SERVER_H_
#define _PATH_PLANNING_SERVER_H_

//tasks dependencies
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>

//messages
#include <C31_PathPlanner/C31_PlanPath.h>
#include <C31_PathPlanner/C31_GetPath.h>
#include <C31_PathPlanner/C31_Exception.h>

#include "C22_GroundRecognitionAndMapping/C22C0_PATH.h"

#include <C23_ObjectRecognition/C23C0_OD.h>
#include <C23_ObjectRecognition/C23C0_ODIM.h>

#include <vector>

using namespace std;
using namespace C0_RobilTask;

#include "../Vec2d.hpp"


vector<Vec2d> path;
boost::mutex path_mtx;

namespace {
	Vec2d extractLocation(const C22_GroundRecognitionAndMapping::C22C0_PATH &res){
		Vec2d gps(0,0);

		size_t h = res.row.size();
		if(h){

			ROS_INFO("start parse message of location : pos=%f,%f   map offset=%f,%f",
					 (float) res.robotPos.x, (float) res.robotPos.y, (float) res.xOffset, (float) res.yOffset);

			gps.x = res.robotPos.x;
			gps.y = res.robotPos.y;

			ROS_INFO("... new data gotten : position point = %f,%f", (float) gps.x, (float) gps.y);
		}
		return gps;
	};

	Vec2d extractLocation(const C31_PathPlanner::C31_Location &res){
		Vec2d gps(0,0);

		ROS_INFO("start parse test message of location : pos=%f,%f", (float) res.x, (float) res.y);

		gps.x = res.x;
		gps.y = res.y;

		ROS_INFO("... new data gotten : position point = %f,%f", (float) gps.x, (float) gps.y);

		return gps;
	};

	size_t searchOnPathPosition(const Vec2d& pos, const vector<Vec2d>& path){
		size_t nearest = 0;
		double min_dist = Vec2d::distance(pos, path[0]);
		cout<<"searchOnPathPosition: "<<endl;
		for(size_t i=0;i<path.size();i++){
			double dist = Vec2d::distance(pos, path[i]);
			cout<<"... mid dist="<<min_dist<<", dist="<<dist<<", i="<<i<<", wp="<<path[i]<<", loc="<<pos<<endl;
			if(dist <= min_dist){
				min_dist = dist;
				nearest = i;
				cout<<"... new nearest = "<<i<<endl;
			}
		}
		cout<<"... nearest = "<<nearest<<endl;

		if(nearest == path.size()-1 && nearest == 0){
			return nearest;
		}

		if(nearest == path.size()-1){
			cout<<"... nearest == path.size()-1"<<endl;
			Vec2d A = pos, C = path[nearest], B = path[nearest-1];
			Vec2d b = B-C, c = C-A, a = B-A;
			//http://en.wikipedia.org/wiki/Law_of_cosines
			double G = acos( (B-C).dot(A-C)/(b.len()*c.len()) );
			cout<<"... G="<<Vec2d::r2d(G)<<"deg  A="<<A<<" B="<<B<<" C="<<C<<endl;
			if( G>PI*0.5 ){
				cout<<"... ... G="<<Vec2d::r2d(G)<<"deg < PI/2 => nearest = "<<(nearest+1)<<endl;
				nearest++;
			}

			return nearest;
		}
		Vec2d A = pos, C = path[nearest], B = path[nearest+1];
		Vec2d b = B-C, c = C-A, a = B-A;
		//http://en.wikipedia.org/wiki/Law_of_cosines
		double G = acos( (B-C).dot(A-C)/(b.len()*c.len()) );
		cout<<"... G="<<Vec2d::r2d(G)<<"deg  A="<<A<<" B="<<B<<" C="<<C<<endl;
		if( G<PI*0.5 ){
			cout<<"... ... G="<<Vec2d::r2d(G)<<"deg < PI/2 => nearest = "<<(nearest+1)<<endl;
			nearest++;
		}
		return nearest;
	}

}


class PathPlanning:public RobilTask{

public:
    PathPlanning(string name = "/PathPlanning"):
    	RobilTask(name)
    {
    	ROS_INFO("Start task /PathPlanning");

    }

    TaskResult task(const string& name, const string& uid, Arguments& args){

    	ROS_INFO("advertise topic /path <C31_PathPlanner::C31_Waypoints>");
    	ros::Publisher c31_PathPublisher =
    			_node.advertise<C31_PathPlanner::C31_Waypoints>("/path", 10);

		ROS_INFO("subscribe to topic /C22_pub <C22_GroundRecognitionAndMapping::C22C0_PATH>");
    	ros::Subscriber c22c0Client = _node.subscribe("C22_pub", 1000, &PathPlanning::callbackNewMap, this );

		ROS_INFO("subscribe to topic /C22_pub_stub <C31_PathPlanner::C31_Location>");
    	ros::Subscriber c22c0TestClient = _node.subscribe("C22_pub_stub", 1000, &PathPlanning::callbackTestPosition, this );
		
        while(true){
            if (isPreempt()){

                /* HERE PROCESS PREEMPTION OR INTERAPT */

            	return TaskResult::Preempted();
            }

 			publish_new_plan(c31_PathPublisher);

            /* SLEEP BETWEEN LOOP ITERATIONS */
            sleep(1000); //millisec
        }

        return TaskResult::FAULT();
    }

    void publish_new_plan(ros::Publisher& c31_PathPublisher){
    	//if(c31_PathPublisher.getNumSubscribers()>0){
			ROS_INFO("PUBLISH PATH");
			C31_PathPlanner::C31_Waypoints res_path;
			vector<Vec2d> gpath;
			{boost::mutex::scoped_lock locker(path_mtx);
				 gpath = ::path;
			}
			size_t size_of_path = gpath.size();
			ROS_INFO("path size is %i",(int)size_of_path);
			if(size_of_path>0){
				size_t start = searchOnPathPosition(position, gpath);
				for( size_t i=start;i<size_of_path;i++ ){
				  C31_PathPlanner::C31_Location loc;
					  loc.x=gpath[i].x;
					  loc.y=gpath[i].y;
				  res_path.points.push_back(loc);
				}
			}
		if(c31_PathPublisher.getNumSubscribers()>0){
			c31_PathPublisher.publish(res_path);
		}
    }



    void callbackNewMap(const C22_GroundRecognitionAndMapping::C22C0_PATH::ConstPtr & msg){

		position = extractLocation(*msg);

	}
    void callbackTestPosition(const C31_PathPlanner::C31_Location::ConstPtr & msg){

		position = extractLocation(*msg);

	}

    Vec2d position;
};

class PathPlanningFocus:public RobilTask{
public:
    PathPlanningFocus(string name = "/PathPlanningFocus"):
    	RobilTask(name)
    {
    	ROS_INFO("Start task /PathPlanningFocus");
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){


		if( args.find("target")!=args.end() ){
			std::string objectName = args["target"];

			ROS_INFO("%s: set planning goal to target = %s", _name.c_str(), objectName.c_str());

			if(objectName == "test1"){
				vector<Vec2d> p;
				p.push_back(Vec2d(1,0));
				p.push_back(Vec2d(2,0));
				p.push_back(Vec2d(3,0));
				p.push_back(Vec2d(4,0));
				p.push_back(Vec2d(5,0));

				copyToPath(p);
			}
			if(objectName == "Qual1P1"){
				vector<Vec2d> p;
				p.push_back(Vec2d(1.005, -0.082));
				p.push_back(Vec2d(2.255, -0.12917));
				p.push_back(Vec2d(3.4205, -0.054));
				p.push_back(Vec2d(4.767, -0.046));
				p.push_back(Vec2d(6.069, 0.0447));
				p.push_back(Vec2d(7.55, 0.078));
				p.push_back(Vec2d(8.77, 0.0839));

				p.push_back(Vec2d(9.68, 0.33));

				p.push_back(Vec2d(9.73, 2.149));
				p.push_back(Vec2d(9.755, 3.379));
				p.push_back(Vec2d(9.77, 4.8059));
				p.push_back(Vec2d(9.755, 6.137));

				p.push_back(Vec2d(9.765, 8.002));

				p.push_back(Vec2d(11.03, 8.089));
				p.push_back(Vec2d(12.05, 8.065));
				p.push_back(Vec2d(13.36, 8.089));
				p.push_back(Vec2d(14.56, 8.12));
				p.push_back(Vec2d(16.26, 8.12));
				p.push_back(Vec2d(16.70, 8.00));

				copyToPath(p);
			}
			else
			if(objectName == "Qual1P2"){
				vector<Vec2d> p;
				p.push_back(Vec2d(17.42, 8.089));
				p.push_back(Vec2d(17.88, 8.385));
				p.push_back(Vec2d(18.58, 8.39));

				copyToPath(p);
			}
			else
			if(objectName == "Qual1P3"){
				vector<Vec2d> p;
				p.push_back(Vec2d(19.17, 8.07));
				p.push_back(Vec2d(20.5, 8.008));
				p.push_back(Vec2d(21.99, 8.03));

				copyToPath(p);
			}

			return TaskResult(SUCCESS, "OK");
		}

        return TaskResult::FAULT();
    }

    void copyToPath(vector<Vec2d> vec){
    	boost::mutex::scoped_lock locker(path_mtx);
    	path.resize(vec.size());
    	for(size_t i=0; i<vec.size(); i++){
    		path[i] = vec[i];
    	}
    }

};


class WhileInXXXArea:public RobilTask{
protected:
	class Rect{
	public:
		double x,y,w,h;
		Rect(double x, double y, double w, double h):x(x),y(y),w(w),h(h){}
		Rect(Vec2d p1, Vec2d p2):x(p1.x), y(p1.y), w((p2-p1).x), h((p2-p1).y){}
		bool contains(Vec2d p){
			return ( x<=p.x && y<=p.y && (x+w)>=p.x && (y+h)>=p.y);
		}
		bool contains(double x, double y){ return contains(Vec2d(x,y)); }
	};
	class AreaClassifier{
	public:
		enum TYPE{
			ZMP, QuasiStatic
		};
		vector<Rect> zmpArea;
		AreaClassifier(){
			zmpArea.push_back(Rect(Vec2d(-1.69, -6.67), Vec2d(16.29, 10.54)));
			zmpArea.push_back(Rect(Vec2d(19.94, 5.07), Vec2d(23.68, 10.27)));
		}
		TYPE classify(Vec2d position){

			for(size_t i=0;i<zmpArea.size();i++){
				if(zmpArea[i].contains(position))
					return ZMP;
			}
			return QuasiStatic;
		}
	};

	AreaClassifier ac;
	Vec2d position;
public:
	WhileInXXXArea(string name):
    	RobilTask(name)
    {
    	ROS_INFO("Start task %s", name.c_str());
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){
		ROS_INFO("subscribe to topic /C22_pub <C22_GroundRecognitionAndMapping::C22C0_PATH>");
    	ros::Subscriber c22c0Client = _node.subscribe("C22_pub", 1000, &WhileInXXXArea::callbackNewMap, this );

		ROS_INFO("subscribe to topic /C22_pub_stub <C31_PathPlanner::C31_Location>");
    	ros::Subscriber c22c0TestClient = _node.subscribe("C22_pub_stub", 1000, &WhileInXXXArea::callbackTestPosition, this );

    	while(stopCondition()){
    		if(isPreempt()){
    			return TaskResult::Preempted();
    		}
    		sleep(1000); //millisec
    	}
		return TaskResult(SUCCESS, "OK");
    }

    void callbackNewMap(const C22_GroundRecognitionAndMapping::C22C0_PATH::ConstPtr & msg){
		position = extractLocation(*msg);
	}
    void callbackTestPosition(const C31_PathPlanner::C31_Location::ConstPtr & msg){
		position = extractLocation(*msg);
	}

    virtual bool stopCondition()=0;
};

class WhileInZMPArea:public WhileInXXXArea{
public:
	WhileInZMPArea(string name = "/whileInZmpArea"):
		WhileInXXXArea(name)
    {

    }

	bool stopCondition(){
		return ac.classify(position) != AreaClassifier::ZMP;
	}

};

class WhileInQSArea:public WhileInXXXArea{
public:
	WhileInQSArea(string name = "/whileInQSArea"):
		WhileInXXXArea(name)
    {

    }

	bool stopCondition(){
		return ac.classify(position) != AreaClassifier::QuasiStatic;
	}

};


class ObjectRecognition:public RobilTask{
protected:
	vector<Vec2d> p;
	string objectName;
	Vec2d position;
public:
	ObjectRecognition(string name = "/SearchObject"):
    	RobilTask(name)
    {
    	ROS_INFO("Start task %s", name.c_str());

//		p.push_back(Vec2d(1.005, -0.082));
//		p.push_back(Vec2d(2.255, -0.12917));
//		p.push_back(Vec2d(3.4205, -0.054));
//		p.push_back(Vec2d(4.767, -0.046));
//		p.push_back(Vec2d(6.069, 0.0447));
//		p.push_back(Vec2d(7.55, 0.078));
//		p.push_back(Vec2d(8.77, 0.0839));
//
//		p.push_back(Vec2d(9.68, 0.33));
//
//		p.push_back(Vec2d(9.73, 2.149));
//		p.push_back(Vec2d(9.755, 3.379));
//		p.push_back(Vec2d(9.77, 4.8059));
//		p.push_back(Vec2d(9.755, 6.137));
//
//		p.push_back(Vec2d(9.765, 8.002));
//
//		p.push_back(Vec2d(11.03, 8.089));
//		p.push_back(Vec2d(12.05, 8.065));
//		p.push_back(Vec2d(13.36, 8.089));
//		p.push_back(Vec2d(14.56, 8.12));
//		p.push_back(Vec2d(16.26, 8.12));
//
//		p.push_back(Vec2d(17.42, 8.089));
//		p.push_back(Vec2d(17.88, 8.385));
//		p.push_back(Vec2d(18.58, 8.39));
//
//		p.push_back(Vec2d(19.17, 8.07));
//		p.push_back(Vec2d(20.5, 8.008));
//		p.push_back(Vec2d(21.99, 8.03));

    	p.push_back(Vec2d(0.9, -0.177));
    	p.push_back(Vec2d(6.06, -0.17));
    	p.push_back(Vec2d(10.16, 3.995));
    	p.push_back(Vec2d(14.2, 8.43));
    	p.push_back(Vec2d(22.03, 7.99));
    }

    TaskResult task(const string& name, const string& uid, Arguments& args){
		ROS_INFO("subscribe to topic /C22_pub <C22_GroundRecognitionAndMapping::C22C0_PATH>");
    	ros::Subscriber c22c0Client = _node.subscribe("C22_pub", 1000, &ObjectRecognition::callbackNewMap, this );

		ROS_INFO("subscribe to topic /C22_pub_stub <C31_PathPlanner::C31_Location>");
    	ros::Subscriber c22c0TestClient = _node.subscribe("C22_pub_stub", 1000, &ObjectRecognition::callbackTestPosition, this );

    	ROS_INFO("advertise topic /C23/object_deminsions <C23_ObjectRecognition::C23C0_ODIM>");
    	ros::Publisher c23_dim = _node.advertise<C23_ObjectRecognition::C23C0_ODIM>("/C23/object_deminsions", 10);

    	if(args.find("target")!=args.end()){
    		objectName = args["target"];
    	}

    	while(true){
    		if(isPreempt()){
    			return TaskResult::Preempted();
    		}

    		publishObjectLocation(c23_dim);

    		sleep(1000); //millisec
    	}
		return TaskResult(SUCCESS, "OK");
    }

    void callbackNewMap(const C22_GroundRecognitionAndMapping::C22C0_PATH::ConstPtr & msg){
		position = extractLocation(*msg);
	}
    void callbackTestPosition(const C31_PathPlanner::C31_Location::ConstPtr & msg){
		position = extractLocation(*msg);
	}

    void publishObjectLocation(ros::Publisher& topic){
    	C23_ObjectRecognition::C23C0_ODIM msg;
    	msg.Object = objectName;

    	size_t i = searchOnPathPosition(position, p);
    	if(i<=p.size()){
    		msg.x = p[i].x;
    		msg.y = p[i].y;
    		topic.publish(msg);
    	}
    }

};


#endif //_PATH_PLANNING_SERVER_H_
