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

#include <vector>

using namespace std;
using namespace C0_RobilTask;

#include "../Vec2d.hpp"


vector<Vec2d> path;
boost::mutex path_mtx;

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

			if(objectName == "QualP1S1"){
				vector<Vec2d> p;
				p.push_back(Vec2d(1,0));
				p.push_back(Vec2d(2,0));
				p.push_back(Vec2d(3,0));
				p.push_back(Vec2d(4,0));
				p.push_back(Vec2d(5,0));

				copyToPath(p);
			}
			else
			if(objectName == "QualP1S2"){
				vector<Vec2d> p;
				p.push_back(Vec2d(1,1));
				p.push_back(Vec2d(2,2));
				p.push_back(Vec2d(3,3));
				p.push_back(Vec2d(4,4));
				p.push_back(Vec2d(5,5));

				copyToPath(p);
			}
			else
			if(objectName == "QualP1S3"){
				vector<Vec2d> p;
				p.push_back(Vec2d(1,2));
				p.push_back(Vec2d(3,4));
				p.push_back(Vec2d(5,6));
				p.push_back(Vec2d(7,8));

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


#endif //_PATH_PLANNING_SERVER_H_
