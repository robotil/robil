#ifndef _C11_PUSH_SERVER_H_
#define _C11_PUSH_SERVER_H_

//tasks dependencies
#include <actionlib/server/simple_action_server.h>
#include <C0_RobilTask/RobilTask.h>
#include <C0_RobilTask/RobilTaskAction.h>
#include <C0_RobilTask/StringOperations.h>
#include "C21_VisionAndLidar/C21_Pan.h"
#include "C21_VisionAndLidar/C21_Pic.h"
//#include <cstdlib>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "C22_GroundRecognitionAndMapping/C22.h"
#include "C31_PathPlanner/C31_GetPath.h"
#include "C10_Common/push_img.h"
#include "C10_Common/push_occupancy_grid.h"
#include "C10_Common/push_path.h"
#include "C11_structs.h"
//#include "C11_TCPServer.h"
#include <QImage>

using namespace std;
using namespace C0_RobilTask;

class IPushHMIInterface
{
public:
  virtual void PushImage(QImage img) = 0;
  virtual void PushGrid(StructGridData grid) = 0;
  virtual void PushPath(vector<StructPoint> path) = 0;
};

class PushHMIServer:public RobilTask{


public:
	PushHMIServer(string name = "/PushHMI"):
    	RobilTask(name)
    {
	  pIPushHMIInterface = NULL;
	  IsWaitForRelease = false;
    }

	void SetPushHMIInterface(IPushHMIInterface *ipushHMIInterface)
	{
	  pIPushHMIInterface = ipushHMIInterface;
	  path_subscriber = _node.subscribe("/path",1000,&PushHMIServer::PathCallback,this);
	}

	void SetReleased()
	{
	  IsWaitForRelease = false;
	}

	void PathCallback(const C31_PathPlanner::C31_Waypoints& path)
	{
	  if(!Path.empty())
	    {
	      Path.clear();
	    }
	  for(int i=0; i<path.points.size(); i++)
            {
                    StructPoint point;
                    point.x = path.points[i].x;
                    point.y = path.points[i].y;
                    Path.push_back(point);
            }
	}

	TaskResult panoramic_image_task()
	{
		ROS_INFO("C11_Agent: panoramic_image_task begin!\n");
		ros::ServiceClient c21Client = _node.serviceClient<C21_VisionAndLidar::C21_Pic>("C21/Pic");

		C21_VisionAndLidar::C21_Pic srv21;
		srv21.request.req.cmd=C21_VisionAndLidar::C21_PICTURE::LEFT;
		if (!c21Client.call(srv21))
		{
			ROS_ERROR("couldn't get a picture, exiting\n");
			return TaskResult::FAULT();
		}
		ROS_INFO("C11_Agent: panoramic_image received!\n");


		cout<<"Size of vector: "<<srv21.response.res.data.size()<<"\n";
		cout<<"Width: "<<srv21.response.res.width<<"\n";
		cout<<"Height: "<<srv21.response.res.height<<"\n";
		cout<<"Step: "<<srv21.response.res.step<<"\n";
		QImage img(srv21.response.res.data.data(),srv21.response.res.width,srv21.response.res.height,QImage::Format_RGB888);
//
		if(pIPushHMIInterface != NULL)
		  {
		    IsWaitForRelease = true;
		    pIPushHMIInterface->PushImage(img);
		    while(IsWaitForRelease)
		      {
		        sleep(100);
		      }
		  }
		ROS_INFO("C11_Agent: panoramic_image sent!\n");


		 ROS_INFO("C11_Agent: panoramic_image_task end!\n");

		 return TaskResult::SUCCESS ();
	}

	TaskResult occupancy_grid_task()
	{
		ROS_INFO("C11_Agent: occupancy_grid_task begin!\n");
		ros::ServiceClient c22Client = _node.serviceClient<C22_GroundRecognitionAndMapping::C22>("C22");
		C22_GroundRecognitionAndMapping::C22 srv22;
		if (!c22Client.call(srv22))
		{
			ROS_ERROR("couldn't get a occupancy grid, exiting\n");
			return TaskResult::FAULT();
		}

		ROS_INFO("C11_Agent: occupancy_grid received!\n");

		StructGridData grid;
		grid.RobotPos.x = srv22.response.drivingPath.robotPos.x;
		grid.RobotPos.y = srv22.response.drivingPath.robotPos.y;
		grid.RobolOrientation = srv22.response.drivingPath.robotOri.z;
		grid.XOffset = srv22.response.drivingPath.xOffset;
		grid.YOffset = srv22.response.drivingPath.yOffset;
		for(int i=0; i<100; i++)
		        {
		                for(int j=0; j<100;j++)
		                {
		                        grid.Grid[i][j] = srv22.response.drivingPath.row[i].column[j].status;
		                }
		        }

		if(pIPushHMIInterface != NULL)
		  {
		    IsWaitForRelease = true;
		    pIPushHMIInterface->PushGrid(grid);
		    while(IsWaitForRelease)
                     {
                       sleep(100);
                     }
		  }

//		ros::ServiceClient c11Client = _node.serviceClient<C10_Common::push_occupancy_grid>("C11/push_occupancy_grid");
//
//		C10_Common::push_occupancy_grid srv11;
//
//		srv11.request.OGD = srv22.response.drivingPath;
//		if (!c11Client.call(srv11))
//		{
//			ROS_ERROR("couldn't get a C11 reply, exiting\n");
//			return TaskResult::FAULT();
//
//		}

		ROS_INFO("C11_Agent: occupancy_grid sent!\n");

//		 if (srv11.response.ACK.mes != 1) {
//			ROS_ERROR("C11 ack is fault, exiting\n");
//			return TaskResult::FAULT();
//		 }

		ROS_INFO("C11_Agent: occupancy_grid_task end!\n");
		return TaskResult::SUCCESS ();
	}

	TaskResult path_task()
	{
		ROS_INFO("C11_Agent: path_task begin!\n");
//		ros::ServiceClient c31Client = _node.serviceClient<C31_PathPlanner::C31_GetPath>("C31_GlobalPathPlanner/getPath");
//		C31_PathPlanner::C31_GetPath srv31;
//		if (!c31Client.call(srv31))
//		{
//			ROS_ERROR("couldn't get a path, exiting\n");
//			return TaskResult::FAULT();
//		}
//
//		ROS_INFO("C11_Agent: path received!\n");
//		cout<<srv31.response.path<<"\n";

//		ros::ServiceClient c11Client = _node.serviceClient<C10_Common::push_path>("C11/push_path");
//
//		C10_Common::push_path srv11;
//
//		srv11.request.PTH = srv31.response.path;
//		if (!c11Client.call(srv11))
//		{
//			ROS_ERROR("couldn't get a C11 reply, exiting\n");
//			return TaskResult::FAULT();
//
//		}
//		vector<StructPoint> path;
//		for(int i=0; i<srv31.response.path.points.size(); i++)
//                {
//                        StructPoint point;
//                        point.x = srv31.response.path.points[i].x;
//                        point.y = srv31.response.path.points[i].y;
//                        path.push_back(point);
//                }
		if(pIPushHMIInterface != NULL)
                {
                  IsWaitForRelease = true;
                  pIPushHMIInterface->PushPath(Path);
                  while(IsWaitForRelease)
                    {
                      sleep(100);
                    }
                }

		ROS_INFO("C11_Agent: path sent!\n");

//		if (srv11.response.ACK.mes != 1) {
//			ROS_ERROR("C11 ack is fault, exiting\n");
//			return TaskResult::FAULT();
//		 }

		ROS_INFO("C11_Agent: path_task end!\n");
		return TaskResult::SUCCESS ();
	}

    TaskResult task(const string& name, const string& uid, Arguments& args){

    	ROS_INFO("C11_Agent: PushHMI called!\n");
    	if(!args.empty())
    	{
    		ROS_INFO("%s: %s\n","data",args["data"].data());
    		string str = args["data"];
    		if(!str.empty())
    		{
    			if(str == "panoramic_image")
    			{
    				ROS_INFO("panoramic_image BINGO");
    				return panoramic_image_task();
    			}
    			else if(str == "occupancy_grid")
				{
    				ROS_INFO("occupancy_grid BINGO");
    				return occupancy_grid_task();
				}
    			else if(str == "path")
				{
					ROS_INFO("path BINGO");
					return path_task();
				}
    			else if(str == "cabin_image")
                                {
    			                return TaskResult::SUCCESS ();
                                }
    			else if(str == "cabin_parametrics")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "inSideCabin_image")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "InSideCabin_parametrics")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "vehicle_parametrics")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "door_image")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "door_parameters")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    			else if(str == "objects")
                                {
                                        return TaskResult::SUCCESS ();
                                }
    		}
    	}
    	return TaskResult::FAULT();
    }

private:
 //   CTcpServer* pCTcpServer;
    IPushHMIInterface *pIPushHMIInterface;
    bool IsWaitForRelease;
    ros::Subscriber path_subscriber;
    vector<StructPoint> Path;
};


#endif //_C11_PUSH_SERVER_
