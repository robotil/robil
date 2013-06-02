#include "C11_Agent/C11.h"
#include "C34_Executer/run.h"
#include "C34_Executer/stop.h"
#include "C34_Executer/resume.h"
#include "C34_Executer/pause.h"
#include "C31_PathPlanner/C31_Waypoints.h"
#include <sstream>
#include <stdlib.h>
#include "tinyxml2.h"
#include "C11_Agent_Node.h"

using namespace tinyxml2;

C11_Agent_Node::C11_Agent_Node(int argc, char** argv):
                  init_argc(argc),
                  init_argv(argv)
{
  pushS = NULL;
  HMIResS = NULL;
  pIAgentInterface = NULL;
  IsWaitForRelease = false;
  start_pos = 0;
  std::string filepath;
  filepath = ros::package::getPath("C11_OperatorControl");
  filepath.append("/bin/Missions.txt");
  QFile missfile(filepath.data());
  if (!missfile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      std::cout << "Can't open Missions config file!!! Restart the application" << std::endl;
    }
  else
    {
      while (!missfile.atEnd())
       {
          QString line = missfile.readLine();
          line.remove(QChar('\n'));
          MissionsList.append(line);
       }
    }
}
C11_Agent_Node::~C11_Agent_Node()
{
  if(pushS != NULL)
    {
      delete pushS;
      pushS = NULL;
    }
  if(HMIResS != NULL)
      {
        delete HMIResS;
        HMIResS = NULL;
      }

}

//void C11_Agent_Node::SetTcp(CTcpServer* ptcpServer)
//{
//  pCTcpServer = ptcpServer;
//}

void C11_Agent_Node::SetAgentInterface(IAgentInterface* piAgentInterface)
{
  pIAgentInterface = piAgentInterface;
}

bool C11_Agent_Node::init()
{
  tree_id_str="";
  ros::init(init_argc, init_argv, "C11_Agent");
    if ( ! ros::master::check() ) {
                    return false;
            }
    nh_ = new ros::NodeHandle();

    path_update_pub = nh_->advertise<C31_PathPlanner::C31_Waypoints>("c11_path_update",10000);
    service_MissionSelection = nh_->advertiseService("MissionSelection", &C11_Agent_Node::MissionSelection,this);
    service_PauseMission = nh_->advertiseService("PauseMission", &C11_Agent_Node::PauseMission,this);
    service_ResumeSelection = nh_->advertiseService("ResumeMission", &C11_Agent_Node::ResumeMission,this);
    service_PathUpdate = nh_->advertiseService("PathUpdate", &C11_Agent_Node::PathUpdate,this);
    robot_pos_subscriber = nh_->subscribe("C25/publish",1000,&C11_Agent_Node::RobotPosUpdateCallback,this);


    pushS = new PushHMIServer();
    HMIResS = new HMIResponseServer();
    pushS->SetPushHMIInterface(this);
    HMIResS->SetHMIResponseInterface(this);


    start();
    return true;
}

void C11_Agent_Node::run()
{
  ros::Rate loop_rate(10);
  int count = 0;
  while ( ros::ok() ) {
          ros::spinOnce();
          loop_rate.sleep();
          ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
}

bool C11_Agent_Node::MissionSelection(C10_Common::mission_selection::Request& req,
                C10_Common::mission_selection::Response& res)
{
          res.MES.mes = 1;
          int test = req.MSN.MSN;


          c34RunClient = nh_->serviceClient<C34_Executer::run>("executer/run");

          C34_Executer::run srv34Run;

   //     std::string ostr;
          std::stringstream out;
          out <<"id"<< req.MSN.MSN << std::endl;
          tree_id_str = out.str();
          ROS_INFO(tree_id_str.data());
          srv34Run.request.tree_id = tree_id_str;
          //srv34.request.tree_id << req.MSN.MSN << std::endl;

          //ostr << "skill3.xml";
          std::string filename;
          filename = ros::package::getPath("C34_Designer");
          filename.append("/plans/");
          filename.append(MissionsList.at(test).toStdString());

          srv34Run.request.filename = filename;
  //      srv34.request.req.filename << "skill3.xml";

          if (!c34RunClient.call(srv34Run))
          {
                  ROS_ERROR("send of mission error, exiting\n");
                  return false;
          }

     ROS_INFO("send of mission success");

     std::string str = srv34Run.response.output;
     ROS_INFO(str.data());

     c34ResumeClient = nh_->serviceClient<C34_Executer::resume>("executer/resume");
     C34_Executer::resume srv34Resume;
     srv34Resume.request.tree_id = tree_id_str;
     if (!c34ResumeClient.call(srv34Resume))
          {
                  ROS_ERROR("resume of mission error, exiting\n");
                  return false;
          }
     ROS_INFO("resume of mission success");

     status_subscriber = nh_->subscribe("executer/stop_stream",1000,&C11_Agent_Node::StopExecuteMessageCallback,this);

     return true;
}

bool C11_Agent_Node::PauseMission(C10_Common::pause_mission::Request& req,
                C10_Common::pause_mission::Response& res)
{
  ROS_INFO("pause request received\n");

    ros::ServiceClient c34PauseClient = nh_->serviceClient<C34_Executer::pause>("executer/pause");

    C34_Executer::pause srv34Pause;
    srv34Pause.request.tree_id = tree_id_str;
    if (!c34PauseClient.call(srv34Pause))
    {
            ROS_ERROR("pause of mission error, exiting\n");
//            c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
//                    C10_Common::execution_status_change srv11EST;
//                    srv11EST.request.status.new_status = 0;
//                    if (!c11ExecutionStatusChangeClient.call(srv11EST))
//                    {
//                            ROS_ERROR("running update error, exiting\n");
//                    }
//                    else
//                    {
//                            ROS_INFO("running update sent\n");
//                    }
            if(pIAgentInterface != NULL)
              {
                pIAgentInterface->ExecutionStatusChanged(0);
              }
            return false;
    }
}

bool C11_Agent_Node::ResumeMission(C10_Common::resume_mission::Request& req,
                C10_Common::resume_mission::Response& res)
{
  ROS_INFO("resume request received\n");

    ros::ServiceClient c34ResumeClient = nh_->serviceClient<C34_Executer::resume>("executer/resume");

    C34_Executer::resume srv34Resume;
    srv34Resume.request.tree_id = tree_id_str;
    if (!c34ResumeClient.call(srv34Resume))
    {
            ROS_ERROR("resume of mission error, exiting\n");
//            c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
//                    C10_Common::execution_status_change srv11EST;
//                    srv11EST.request.status.new_status = 1;
//                    if (!c11ExecutionStatusChangeClient.call(srv11EST))
//                    {
//                            ROS_ERROR("paused update error, exiting\n");
//                    }
//                    else
//                    {
//                            ROS_INFO("paused update sent\n");
//                    }
            if(pIAgentInterface != NULL)
              {
                pIAgentInterface->ExecutionStatusChanged(1);
              }
            return false;
    }
}

bool C11_Agent_Node::PathUpdate(C10_Common::path_update::Request& req,
                C10_Common::path_update::Response& res)
{
  ROS_INFO("path update received\n");
  path_update_pub.publish(req.PTH);
  res.ACK.mes = 1;
  return true;
}

void C11_Agent_Node::StopExecuteMessageCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO(msg->data.data());
  if(!tree_id_str.empty())
  c34StopClient = nh_->serviceClient<C34_Executer::stop>("executer/stop");
  C34_Executer::stop srv34Stop;
  srv34Stop.request.tree_id = tree_id_str;
  if (!c34StopClient.call(srv34Stop))
  {
          ROS_ERROR("stop of mission error, exiting\n");
  }
  else
  {
          ROS_INFO("Stop request sent\n");
  }
//  c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
//  C10_Common::execution_status_change srv11EST;
//  srv11EST.request.status.new_status = 2;
//  if (!c11ExecutionStatusChangeClient.call(srv11EST))
//  {
//          ROS_ERROR("stop update error, exiting\n");
//  }
//  else
//  {
//          ROS_INFO("Stop update sent\n");
//  }
  if(pIAgentInterface != NULL)
  {
    pIAgentInterface->ExecutionStatusChanged(2);
  }
}

void C11_Agent_Node::RobotPosUpdateCallback(const C25_GlobalPosition::C25C0_ROP& robot_pos)
  {
    position.x = robot_pos.pose.pose.pose.position.x;
    position.y = robot_pos.pose.pose.pose.position.y;
    CheckPath();
  }

void C11_Agent_Node::PushImage(QImage img)
{
  pIAgentInterface->PushImage(img);
}

void C11_Agent_Node::PushGrid(StructGridData grid)
{
  pIAgentInterface->PushGrid(grid);
}

void C11_Agent_Node::PushPath(vector<StructPoint> path)
{
  pIAgentInterface->PushPath(path);
}

void C11_Agent_Node::HMIResponse()
{
  pIAgentInterface->HMIResponse();
}

void C11_Agent_Node::SetReleased()
{
  IsWaitForRelease = false;
  pushS->SetReleased();
}

void C11_Agent_Node::HMIResponded()
{
  if(HMIResS != NULL)
  {
      HMIResS->ResponseReceived();
  }
}

void C11_Agent_Node::Pause()
{
  ROS_INFO("pause request received\n");

  ros::ServiceClient c34PauseClient = nh_->serviceClient<C34_Executer::pause>("executer/pause");

  C34_Executer::pause srv34Pause;
  srv34Pause.request.tree_id = tree_id_str;
  if (!c34PauseClient.call(srv34Pause))
  {
    ROS_ERROR("pause of mission error, exiting\n");
    if(pIAgentInterface != NULL)
      {
        pIAgentInterface->ExecutionStatusChanged(0);
      }
  }
}

void C11_Agent_Node::Resume()
{
  ROS_INFO("resume request received\n");

  ros::ServiceClient c34ResumeClient = nh_->serviceClient<C34_Executer::resume>("executer/resume");

  C34_Executer::resume srv34Resume;
  srv34Resume.request.tree_id = tree_id_str;
  if (!c34ResumeClient.call(srv34Resume))
  {
          ROS_ERROR("resume of mission error, exiting\n");
          if(pIAgentInterface != NULL)
          {
            pIAgentInterface->ExecutionStatusChanged(1);
          }
          return;
  }
}

void C11_Agent_Node::LoadMission(int missionId)
{
  c34RunClient = nh_->serviceClient<C34_Executer::run>("executer/run");
  C34_Executer::run srv34Run;

//  std::stringstream out;
//  out <<"id"<< missionId << std::endl;
//  tree_id_str = out.str();
//  ROS_INFO(tree_id_str.data());
//  srv34Run.request.tree_id = tree_id_str;

  std::string filename;
  filename = ros::package::getPath("C34_Designer");

  filename.append("/plans/");
  filename.append(MissionsList.at(missionId).toStdString());

  srv34Run.request.filename = filename;
  std::cout << "The task is:" << filename<< std::endl;

  XMLDocument doc;
  doc.LoadFile( filename.data());
  if(doc.ErrorID())
    {
      ROS_ERROR("can't open the the plan file, exiting\n");
//      return;
    }
  XMLElement* titleElement = doc.FirstChildElement();
  if(titleElement != NULL)
    {
      XMLElement* firstElement = titleElement->FirstChildElement();
      if(firstElement == NULL)
        {
          ROS_ERROR("the plan file is incorrect, exiting\n");
                          return;
        }
      const char* value = firstElement->Attribute( "id" );
      if(value != NULL)
        {
          std::string strid(value);
          srv34Run.request.tree_id = strid;
          tree_id_str = strid;
        }
      else
        {
          ROS_ERROR("the plan file is incorrect, id not found , exiting\n");
                return;
        }
    }
  else
    {
      ROS_ERROR("the plan file is incorrect, exiting\n");
            return;
    }

  if (!c34RunClient.call(srv34Run))
  {
          ROS_ERROR("send of mission error, exiting\n");
          return;
  }

  ROS_INFO("send of mission success");

  std::string str = srv34Run.response.output;
  ROS_INFO(str.data());

  c34ResumeClient = nh_->serviceClient<C34_Executer::resume>("executer/resume");
  C34_Executer::resume srv34Resume;
  srv34Resume.request.tree_id = tree_id_str;
  if (!c34ResumeClient.call(srv34Resume))
      {
              ROS_ERROR("resume of mission error, exiting\n");
              return;
      }
  ROS_INFO("resume of mission success");

  status_subscriber = nh_->subscribe("executer/stop_stream",1000,&C11_Agent_Node::StopExecuteMessageCallback,this);
}

void C11_Agent_Node::PathUpdated(std::vector<StructPoint> points)
{
  ROS_INFO("path update received\n");
  C31_PathPlanner::C31_Location location;
  C31_PathPlanner::C31_Waypoints waypoints;
  Vec2d pos;
//  UpdatedPath = points;
  for(int i=0; i<points.size(); i++)
  {
    location.x = points[i].x;
    location.y = points[i].y;
    pos.x = points[i].x;
    pos.x = points[i].y;
    UpdatedPath.push_back(pos);
    waypoints.points.push_back(location);
  }
  start_pos = 0;
  path_update_pub.publish(waypoints);
}

void C11_Agent_Node::ImageRequest()
{
  pushS->panoramic_image_task();
}

void C11_Agent_Node::GridRequest()
{
  pushS->occupancy_grid_task();
}

void C11_Agent_Node::PathRequest()
{
  pushS->path_task();
}

void C11_Agent_Node::CheckPath()
{
  size_t size_of_path = UpdatedPath.size();
  if(size_of_path>0)
    {
        size_t start = searchOnPathPosition(position, UpdatedPath);
        if(start_pos != start)
          {
            C31_PathPlanner::C31_Waypoints waypoints;
            for( size_t i=start;i<size_of_path;i++ )
            {
              C31_PathPlanner::C31_Location loc;
              loc.x=UpdatedPath[i].x;
              loc.y=UpdatedPath[i].y;
              waypoints.points.push_back(loc);
            }
            path_update_pub.publish(waypoints);
          }
     }
}

size_t C11_Agent_Node::searchOnPathPosition(const Vec2d& pos, const vector<Vec2d>& path)
{
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
