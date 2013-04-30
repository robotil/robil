#include "C11_Agent/C11.h"
#include "C34_Executer/run.h"
#include "C34_Executer/stop.h"
#include "C34_Executer/resume.h"
#include "C34_Executer/pause.h"
#include "C31_PathPlanner/C31_Waypoints.h"
#include <sstream>
#include <stdlib.h>
#include "C11_Agent_Node.h"

C11_Agent_Node::C11_Agent_Node(int argc, char** argv):
                  init_argc(argc),
                  init_argv(argv)
{
  pushS = NULL;
  HMIResS = NULL;
  pIAgentInterface = NULL;
  IsWaitForRelease = false;
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

    pushS = new PushHMIServer();
    HMIResS = new HMIResponseServer();
    pushS->SetPushHMIInterface(this);


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
          filename.append("/plans/skill3.xml");

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
            c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
                    C10_Common::execution_status_change srv11EST;
                    srv11EST.request.status.new_status = 0;
                    if (!c11ExecutionStatusChangeClient.call(srv11EST))
                    {
                            ROS_ERROR("running update error, exiting\n");
                    }
                    else
                    {
                            ROS_INFO("running update sent\n");
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
            c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
                    C10_Common::execution_status_change srv11EST;
                    srv11EST.request.status.new_status = 1;
                    if (!c11ExecutionStatusChangeClient.call(srv11EST))
                    {
                            ROS_ERROR("paused update error, exiting\n");
                    }
                    else
                    {
                            ROS_INFO("paused update sent\n");
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
  c11ExecutionStatusChangeClient = nh_->serviceClient<C10_Common::execution_status_change>("C11/execution_status_change");
  C10_Common::execution_status_change srv11EST;
  srv11EST.request.status.new_status = 2;
  if (!c11ExecutionStatusChangeClient.call(srv11EST))
  {
          ROS_ERROR("stop update error, exiting\n");
  }
  else
  {
          ROS_INFO("Stop update sent\n");
  }
}

void C11_Agent_Node::PushImage(QImage img)
{
  pIAgentInterface->PushImage(img);
}

void C11_Agent_Node::SetReleased()
{
  IsWaitForRelease = false;
  pushS->SetReleased();
}
