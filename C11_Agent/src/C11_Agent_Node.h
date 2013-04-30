#ifndef C11_AGENT_NODE_H
#define C11_AGENT_NODE_H

#include "C10_Common/mission_selection.h"
#include "C10_Common/pause_mission.h"
#include "C10_Common/resume_mission.h"
#include "C10_Common/execution_status_change.h"
#include "C10_Common/path_update.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "C11_PushServer.hpp"
#include "C11_HMIResponseServer.hpp"
#include "C11_UntilOperatorIntervention.hpp"
#include "C11_TCPServer.h"
#include <QThread>

class IAgentInterface
{
public:
  virtual void PushImage(QImage img) = 0;
};

class C11_Agent_Node : public QThread, public IPushHMIInterface
{
  Q_OBJECT

public:
  C11_Agent_Node(int argc, char** argv);
  ~C11_Agent_Node();

  void run();
  bool init();

//  void SetTcp(CTcpServer* ptcpServer);
  void SetAgentInterface(IAgentInterface* piAgentInterface);

  bool MissionSelection(C10_Common::mission_selection::Request& req,
                  C10_Common::mission_selection::Response& res);
  bool PauseMission(C10_Common::pause_mission::Request& req,
                  C10_Common::pause_mission::Response& res);
  bool ResumeMission(C10_Common::resume_mission::Request& req,
                  C10_Common::resume_mission::Response& res);
  bool PathUpdate(C10_Common::path_update::Request& req,
                  C10_Common::path_update::Response& res);
  void StopExecuteMessageCallback(const std_msgs::StringConstPtr& msg);

  void SetReleased();

  virtual void PushImage(QImage img);

private:
  ros::NodeHandle *nh_;
  ros::Publisher path_update_pub;
  ros::ServiceServer service_MissionSelection;
  ros::ServiceServer service_PauseMission;
  ros::ServiceServer service_ResumeSelection;
  ros::ServiceServer service_PathUpdate;
  ros::Subscriber status_subscriber;
  ros::ServiceClient c34StopClient;
  ros::ServiceClient c11ExecutionStatusChangeClient;
  ros::ServiceClient c34RunClient;
  ros::ServiceClient c34ResumeClient;

  int init_argc;
  char** init_argv;
  std::string tree_id_str;

  PushHMIServer *pushS;
  HMIResponseServer *HMIResS;
  IAgentInterface* pIAgentInterface;
  bool IsWaitForRelease;
};

#endif // C11_AGENT_NODE_H
