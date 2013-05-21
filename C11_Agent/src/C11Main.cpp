#include "C11Main.h"

C11Main::C11Main(int argc, char **argv)
: QObject()
{
  qRegisterMetaType<StructGridData>("StructGridData");
  qRegisterMetaType< vector<StructPoint> >("vector<StructPoint>");
  pC11Node = new C11_Agent_Node(argc, argv);
  pC11Node->init();
  pC11Node->SetAgentInterface(this);

  connect(this,SIGNAL(SigOnImageSend(QImage)),this,SLOT(SltOnImageSend(QImage)));
  connect(this,SIGNAL(SigOnGridSend(StructGridData)),this,SLOT(SltOnGridSend(StructGridData)));
  connect(this,SIGNAL(SigOnPathSend(vector<StructPoint>)),this,SLOT(SltOnPathSend(vector<StructPoint>)));
  connect(this,SIGNAL(SigOnHMIResponse()),this,SLOT(SltOnHMIResponse()));
  connect(this,SIGNAL(SigOnExecutionStatusChange(int)),this,SLOT(SltOnExecutionStatusChange(int)));
}

C11Main::~C11Main()
{
  if(pC11Node != NULL)
    {
      delete pC11Node;
      pC11Node =  NULL;
    }
}

void C11Main::SetTcp(CTcpServer* ptcpServer)
{
  pCTcpServer = ptcpServer;
  connect(pCTcpServer,SIGNAL(SigHMIResponded()),this,SLOT(SltHMIResponded()));
  connect(pCTcpServer,SIGNAL(SigPause()),this,SLOT(SltPause()));
  connect(pCTcpServer,SIGNAL(SigResume()),this,SLOT(SltResume()));
  connect(pCTcpServer,SIGNAL(SigLoadMission(int)),this,SLOT(SltLoadMission(int)));
  connect(pCTcpServer,SIGNAL(SigPathUpdated(std::vector<StructPoint>)),this,SLOT(SltPathUpdated(std::vector<StructPoint>)));
  connect(pCTcpServer,SIGNAL(SigImageRequest()),this,SLOT(SltImageRequest()));
  connect(pCTcpServer,SIGNAL(SigGridRequest()),this,SLOT(SltGridRequest()));
  connect(pCTcpServer,SIGNAL(SigPathRequest()),this,SLOT(SltPathRequest()));
}

void C11Main::PushImage(QImage img)
{
  emit SigOnImageSend(img);
}

void C11Main::PushGrid(StructGridData grid)
{
  emit SigOnGridSend(grid);
}

void C11Main::PushPath(vector<StructPoint> path)
{
  emit SigOnPathSend(path);
}

void C11Main::HMIResponse()
{
  emit SigOnHMIResponse();
}

void C11Main::ExecutionStatusChanged(int status)
{
  emit SigOnExecutionStatusChange(status);
}

void C11Main::SltOnImageSend(QImage img)
{
  pCTcpServer->SendImage(img);
  pC11Node->SetReleased();
}

void C11Main::SltOnGridSend(StructGridData grid)
{
  pCTcpServer->SendGrid(grid);
  pC11Node->SetReleased();
}

void C11Main::SltOnPathSend(vector<StructPoint> path)
{
  pCTcpServer->SendPath(path);
  pC11Node->SetReleased();
}

void C11Main::SltOnHMIResponse()
{
  pCTcpServer->SendHMIResponse();
}

void C11Main::SltOnExecutionStatusChange(int status)
{
  pCTcpServer->SendExecutionStatusChange(status);
}

void C11Main::SltHMIResponded()
{
  pC11Node->HMIResponded();
}

void C11Main::SltPause()
{
  pC11Node->Pause();
}

void C11Main::SltResume()
{
  pC11Node->Resume();
}

void C11Main::SltLoadMission(int MissionId)
{
  pC11Node->LoadMission(MissionId);
}

void C11Main::SltPathUpdated(std::vector<StructPoint> points)
{
  pC11Node->PathUpdated(points);
}

void C11Main::SltImageRequest()
{
  pC11Node->ImageRequest();
}

void C11Main::SltGridRequest()
{
  pC11Node->GridRequest();
}

void C11Main::SltPathRequest()
{
  pC11Node->PathRequest();
}
