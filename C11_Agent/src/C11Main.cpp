#include "C11Main.h"

C11Main::C11Main(int argc, char **argv)
: QObject()
{
  qRegisterMetaType<StructGridData>("StructGridData");
  pC11Node = new C11_Agent_Node(argc, argv);
  pC11Node->init();
  pC11Node->SetAgentInterface(this);

  connect(this,SIGNAL(SigOnImageSend(QImage)),this,SLOT(SltOnImageSend(QImage)));
  connect(this,SIGNAL(SigOnGridSend(StructGridData)),this,SLOT(SltOnGridSend(StructGridData)));
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
}

void C11Main::PushImage(QImage img)
{
  emit SigOnImageSend(img);
}

void C11Main::PushGrid(StructGridData grid)
{
  emit SigOnGridSend(grid);
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
