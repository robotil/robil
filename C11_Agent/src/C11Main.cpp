#include "C11Main.h"

C11Main::C11Main(int argc, char **argv)
: QObject()
{
  pC11Node = new C11_Agent_Node(argc, argv);
  pC11Node->init();
  pC11Node->SetAgentInterface(this);

  connect(this,SIGNAL(SigOnImageSend(QImage)),this,SLOT(SltOnImageSend(QImage)));
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

void C11Main::SltOnImageSend(QImage img)
{
  pCTcpServer->SendImage(img);
  pC11Node->SetReleased();
}
