#ifndef C11_MAIN_H
#define C11_MAIN_H

#include <QObject>
#include "C11_Agent_Node.h"
#include "C11_TCPServer.h"

class C11Main: public QObject, public IAgentInterface
{
  Q_OBJECT

  Q_SIGNALS:
          void SigOnImageSend(QImage img);
          void SigOnGridSend(StructGridData grid);

  public Q_SLOTS:
    void SltOnImageSend(QImage img);
    void SltOnGridSend(StructGridData grid);

public:
  C11Main(int argc, char **argv);
  ~C11Main();

  void SetTcp(CTcpServer* ptcpServer);

  virtual void PushImage(QImage img);
  virtual void PushGrid(StructGridData grid);

private:
  C11_Agent_Node* pC11Node;
  CTcpServer* pCTcpServer;
};

#endif // C11_MAIN_H
