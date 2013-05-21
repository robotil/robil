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
          void SigOnPathSend(vector<StructPoint> path);
          void SigOnHMIResponse();
          void SigOnExecutionStatusChange(int status);

  public Q_SLOTS:
    void SltOnImageSend(QImage img);
    void SltOnGridSend(StructGridData grid);
    void SltOnPathSend(vector<StructPoint> path);
    void SltOnHMIResponse();
    void SltHMIResponded();
    void SltOnExecutionStatusChange(int status);
    void SltPause();
    void SltResume();
    void SltLoadMission(int MissionId);
    void SltPathUpdated(std::vector<StructPoint> points);
    void SltImageRequest();
    void SltGridRequest();
    void SltPathRequest();

public:
  C11Main(int argc, char **argv);
  ~C11Main();

  void SetTcp(CTcpServer* ptcpServer);

  virtual void PushImage(QImage img);
  virtual void PushGrid(StructGridData grid);
  virtual void PushPath(vector<StructPoint> path);
  virtual void HMIResponse();
  virtual void ExecutionStatusChanged(int status);

private:
  C11_Agent_Node* pC11Node;
  CTcpServer* pCTcpServer;
};

#endif // C11_MAIN_H
