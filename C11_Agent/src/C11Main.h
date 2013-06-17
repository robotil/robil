#ifndef C11_MAIN_H
#define C11_MAIN_H

#include <QObject>
#include <QTimer>
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
          void SigOnSendExecuterStack(QString);
          void SigOnVRCScoreData(double timeSec, int competionScore, int falls, QString message);
          void SigOnSendDownlink(QString);
          void SigOnSendUplink(QString);
          void SigOnRobotData(StructPoint pos, StructOrientation orient);

  public Q_SLOTS:
    void SltOnImageSend(QImage img);
    void SltOnGridSend(StructGridData grid);
    void SltOnPathSend(vector<StructPoint> path);
    void SltOnHMIResponse();
    void SltHMIResponded();
    void SltOnExecutionStatusChange(int status);
    void SltOnSendExecuterStack(QString);
    void SltOnVRCScoreData(double timeSec, int competionScore, int falls, QString message);
    void SltOnSendDownlink(QString);
    void SltOnSendUplink(QString);
    void SltOnRobotData(StructPoint pos, StructOrientation orient);
    void SltPause();
    void SltResume();
    void SltLoadMission(int MissionId);
    void SltPathUpdated(std::vector<StructPoint> points);
    void SltImageRequest();
    void SltGridRequest();
    void SltPathRequest();
    void SltAllRequest();
    void SltStopRequest();
    void SltNewGoalRequest(StructPoint goal);
    void SltResetRequest();
    void SltGridAndPathRequest();
    void SltOnDataTimerTimeout();

public:
  C11Main(int argc, char **argv);
  ~C11Main();

  void SetTcp(CTcpServer* ptcpServer);
  void SetImgTcp(CTcpServer* ptcpServer);
  void SetDesignerTcp(CTcpServer* ptcpServer);

  virtual void PushImage(QImage img);
  virtual void PushGrid(StructGridData grid);
  virtual void PushPath(vector<StructPoint> path);
  virtual void HMIResponse();
  virtual void ExecutionStatusChanged(int status);
  virtual void SendExecuterStack(QString);
  virtual void SendVRCScoreData(double timeSec, int competionScore, int falls, QString message);
  virtual void SendDownlink(QString);
  virtual void SendUplink(QString);
  virtual void SendRobotData(StructPoint pos, StructOrientation orient);

private:
  C11_Agent_Node* pC11Node;
  CTcpServer* pCTcpServer;
  CTcpServer* pImageCTcpServer;
  CTcpServer* pDesignerCTcpServer;
  QTimer* DataTimer;
};

#endif // C11_MAIN_H
