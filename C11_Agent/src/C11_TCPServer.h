#ifndef C11_TCPSERVER_H
#define C11_TCPSERVER_H

#include <QObject>
#include <QImage>
#include <vector>
#include "C11_structs.h"

using namespace std;

class QTcpServer;
class QTcpSocket;
class QHostAddress;

class CTcpServer : public QObject
{
        Q_OBJECT

Q_SIGNALS:
        void SigConnected();
        void SigHMIResponded();
        void SigPause();
        void SigResume();
        void SigLoadMission(int MissionId);
        void SigPathUpdated(std::vector<StructPoint> points);
        void SigImageRequest();
        void SigGridRequest();
        void SigPathRequest();

public Q_SLOTS:
        void SltOnNewConnection();
        void SltOnDisconnect();
        void SltOnDataReceived();

public:
        CTcpServer(QString ipAddress,int port, QObject *parent = NULL);
        ~CTcpServer();

        void SendImage(QImage img);
        void SendGrid(StructGridData grid);
        void SendPath(vector<StructPoint> path);
        void SendHMIResponse();
        void SendExecutionStatusChange(int status);
        void SendExecuterStack(QString str);
        void SendHello();

private:
        QTcpServer *pTcpServer;
        QTcpSocket *pClientConnection;
        QHostAddress *IpAddress;
        int Port;
        short MsgId;
        short DataSize;
        short Spare;
        bool isConnected;
        bool isWaitToContinue;
        short MsgIdToContinue;
        short DataSizeToContinue;
        short SpareToContinue;
        unsigned int Counter;
        bool IsPathComing;
};

#endif // C11_TCPSERVER_H
