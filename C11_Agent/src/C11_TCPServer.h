#ifndef C11_TCPSERVER_H
#define C11_TCPSERVER_H

#include <QObject>
#include <QImage>
#include "C11_structs.h"

class QTcpServer;
class QTcpSocket;
class QHostAddress;

class CTcpServer : public QObject
{
        Q_OBJECT

Q_SIGNALS:
        void SigConnected();

public Q_SLOTS:
        void SltOnNewConnection();
        void SltOnDisconnect();
        void SltOnDataReceived();

public:
        CTcpServer(QString ipAddress,int port, QObject *parent = NULL);
        ~CTcpServer();

        void SendImage(QImage img);
        void SendGrid(StructGridData grid);
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
};

#endif // C11_TCPSERVER_H
