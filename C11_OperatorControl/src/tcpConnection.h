#ifndef TCPCONNECTION_H
#define TCPCONNECTION_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QAbstractSocket>
#include "structs.h"

class QTcpSocket;
class QHostAddress;

class ITcpConnectionInterface
{
public:
  virtual void OnOccupancyGridReceived(int grid[100][100], StructPoint robotPos, int xOffset, int yOffset, double orient) = 0;
};

class CTcpConnection : public QObject
{
        Q_OBJECT

Q_SIGNALS:
        void SigOnImgReceived(QImage img);
        void SigOnGridReceived(int grid[100][100], StructPoint robotPos, int xOffset, int yOffset, double orient);

public Q_SLOTS:
        void SltReadyRead();
        void SltError(QAbstractSocket::SocketError socketError);
        void SltOnTimer();

public:

        CTcpConnection(QString ipAddress,int port);
        ~CTcpConnection();

        void SetSubscriber(ITcpConnectionInterface* pitcpConnectionInterface);

private:
        QTcpSocket *pConnection;
        QHostAddress *IpAddress;
        QTimer* pTimer;
        bool IsSendingImage;
        bool IsSendingGrid;
        int ImgSize;
        ITcpConnectionInterface* pITcpConnectionInterface;
};

#endif // TCPCONNECTION_H
