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
  virtual void OnPathReceived(std::vector<StructPoint> points) = 0;
  virtual void OnImgReceived(QImage image) = 0;
  virtual void OnHMIResponseReceived()=0;
  virtual void OnExecutionStatusUpdate(int status) = 0;
  virtual void OnExecuterStackUpdate(QString strQString) = 0;
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

        void LoadMission(int index);
        void Pause();
        void Resume();
        void SendPathUpdate(std::vector<StructPoint> points);

        void SendImageRequest();
        void SendGridRequest();
        void SendPathRequest();

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
        bool IsSendingPath;
        bool IsSendingExecuterStatus;
        int ImgSize;
        bool WaitingForResponse;
        int Counter;
        ITcpConnectionInterface* pITcpConnectionInterface;
};

#endif // TCPCONNECTION_H
