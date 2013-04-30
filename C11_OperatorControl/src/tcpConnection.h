#ifndef TCPCONNECTION_H
#define TCPCONNECTION_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QAbstractSocket>

class QTcpSocket;
class QHostAddress;

class CTcpConnection : public QObject
{
        Q_OBJECT

Q_SIGNALS:
        void SigOnImgReceived(QImage img);

public Q_SLOTS:
        void SltReadyRead();
        void SltError(QAbstractSocket::SocketError socketError);
        void SltOnTimer();

public:

        CTcpConnection(QString ipAddress,int port);
        ~CTcpConnection();

private:
        QTcpSocket *pConnection;
        QHostAddress *IpAddress;
        QTimer* pTimer;
        bool IsSendingImage;
        int ImgSize;
};

#endif // TCPCONNECTION_H
