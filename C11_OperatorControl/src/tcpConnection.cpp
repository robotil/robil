#include <QTcpSocket>
#include <QHostAddress>
#include "tcpConnection.h"
#include "iostream"

using namespace std;

CTcpConnection::CTcpConnection(QString ipAddress,int port)
{
  IsSendingImage = false;
  ImgSize = 0;
  IpAddress = new  QHostAddress(ipAddress);
  pConnection = new QTcpSocket(this);
  connect(pConnection, SIGNAL(readyRead()), SLOT(SltReadyRead()));
  connect(pConnection, SIGNAL(error(QAbstractSocket::SocketError)),SLOT(SltError(QAbstractSocket::SocketError)));
  std::cout<<"TCP: trying to connect!\n";
  pConnection->connectToHost(*IpAddress,port);
  pTimer = new QTimer();
  connect(pTimer, SIGNAL(timeout()), SLOT(SltOnTimer()));
//  pTimer->start(1000);
}

CTcpConnection::~CTcpConnection()
{
//  if(pConnection != NULL)
//    {
//      delete pConnection;
//      pConnection =  NULL;
//    }
  if(IpAddress != NULL)
    {
      delete IpAddress;
    }
}

void CTcpConnection::SltReadyRead()
{
  std::cout<<"TCP: Data received!\n";
  int bufSize;
  short msgId;
  unsigned int dataSize;
  unsigned int counter;
  QDataStream in(pConnection);
  in.setByteOrder(QDataStream::LittleEndian);

  while(pConnection->bytesAvailable() > 0)
    {
      std::cout<<"TCP: Bytes avaliable: " << pConnection->bytesAvailable() << "\n";
 //     pConnection->waitForReadyRead();
      bufSize = pConnection->bytesAvailable();
      if(IsSendingImage)
        {
          if(bufSize < ImgSize)
            {
              std::cout<<"TCP: Size not big enough!\n";
              std::cout<<"TCP: expected = " << ImgSize << " received = " << bufSize << "\n";
              return;
            }
          else
          {
              IsSendingImage = false;
          }
          if(!IsSendingImage)
           {
              QImage img;
           in >> img;
//           if(!img.save("received.jpg"))
//             {
//               std::cout<<"TCP: Image save failed!\n";
//             }
           std::cout<<"TCP: Image receive completed!\n";
           emit SigOnImgReceived(img);
           }
        }
      else
      { if (pConnection->bytesAvailable() < (int)sizeof(short))
        {
                return;
        }
        in >> msgId;
        if(msgId == 45)
          {
            std::cout<<"TCP: Hello Received!\n";
            return;
          }
        else
          {
   //         std::cout<<"TCP: msg id: " << msgId << "\n";
          }
        bufSize = pConnection->bytesAvailable();
        if (pConnection->bytesAvailable() < (int)sizeof(unsigned int))
        {
                return;
        }
        in >> dataSize;
        std::cout<<"TCP: image coming size = " << dataSize << "\n";
        bufSize = pConnection->bytesAvailable();
        if (pConnection->bytesAvailable() < (int)sizeof(unsigned int))
        {
                return;
        }
        in >> counter;
        if(1 == msgId)
          {
            IsSendingImage = true;
            ImgSize = dataSize;
            std::cout<<"TCP: image coming!\n";
  //          std::cout<<"TCP: image coming size = " << ImgSize << "\n";
  //         bufSize = pConnection->bytesAvailable();
  //          if(bufSize < dataSize)
  //            {
  //              return;
  //            }
  //         if(!pConnection->waitForReadyRead())
  //           {
  //
  //
  //           }
//           bufSize = pConnection->bytesAvailable();
//           if (pConnection->bytesAvailable() < (int)sizeof(int))
//             {
//               return;
//             }
//           in >> ImgSize;
//           std::cout<<"TCP: image coming size = " << ImgSize << "\n";
//           IsSendingImage = true;
  //          QImage img;
  //          in >> img;
  //          if(!img.save("received.jpg"))
  //            {
  //              std::cout<<"TCP: can't save the image!\n";
  //              return;
  //            }
  //          std::cout<<"TCP: image saved!\n";
          }
      }
    }
}

void CTcpConnection::SltError(QAbstractSocket::SocketError socketError)
{
  switch (socketError) {
  case QAbstractSocket::RemoteHostClosedError:
    std::cout<<"TCP: Connection to Agent been closed!\n";
    break;
  case QAbstractSocket::HostNotFoundError:
    std::cout<<"TCP: Agent host not found!\n";
    break;
  case QAbstractSocket::ConnectionRefusedError:
    std::cout<<"TCP: Connection refused!\n";
    break;
  default:
    std::cout<<"TCP: Connection error!\n";
    break;

  }
}

void CTcpConnection::SltOnTimer()
{
//  std::cout<<"TCP: Bytes avaliable: "<< pConnection->bytesAvailable() <<"\n";
//  std::cout<<"TCP: The socket is: "<< pConnection->state() <<"\n";
}
