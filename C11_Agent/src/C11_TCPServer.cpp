#include <QTcpServer>
#include <QTcpSocket>
#include <QHostAddress>
#include "C11_structs.h"
#include "C11_TCPServer.h"

CTcpServer::CTcpServer(QString ipAddress,int port, QObject *parent)
        : QObject(parent)
{
          IpAddress = new  QHostAddress(ipAddress);
          Port = port;
          isConnected = false;
          isWaitToContinue = false;
          MsgIdToContinue = 0;
          DataSizeToContinue = 0;
          SpareToContinue = 0;
          Counter = 0;
          pClientConnection = NULL;

          pTcpServer = new QTcpServer(this);
          if (!pTcpServer->listen(QHostAddress::Any,Port))
          {
              std::cout<<"TCP: Connection initialization fail\n";
           return;
          }
          connect(pTcpServer, SIGNAL(newConnection()), this, SLOT(SltOnNewConnection()));
          std::cout<<"TCP: TcpServer on\n";
//          if(pTcpServer->waitForNewConnection(10000))
//            {
//              std::cout<<"true\n";
//            }
//          else
//            {
//              std::cout<<"false\n";
//            }
}

CTcpServer::~CTcpServer()
{
          pTcpServer->close();
          if(IpAddress != NULL)
              {
                delete IpAddress;
              }
}

void CTcpServer::SltOnNewConnection()
{
          std::cout<<"TCP: New connection from operator\n";
          pClientConnection = pTcpServer->nextPendingConnection();
          connect(pClientConnection, SIGNAL(disconnected()),
                   this, SLOT(SltOnDisconnect()));
          connect(pClientConnection, SIGNAL(readyRead()),
                   this, SLOT(SltOnDataReceived()));
          isConnected = true;
          SendHello();

}

void CTcpServer::SltOnDisconnect()
{

}

void CTcpServer::SltOnDataReceived()
{

}

void CTcpServer::SendHello()
{
  if(NULL == pClientConnection)
  {
    std::cout<<"TCP: No connection\n";
    return;
  }
  short msgNum = 45;
//  char buff[100];
//  memcpy(buff,&msgNum,sizeof(msgNum));
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out<<msgNum;
//  for(int i=1; i<sizeof(msgNum);i++)
//          {
//                  block[i] = buff[i];
//          }
  pClientConnection->write(block);
 // pClientConnection->write("HELLO");
  std::cout<<"TCP: Hello sent\n";
}

void CTcpServer::SendImage(QImage img)
{
  std::cout<<"TCP: SendImage\n";
              if(NULL == pClientConnection)
                {
                  std::cout<<"TCP: No connection\n";
                  return;
               }
              StructHeader header;
              header.MessageID = 1;
              header.Counter = 0;
//              StructImage msg;
//              QByteArray block;
//              msg.Header.MessageID = 1;
//              msg.Header.DataSize = 0/*sizeof(img)*/;
//              msg.Header.Counter = Counter;
//              std::cout<<"TCP: msg.Header.Counter = Counter\n";
//              msg.Img = img;
//              std::cout<<"msg.Img = img\n";
//              QDataStream out(&block, QIODevice::WriteOnly);
//              out.setByteOrder(QDataStream::LittleEndian);
//              out << img;
//              msg.Header.DataSize = block.size();
//              std::cout<<"TCP: msg.Header.DataSize = " << block.size() << "\n";
//              block.clear();
//              out << msg;
//              pClientConnection->write(block);
              QByteArray block;
              QByteArray block2;
              QDataStream out(&block, QIODevice::WriteOnly);
              out.setByteOrder(QDataStream::LittleEndian);
              QDataStream test(&block2, QIODevice::WriteOnly);
              test.setByteOrder(QDataStream::LittleEndian);
//              out << msgId;
//              pClientConnection->write(block);
              test << img;
              //size = block2.size();
              header.DataSize = block2.size();
              out << header;
              std::cout<<"img size = " << header.DataSize << "\n";
//              pClientConnection->write(block);
              out << img;
              pClientConnection->write(block);
              pClientConnection->flush();
              pClientConnection->waitForBytesWritten();
              std::cout<<"TCP: Image sent\n";
}
